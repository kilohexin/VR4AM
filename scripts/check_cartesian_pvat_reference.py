"""Offline Cartesian reference regression, optionally anchored to a frozen log.

Never imports an SDK, starts Docker or emits motion. Native interpolation and
real-time feasibility remain unverified even if the offline cases pass.
"""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
sys.path.insert(0,str(ROOT/'scripts'))
from audit_pose_trace import analyze as audit_trace
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import (JointReference,MdhReferenceModel,OfflineCartesianFollower,
    plan_cartesian_reference,plan_cartesian_braking)
from app.control.vr_pose_reference import VRPoseReference
from app.control.cartesian_reference_bundle import OfflineReferenceSupervisor
from app.schemas.messages import Pose


def origin(path,model):
    if path is None:return JointReference((0.,-.8,1.4,-.6,1.57,0.)),None
    raw=path.read_bytes();anchor=None
    for line in raw.decode('utf-8').splitlines():
        r=json.loads(line)
        if r.get('kind')=='robot_kinematics':anchor=r
        if r.get('kind')=='finite_position_input' and r.get('event')=='held' and r.get('hand_pose'):
            if anchor is None:raise ValueError('offline_reference_missing_anchor')
            reference=JointReference(tuple(anchor['actual_q']))
            p=model.pose(reference.q);native=anchor['actual_tcp']
            position_error=float(np.linalg.norm(np.array(p.p)-native['p']))
            angle_error=float((Rotation.from_quat(p.q)*Rotation.from_quat(native['q']).inv()).magnitude())
            if position_error>1e-6 or angle_error>1e-5:raise ValueError('offline_reference_recorded_model_mismatch')
            return reference,dict(sha256=hashlib.sha256(raw).hexdigest(),
                recorded_fk_error_m=position_error,recorded_fk_error_rad=angle_error,
                origin_rates='nominal_zero_for_math_only')
    raise ValueError('offline_reference_missing_held_input')


def summary(plan,model):
    maximum=0.;height=0.
    first=plan.curve.state(0.)[0]
    for t in np.linspace(0.,plan.curve.duration,201):
        p=model.pose(plan.state(float(t)).q)
        expected=plan.curve.state(float(t))[0]
        maximum=max(maximum,float(np.linalg.norm(np.array(p.p)-expected.p)))
        height=max(height,abs(p.p[2]-first.p[2]))
    return dict(duration_s=plan.curve.duration,segments=len(plan.segments),
        sampled_reference_error_m=maximum,sampled_height_change_m=height,
        interval_position_bound_m=max(c['position_bound_m'] for c in plan.checks),
        interval_angle_bound_rad=max(c['angle_bound_rad'] for c in plan.checks),
        joint_speed_bound_radps=max(c['joint_speed_bound_radps'] for c in plan.checks),
        joint_acceleration_bound_radps2=max(c['joint_acceleration_bound_radps2'] for c in plan.checks),
        joint_jerk_bound_radps3=max(c['joint_jerk_bound_radps3'] for c in plan.checks),
        tcp_speed_bound_mps=max(c['tcp_speed_bound_mps'] for c in plan.checks),
        tcp_acceleration_bound_mps2=max(c['tcp_acceleration_bound_mps2'] for c in plan.checks),
        angular_speed_bound_radps=max(c['angular_speed_bound_radps'] for c in plan.checks),
        angular_acceleration_bound_radps2=max(c['angular_acceleration_bound_radps2'] for c in plan.checks))


def run(frozen_log=None,*,large_pitch=False):
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start,source=origin(frozen_log,model);base=model.pose(start.q)
    def pitch(angle):
        return base.model_copy(update={'q':tuple((Rotation.from_quat(base.q)*Rotation.from_euler('x',angle)).as_quat())})
    before=time.perf_counter()
    report=dict(offline_candidate_passed=False,sdk_connections=0,motion_writes=0,
        motion_safety_verified=False,native_interpolation_verified=False,realtime_verified=False,
        interpolation='quintic_hermite_assumed',model='published_theoretical_mdh_zero_tool',
        source=source,scenario='large_pitch' if large_pitch else 'modest_pitch')
    angle=1. if large_pitch else .25;duration=18. if large_pitch else 4.
    try:
        first=plan_cartesian_reference(model,start,pitch(angle),duration=duration)
        report['pitch']=summary(first,model)
        moving=first.state(duration/2)
        reverse=plan_cartesian_reference(model,moving,pitch(-angle),duration=28. if large_pitch else 10.)
        report['nonzero_rate_reversal']=summary(reverse,model)
        report['offline_candidate_passed']=True
    except ValueError as error:
        if not str(error).startswith('cartesian_reference_'):raise
        report['rejection']=str(error)
    report['planning_wall_s']=time.perf_counter()-before
    return report


def replay(frozen_log,*,response_frequency=4.,reference_trace=None,check_braking=False,demand_mode='coupled',braking_stride=None,
        supervised=False,governed_continuation=False):
    """Counterfactual math only: corrected mapping, one new reference/80ms.

    This replays one continuous held gesture, not the original native PVAT
    outputs. Reject multiple gestures rather than inventing recovery behavior.
    """
    if braking_stride is not None and (not check_braking or type(braking_stride) is not int or braking_stride<=0):
        raise ValueError('offline_reference_braking_stride')
    if governed_continuation and not supervised:raise ValueError('offline_reference_supervisor_required')
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start,source=origin(frozen_log,model);base=model.pose(start.q)
    rows=[json.loads(line) for line in frozen_log.read_text(encoding='utf-8').splitlines()]
    held=[];ended=False
    for row in rows:
        if row.get('kind')!='finite_position_input':continue
        if row.get('event')=='held' and row.get('hand_pose'):
            if ended:raise ValueError('offline_reference_multiple_gestures')
            held.append(row)
        elif held:ended=True
    if not held:raise ValueError('offline_reference_missing_held_input')
    stamps=[h['server_mono_ns'] for h in held]
    now=[stamps[0]];mapper=VRPoseReference(clock=lambda:now[0],translation_scale=1.,orientation_mode='tool')
    mapper.capture(Pose(**held[0]['hand_pose']),base,generation=1,hand_ns=now[0],tcp_ns=now[0])
    factory=OfflineReferenceSupervisor if supervised else OfflineCartesianFollower
    options=dict(frequency=response_frequency,demand_mode=demand_mode)
    if supervised:options['continue_on_known_rejection']=governed_continuation
    follower=factory(model,start,**options)
    report=dict(offline_candidate_passed=False,sdk_connections=0,motion_writes=0,source=source,
        scenario='frozen_hand_counterfactual_corrected_mapping',native_interpolation_verified=False,
        motion_safety_verified=False,realtime_verified=False,product_admission_complete=False,
        completed_steps=0,max_downward_goal_error_m=0.,max_interval_position_bound_m=0.,
        max_step_wall_s=0.,model_joint_bounds_rad=[-3.,3.],response_frequency=response_frequency)
    report['reference_sha256']=hashlib.sha256((ROOT/'backend/app/control/cartesian_pvat_reference.py').read_bytes()).hexdigest()
    report['lookahead_position_m']=getattr(follower,'lookahead_position',.02)
    report['lookahead_rotation_rad']=getattr(follower,'lookahead_rotation',.08)
    report['demand_mode']=demand_mode
    report['direct_quadratic_steps']=0
    report['supervised_model_bundles']=supervised
    report['backoff_steps']=0;report['backup_braking_steps']=0
    report['governed_deceleration_steps']=0
    if supervised:report['supervisor_sha256']=hashlib.sha256((ROOT/'backend/app/control/cartesian_reference_bundle.py').read_bytes()).hexdigest()
    before=time.perf_counter();period_ns=80_000_000
    step_times=[];brake_samples=[]
    observations=[dict(kind='robot_kinematics',server_mono_ns=stamps[0]-1,actual_tcp=base.model_dump())]
    total_steps=(stamps[-1]-stamps[0])//period_ns
    for tick in range(total_steps):
        if supervised and follower.phase=='ready':
            report['rejection']='cartesian_reference_bundle_completed_early';report['rejection_at_s']=tick*.08;break
        now[0]=stamps[0]+tick*period_ns
        sample=held[bisect.bisect_right(stamps,now[0])-1]
        step_before=time.perf_counter()
        try:
            target=mapper.target(Pose(**sample['hand_pose']),generation=1,seq=tick,captured_ns=sample['server_mono_ns'])
            p=tuple(float(x) for x in np.clip(np.array(target.p),np.array(base.p)-.3,np.array(base.p)+.3))
            delta=(Rotation.from_quat(target.q)*Rotation.from_quat(base.q).inv()).as_rotvec()
            if np.linalg.norm(delta)>np.pi/2-.02:delta*=((np.pi/2-.02)/np.linalg.norm(delta))
            q=(Rotation.from_rotvec(delta)*Rotation.from_quat(base.q)).as_quat()
            target=Pose(p=p,q=tuple(float(x) for x in q))
            if supervised:
                packet=follower.advance(target);segment=packet.segment
                check=dict(optimizer_iterations=packet.optimizer_iterations,position_bound_m=packet.position_bound_m)
                report['backoff_steps']+=int(packet.mode=='following' and packet.demand_scale<1.)
                report['backup_braking_steps']+=int(packet.mode=='braking')
                report['governed_deceleration_steps']+=int(packet.mode=='governed_deceleration')
                if packet.mode=='braking':report.setdefault('bundle_rejections',follower.last_rejections)
            else:segment,check=follower.advance(target)
        except ValueError as error:
            if not str(error).startswith(('cartesian_reference_','vr_pose_')):raise
            report['rejection']=str(error);report['rejection_at_s']=tick*.08
            if hasattr(error,'optimizer_status'):
                report['optimizer_diagnostics']={name:getattr(error,name) for name in
                    ('optimizer_status','minimum_reserve','minimum_task_margin')}
            break
        report['completed_steps']+=1
        report['direct_quadratic_steps']+=int(check['optimizer_iterations']==0)
        elapsed=time.perf_counter()-step_before;step_times.append(elapsed)
        report['max_step_wall_s']=max(report['max_step_wall_s'],elapsed)
        selected=(tick in (10,100,400,800,1200,1600) if braking_stride is None
            else tick%braking_stride==0 or tick==total_steps-1)
        if check_braking and selected:brake_samples.append((tick,segment.end))
        report['max_interval_position_bound_m']=max(report['max_interval_position_bound_m'],check['position_bound_m'])
        position=model.pose(segment.end.q)
        observations.append(dict(kind='robot_kinematics',server_mono_ns=now[0]+period_ns,actual_tcp=position.model_dump()))
        report['max_downward_goal_error_m']=max(report['max_downward_goal_error_m'],target.p[2]-position.p[2])
    else:report['offline_candidate_passed']=True
    if supervised and report['backup_braking_steps']:
        report['offline_candidate_passed']=False
        report.setdefault('rejection','cartesian_reference_bundle_rejected_during_follow')
    report['compute_budget_overruns']=sum(t>=.08 for t in step_times)
    report['reference_compute_budget_observed']=bool(report['offline_candidate_passed'] and not report['compute_budget_overruns'])
    report['compute_p95_s']=float(np.percentile(step_times,95)) if step_times else None
    report['compute_p99_s']=float(np.percentile(step_times,99)) if step_times else None
    report['all_reference_steps_admitted']=report['offline_candidate_passed']
    if supervised:
        final_steps=0
        while follower.phase not in ('ready','fault'):
            follower.finish('frozen_input_end');final_steps+=1
        report['final_braking_steps']=final_steps
        report['final_reference_speed_radps']=max(abs(x) for x in follower.state.v)
        report['final_reference_acceleration_radps2']=max(abs(x) for x in follower.state.a)
    audited=sorted(held+observations,key=lambda r:r['server_mono_ns'])
    report['reference_intent_audit']=audit_trace(audited)
    if report['reference_intent_audit']['stable_height_drop_observed']:
        report['offline_candidate_passed']=False
    if reference_trace is not None:
        reference_trace.parent.mkdir(parents=True,exist_ok=True)
        reference_trace.write_text(''.join(json.dumps(r)+'\n' for r in audited),encoding='utf-8')
    if check_braking:
        brake_before=time.perf_counter();checks=[]
        for index,(tick,state) in enumerate(brake_samples):
            try:
                plan,result=plan_cartesian_braking(model,state)
                checks.append(dict(step=tick,available=True,segments=len(plan.segments),**result))
            except ValueError as error:
                if not str(error).startswith(('cartesian_brake_','cartesian_reference_')):raise
                checks.append(dict(step=tick,available=False,rejection=str(error)))
            if len(brake_samples)>100 and (index+1)%100==0:
                print(f'offline braking checks: {index+1}/{len(brake_samples)}',file=sys.stderr,flush=True)
        report['sampled_braking']=checks
        report['all_sampled_brakes_available']=bool(checks) and all(c['available'] for c in checks)
        report['braking_coverage']=dict(checked=len(checks),reference_states=report['completed_steps'],
            stride=braking_stride,all_end_states=bool(checks) and len(checks)==total_steps)
        report['brake_check_wall_s']=time.perf_counter()-brake_before
        if not report['all_sampled_brakes_available']:report['offline_candidate_passed']=False
    report['planning_wall_s']=time.perf_counter()-before
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frozen-log',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--large-pitch',action='store_true')
    p.add_argument('--replay-frozen',action='store_true')
    p.add_argument('--response-frequency',type=float,choices=(2.,4.),default=4.)
    p.add_argument('--reference-trace',type=Path)
    p.add_argument('--check-braking',action='store_true')
    p.add_argument('--demand-mode',choices=('coupled','separate'),default='coupled')
    p.add_argument('--braking-stride',type=int)
    p.add_argument('--supervised',action='store_true')
    p.add_argument('--governed-continuation',action='store_true')
    args=p.parse_args()
    if args.governed_continuation and not args.supervised:p.error('--governed-continuation requires --supervised')
    if args.braking_stride is not None and (not args.check_braking or args.braking_stride<=0):
        p.error('--braking-stride requires --check-braking and a positive integer')
    if args.replay_frozen and (args.frozen_log is None or args.large_pitch):
        p.error('--replay-frozen requires one frozen log and cannot combine --large-pitch')
    if not args.replay_frozen and (args.response_frequency!=4. or args.reference_trace is not None or args.check_braking or args.demand_mode!='coupled' or args.supervised):
        p.error('--response-frequency and --reference-trace only apply to --replay-frozen')
    report=replay(args.frozen_log,response_frequency=args.response_frequency,reference_trace=args.reference_trace,
        check_braking=args.check_braking,demand_mode=args.demand_mode,braking_stride=args.braking_stride,
        supervised=args.supervised,governed_continuation=args.governed_continuation) if args.replay_frozen else run(args.frozen_log,large_pitch=args.large_pitch)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    printable={k:v for k,v in report.items() if k!='sampled_braking'}
    if 'sampled_braking' in report:
        printable['braking_failures']=[c for c in report['sampled_braking'] if not c['available']][:5]
        printable['braking_failure_count']=sum(not c['available'] for c in report['sampled_braking'])
    print(json.dumps(printable))
    return 0 if report['offline_candidate_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
