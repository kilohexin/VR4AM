"""Default preview; independently approved one prepare movej, never a Home."""
import argparse
import asyncio
from collections import Counter
from dataclasses import asdict
from datetime import UTC,datetime,timedelta
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts import real_robot_native_jog_window as identity
from app.commissioning.ready_return import OPTIONS,MAX_READS,READY_TOLERANCE_RAD,_q,run_ready_return

CONFIRM='I_APPROVE_ONE_EXISTING_PREPARE_TO_SAVED_READY_NO_HOME_NO_RETRY'
sha=identity.sha
load_sdk=identity.load_sdk
existing_rpc_connections=identity.existing_rpc_connections


def runtime_hashes():
    paths=identity.runtime_hashes()
    for path in (Path(__file__).resolve(),Path(__file__).with_name('ready_then_position_gate.py').resolve()):
        paths[path.relative_to(ROOT).as_posix()]=sha(path)
    return paths


def readonly_reference(settings,path,hashes):
    """Accept a sealed completed observation as a start, never as stop proof.

    The live runner still rereads and matches this start before its sole movej.
    No state or fault in a preceding session is changed by parsing this file.
    """
    from app.schemas.messages import Pose
    from app.robots.lebai_codec import pose_from_lebai
    from app.commissioning.readonly_kinematics import _angle
    admission=path.parent/'admission.json';approved=path.parent.parent/'preview.json'
    for item in (admission,approved):hashes[str(item)]=sha(item)
    value=json.loads(path.read_bytes());seal=json.loads(admission.read_bytes())
    scope=json.loads(approved.read_bytes())
    if (seal['approved_preview_sha256']!=hashes[str(approved)] or seal['scope']!=scope
            or scope['scope']!='one_readonly_fk_ik_survey_no_robot_writes'
            or scope['mode']!='readonly' or scope['target_ip']!=settings.ip
            or Path(scope['output']).resolve()!=path.parent
            or scope['allowed_writes']!=[] or scope['max_connections']!=1
            or scope['max_total_calls_including_connect']!=24
            or scope['max_ik_calls']!=1 or scope['max_fk_calls']!=4
            or value['runtime']!='LIVE_READONLY' or value['complete'] is not True
            or value['kinematics_consistent'] is not True
            or value['motion_authorized'] is not False
            or value['hardware_stop_verified'] is not False
            or value['pending_remote_result_unknown'] is not False):raise ValueError
    rows=value['calls'];counts=Counter(x['method'] for x in rows)
    motion_queries=counts.pop('get_motion_state',0)
    if (counts!=Counter(connect=1,is_connected=2,get_robot_state=4,get_estop_reason=2,
            get_running_motion=2,get_kin_data=2,get_tcp=2,kinematics_inverse=1,kinematics_forward=4)
            or not 0<=motion_queries<=2 or len(rows)>24
            or any(x['entered'] is not True or x['outcome']!='returned'
                or x['remote_result_unknown'] is not False for x in rows)):raise ValueError
    raw=lambda name:[x['raw'] for x in rows if x['method']==name]
    # Validate required mathematical payloads; they are not used as motion or
    # stop evidence by this return entry (whose live preflight is independent).
    for payload in raw('kinematics_forward'):pose_from_lebai(payload)
    _q(raw('kinematics_inverse')[0])
    if (any(x is not True for x in raw('is_connected'))
            or any(not ((type(x) is int and x==5) or (type(x) is str and x=='IDLE'))
                for x in raw('get_robot_state'))
            or any(not ((type(x) is int and x==0) or (type(x) is str and x.lower() in ('none','0')))
                for x in raw('get_estop_reason'))
            or any(type(x) is not int or x<0 for x in raw('get_running_motion'))
            or sum(x>0 for x in raw('get_running_motion'))!=motion_queries
            or any(x!='FINISHED' for x in raw('get_motion_state'))):raise ValueError
    initial,final=value['initial'],value['final'];start=_q(final['q'])
    expected=pose_from_lebai(asdict(settings.expected_tcp))
    for observation,kin,tool in zip((initial,final),raw('get_kin_data'),raw('get_tcp'),strict=True):
        observed_pose=Pose.model_validate(observation['tcp'])
        raw_pose=pose_from_lebai(kin['actual_tcp_pose'])
        if math.dist(observed_pose.p,raw_pose.p)>1e-9 or _angle(observed_pose.q,raw_pose.q)>1e-6:
            raise ValueError
        if (observation['state']!='IDLE' or _q(observation['q'])!=start
                or _q(kin['actual_joint_pose'])!=start
                or max(abs(a-b) for a,b in zip(_q(kin['target_joint_pose']),start,strict=True))>.001
                or any(_q(observation[key])!=(0.,)*6 for key in ('qd','qdd'))
                or any(_q(kin[key])!=(0.,)*6 for key in ('actual_joint_speed','actual_joint_acc',
                    'target_joint_speed','target_joint_acc'))):raise ValueError
        for setting in (Pose.model_validate(observation['tcp_setting']),pose_from_lebai(tool)):
            if math.dist(setting.p,expected.p)>1e-9 or _angle(setting.q,expected.q)>1e-6:raise ValueError
    a,b=(Pose.model_validate(x['tcp']) for x in (initial,final))
    if math.dist(a.p,b.p)>1e-5 or _angle(a.q,b.q)>.001:raise ValueError
    return start,b


def prepare(config,start_session,ready_pose,output,*,expires=None):
    from app.config import Settings,REAL_ROBOT_CONFIRMATION
    from app.robots.base import BackendCommandError
    from app.schemas.messages import Pose
    config,start_session,ready_pose=(Path(x).resolve(strict=True) for x in (config,start_session,ready_pose))
    output=Path(output).resolve()
    if output==(ROOT/'.tmp').resolve() or not output.is_relative_to((ROOT/'.tmp').resolve()):
        raise ValueError('ready_return_output_under_tmp_required')
    if output.exists():raise FileExistsError('ready_return_output_consumed')
    hashes={str(p):sha(p) for p in (config,start_session,ready_pose)}
    previous=os.environ.get('VR4ARM_REAL_ROBOT_CONFIRM')
    os.environ['VR4ARM_REAL_ROBOT_CONFIRM']=REAL_ROBOT_CONFIRMATION
    try:s=Settings.load(config)
    finally:
        if previous is None:os.environ.pop('VR4ARM_REAL_ROBOT_CONFIRM',None)
        else:os.environ['VR4ARM_REAL_ROBOT_CONFIRM']=previous
    s=s.lebai
    if (s is None or s.mode!='control' or s.control.max_joint_speed_radps!=.05
            or s.control.max_joint_acceleration_radps2!=.2):
        raise ValueError('ready_return_fixed_control_profile_required')
    from app.commissioning.native_jog_site_scope import site_address
    site_address(s.ip)
    if s.ip=='127.0.0.1':raise ValueError('ready_return_site_target_required')
    try:
        selected=json.loads(ready_pose.read_bytes())
        if (selected['selection']!='operator_selected_current_working_ready_pose'
                or selected['raw_state']!='IDLE' or selected['source']['target_ip']!=s.ip
                or _q(selected['q_rad'])!=s.teleop_ready_q
                or selected['tool_tcp_setting']!=asdict(s.expected_tcp)):
            raise ValueError
        reference_kind='successful_static_cleanup'
        if start_session.suffix.lower()=='.json':
            reference_kind='completed_readonly_survey'
            start,pose=readonly_reference(s,start_session,hashes)
        else:
            rows=[json.loads(line) for line in start_session.read_bytes().splitlines()]
            summaries=[x for x in rows if x.get('kind')=='paused_stop_cleanup_result']
            if (len(summaries)!=1 or summaries[0].get('runtime')!='LIVE_STATIC_DIAGNOSTIC'
                    or summaries[0].get('complete') is not True or summaries[0].get('sdk_stop_entries')!=1
                    or summaries[0].get('read_remote_result_unknown') is not False
                    or summaries[0].get('latched_fault') is not None):raise ValueError
            samples=[x for x in rows if x.get('kind')=='stop_observation_sample']
            if not samples or [x['sequence'] for x in samples]!=list(range(1,len(samples)+1)):
                raise ValueError
            last=samples[-1]['state'];start=_q(last['actual_q'])
            if (last['raw_robot_state']!='IDLE' or last['motion_state']!='FINISHED'
                    or _q(last['target_q'])!=start
                    or any(_q(last[k])!=(0.,)*6 for k in ('actual_qd','actual_qdd','target_qd'))):
                raise ValueError
            pose=Pose.model_validate(last['actual_tcp'])
        if any(not lo<=x<=hi for lo,x,hi in zip(s.startup_tcp_min_m,pose.p,s.startup_tcp_max_m,strict=True)):
            raise ValueError
    except (OSError,ValueError,KeyError,TypeError,AttributeError,BackendCommandError):
        raise ValueError('ready_return_reference_rejected') from None
    if any(sha(p)!=h for p,h in hashes.items()):raise ValueError('ready_return_inputs_changed_during_prepare')
    return dict(config=config,start_session=start_session,ready_pose=ready_pose,output=output,
        settings=s,start_q=start,reference_kind=reference_kind,input_hashes=hashes,runtime_hashes=runtime_hashes(),
        authorization_expires_utc=expires)


def preview(p):
    s=p['settings']
    return dict(schema=1,scope='one_existing_prepare_to_saved_ready',target_ip=s.ip,output=str(p['output']),
        input_hashes=p['input_hashes'],runtime_hashes=p['runtime_hashes'],settings_binding=json.loads(json.dumps(asdict(s))),
        start_q_rad=list(p['start_q']),target_q_rad=list(s.teleop_ready_q),
        joint_change_rad=[b-a for a,b in zip(p['start_q'],s.teleop_ready_q,strict=True)],
        authorization_expires_utc=p['authorization_expires_utc'],max_connections=1,
        allowed_writes=['movej','stop_move'],max_movej_entries=1,max_stop_entries=1,
        max_read_entries=MAX_READS,max_fk_calls=0,max_ik_calls=0,options=asdict(OPTIONS),
        acceleration_radps2=.2,blend_radius=0.,movej_rpc_timeout_ms=200,
        start_tolerance_rad=READY_TOLERANCE_RAD,feedback_ttl_ms=100,total_budget_s=90,cleanup_reserve_s=5,
        stop_rpc_timeout_ms=200,stop_stable_ms=300,stop_verification_ms=500,readonly_tail_s=2,
        natural_arrival_does_not_send_stop=True,automatic_retry=False,automatic_home=False,
        automatic_resume=False,automatic_next_motion=False,motion_authorized=False,
        hardware_verified=False,motion_time_stop_verified=False,physical_speed_verified=False,
        requires_fresh_onsite_authorization=True,speed_audit_required=True,
        sdk_binary_sha256=identity.SDK_SHA,runtime_bytes_rehashed_at_write=False,
        metadata_preserving_tampering_detected_at_write=False,
        runtime_policy='full_sha_after_load_before_fresh_reads_metadata_at_write',
        reference_kind=p['reference_kind'],prior_stop_uncertainty_cleared=False,
        evidence_policy='saved_operator_target_and_bound_start_reference_live_rechecked',
        snapshot_read_policy='owned_six_fields_parallel_except_stop')


def metadata(paths):
    return [(str(p),s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
        for p in map(Path,paths) for s in (p.stat(),)]


class Inputs:
    def __init__(self,p,approved,digest,expires,confirm):
        self.p,self.approved,self.digest,self.expires,self.confirm=p,approved,digest,expires,confirm
        self.runtime_metadata=None;self.input_metadata=None;self.copies={}

    def read(self):
        try:
            expiry=datetime.fromisoformat(self.expires.replace('Z','+00:00'));now=datetime.now(UTC)
            if (expiry.utcoffset()!=timedelta(0) or not now<expiry<=now+timedelta(minutes=10)
                    or self.expires!=self.p['authorization_expires_utc'] or self.confirm!=CONFIRM):raise ValueError
            raw=Path(self.approved).read_bytes()
            if hashlib.sha256(raw).hexdigest()!=self.digest or json.loads(raw)!=preview(self.p):raise ValueError
            if sha(self.p['config'])!=self.p['input_hashes'][str(self.p['config'])]:raise ValueError
            paths=(*self.p['input_hashes'],*self.copies)
            if self.input_metadata is not None and metadata(paths)!=self.input_metadata:raise ValueError
            if datetime.now(UTC)>=expiry:raise ValueError
        except (OSError,ValueError,TypeError,AttributeError):
            raise ValueError('ready_return_scope_expired_or_changed') from None

    def cold(self):
        self.read();paths=tuple(ROOT/x for x in self.p['runtime_hashes'])
        if not all(p.resolve(strict=True).is_relative_to(ROOT.resolve()) for p in paths):
            raise ValueError('ready_return_runtime_path_invalid')
        before=metadata(paths);inputs_before=metadata((*self.p['input_hashes'],*self.copies))
        if self.runtime_metadata is not None and self.runtime_metadata!=before:
            raise ValueError('ready_return_runtime_changed')
        if runtime_hashes()!=self.p['runtime_hashes']:raise ValueError('ready_return_runtime_changed')
        if any(sha(p)!=h for p,h in (*self.p['input_hashes'].items(),*self.copies.items())):
            raise ValueError('ready_return_input_changed')
        if before!=metadata(paths) or inputs_before!=metadata((*self.p['input_hashes'],*self.copies)):
            raise ValueError('ready_return_files_changed_during_hashes')
        self.runtime_metadata=before;self.input_metadata=inputs_before;self.read()

    def write(self):
        self.read()
        if (self.runtime_metadata is None or metadata(ROOT/x for x in self.p['runtime_hashes'])!=self.runtime_metadata):
            raise ValueError('ready_return_runtime_identity_changed')
        self.read()


async def execute(p,*,approved,expected_hash,expires,confirm,client_factory=None,
                  clock=time.perf_counter_ns,sleep=asyncio.sleep):
    from app.recording.commissioning import CommissioningRecorder
    if client_factory is None:
        identity.require_execution_runtime()
        if clock is not time.perf_counter_ns or sleep is not asyncio.sleep:
            raise ValueError('ready_return_native_clock_override_forbidden')
    inputs=Inputs(p,approved,expected_hash,expires,confirm);inputs.cold()
    if existing_rpc_connections()!=0:raise ValueError('ready_return_existing_rpc')
    output=p['output'];output.mkdir(parents=True,exist_ok=False)
    (output/'admission.json').write_text(json.dumps(dict(scope=preview(p),approved_preview_sha256=expected_hash),indent=2),encoding='utf-8')
    for i,(name,h) in enumerate(p['input_hashes'].items()):
        copied=output/('input-'+str(i)+Path(name).suffix);copied.write_bytes(Path(name).read_bytes())
        inputs.copies[str(copied)]=h
    inputs.input_metadata=None
    report=dict(runtime='SIMULATION' if client_factory is not None else 'LIVE_SINGLE_READY_RETURN')
    connector=dict(factory_entries=0,native_sdk_init_entered=False,native_sdk_connect_entered=False,
        remote_result_unknown=False,outcome='not_started')
    recorder=CommissioningRecorder(output/'sessions',metadata=dict(workflow='single_ready_return',**report))
    async def factory(ip):
        inputs.cold()
        if connector['factory_entries'] or ip!=p['settings'].ip or existing_rpc_connections()!=0:
            raise ValueError('ready_return_connection_not_admitted')
        connector.update(factory_entries=1,outcome='entered')
        try:
            if client_factory is not None:value=await client_factory(ip)
            else:
                module=load_sdk();inputs.cold();connector['native_sdk_init_entered']=True;module.init();inputs.read()
                connector['native_sdk_connect_entered']=True;value=await module.connect(ip,False)
            inputs.cold();connector['outcome']='returned';return value
        except BaseException:
            connector.update(outcome='error',remote_result_unknown=connector['native_sdk_connect_entered']);raise
    try:
        read_check=inputs.read
        await run_ready_return(p['settings'],p['start_q'],recorder,factory,read_check,
            read_authorize=read_check,write_authorize=inputs.write,clock=clock,sleep=sleep,report=report)
    finally:
        if connector['outcome']=='entered':connector['remote_result_unknown']=True
        report['connection_diagnostics']=dict(connector)
        (output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')
    return report


def close_loop_bounded(loop):
    """Cancel once; never wait indefinitely for a remote call's acknowledgement."""
    tasks={t for t in asyncio.all_tasks(loop) if not t.done()}
    for task in tasks:
        if not task.cancelling():task.cancel()
    if tasks:loop.run_until_complete(asyncio.wait(tasks,timeout=.05))
    pending=sum(not t.done() for t in tasks)
    loop.close()
    return pending


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('config','start-session','ready-pose','output'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--execute-once',action='store_true');parser.add_argument('--approved-preview',type=Path)
    parser.add_argument('--approved-preview-sha256');parser.add_argument('--authorization-expires-utc');parser.add_argument('--confirm')
    a=parser.parse_args(argv)
    try:
        p=prepare(a.config,a.start_session,a.ready_pose,a.output,expires=a.authorization_expires_utc)
        if not a.execute_once:print(json.dumps(preview(p),indent=2,allow_nan=False));return 0
        from app.commissioning.qpc_event_loop import new_qpc_loop
        loop=new_qpc_loop();interrupted=False
        task=loop.create_task(execute(p,approved=a.approved_preview,expected_hash=a.approved_preview_sha256,
            expires=a.authorization_expires_utc,confirm=a.confirm))
        try:
            r=loop.run_until_complete(task)
        except KeyboardInterrupt:
            interrupted=True;task.cancel()
            # Allow the root's original reserved stopping transaction to finish.
            # This waits for the root, never for an abandoned native child.
            try:loop.run_until_complete(asyncio.wait({task},timeout=5.))
            except KeyboardInterrupt:pass
            if task.done() and not task.cancelled():task.exception()
            path=p['output']/'result.json'
            if path.exists():r=json.loads(path.read_bytes())
            else:
                p['output'].mkdir(parents=True,exist_ok=True)
                r=dict(complete=False,error='ready_return_interrupted_without_terminal_result',
                    motion_result_unverified=True,native_connection_release_verified=False)
            r.update(complete=False,interrupted=True)
        finally:pending=close_loop_bounded(loop)
        r['pending_tasks_at_process_exit']=pending
        if pending:r['complete']=False
        (p['output']/'result.json').write_text(json.dumps(r,indent=2,allow_nan=False),encoding='utf-8')
        print(json.dumps({k:r.get(k) for k in ('complete','movej_entries','stop_entries','latched_fault')}))
        return 130 if interrupted else 0 if r['complete'] else 2
    except (OSError,ValueError):parser.error('ready_return_input_or_scope_rejected')


if __name__=='__main__':raise SystemExit(main())
