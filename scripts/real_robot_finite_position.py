"""Preview by default: one +Z finite movej, natural arrival, original fault stop."""
import argparse
import asyncio
from dataclasses import asdict
from datetime import UTC,datetime,timedelta
import hashlib
import json
import os
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts import real_robot_native_jog_window as identity
from scripts.real_robot_ready_return import close_loop_bounded
from app.commissioning.finite_position_site import MdhPositionModel,run_single_position,MAX_READS
from app.control.finite_position_types import PLAN_NS

CONFIRM='I_APPROVE_ONE_FINITE_POSITION_Z_NATURAL_ARRIVAL_AND_RESERVED_FAULT_STOP'
sha=identity.sha
load_sdk=identity.load_sdk
existing_rpc_connections=identity.existing_rpc_connections


def runtime_hashes():
    values=identity.runtime_hashes()
    for name in ('real_robot_finite_position.py','real_robot_ready_return.py'):
        p=Path(__file__).with_name(name).resolve();values[p.relative_to(ROOT).as_posix()]=sha(p)
    return values


def prepare(config,reference_survey,ready_pose,output,*,expires=None):
    from app.config import Settings,REAL_ROBOT_CONFIRMATION
    from app.robots.lebai_codec import pose_from_lebai
    from app.commissioning.native_jog_site_scope import site_address
    config,reference_survey,ready_pose=(Path(p).resolve(strict=True) for p in (config,reference_survey,ready_pose))
    admission=reference_survey.with_name('admission.json').resolve(strict=True)
    output=Path(output).resolve()
    if output==(ROOT/'.tmp').resolve() or not output.is_relative_to((ROOT/'.tmp').resolve()):
        raise ValueError('position_site_output_under_tmp_required')
    if output.exists():raise FileExistsError('position_site_output_consumed')
    hashes={str(p):sha(p) for p in (config,reference_survey,admission,ready_pose)}
    previous=os.environ.get('VR4ARM_REAL_ROBOT_CONFIRM')
    os.environ['VR4ARM_REAL_ROBOT_CONFIRM']=REAL_ROBOT_CONFIRMATION
    try:settings=Settings.load(config)
    finally:
        if previous is None:os.environ.pop('VR4ARM_REAL_ROBOT_CONFIRM',None)
        else:os.environ['VR4ARM_REAL_ROBOT_CONFIRM']=previous
    s=settings.lebai
    if (settings.backend!='lebai' or s is None or s.mode!='control'
            or not 0<s.control.max_joint_speed_radps<=.05
            or not 0<s.control.max_joint_acceleration_radps2<=.2):
        raise ValueError('position_site_control_profile_required')
    site_address(s.ip)
    if s.ip=='127.0.0.1':raise ValueError('position_site_private_target_required')
    tool=identity._reference(reference_survey,s);model=MdhPositionModel(tool)
    selected=json.loads(ready_pose.read_bytes());reference=json.loads(reference_survey.read_bytes())
    try:
        if (selected['selection']!='operator_selected_current_working_ready_pose' or selected['raw_state']!='IDLE'
                or selected['source']['target_ip']!=s.ip or tuple(selected['q_rad'])!=s.teleop_ready_q
                or selected['tool_tcp_setting']!=asdict(s.expected_tcp)
                or max(abs(a-b) for a,b in zip(reference['initial']['q'],s.teleop_ready_q,strict=True))>.001):
            raise ValueError
        pose=pose_from_lebai(selected['actual_tcp_pose'])
        if any(lo<p-.01-1e-9 or hi>p+.01+1e-9 for lo,p,hi in zip(s.startup_tcp_min_m,pose.p,s.startup_tcp_max_m,strict=True)):
            raise ValueError
        if any(not lo<=p<=hi for lo,p,hi in zip(s.startup_tcp_min_m,pose.p,s.startup_tcp_max_m,strict=True)):
            raise ValueError
    except (ValueError,KeyError,TypeError,AttributeError):
        raise ValueError('position_site_saved_ready_or_workspace_rejected') from None
    if any(sha(p)!=h for p,h in hashes.items()):raise ValueError('position_site_inputs_changed_during_prepare')
    return dict(config=config,reference_survey=reference_survey,reference_admission=admission,ready_pose=ready_pose,
        output=output,settings=s,model=model,input_hashes=hashes,runtime_hashes=runtime_hashes(),
        authorization_expires_utc=expires)


def preview(p):
    return dict(schema=1,scope='one_finite_position_z_natural_arrival',target_ip=p['settings'].ip,
        output=str(p['output']),input_hashes=p['input_hashes'],runtime_hashes=p['runtime_hashes'],
        settings_binding=json.loads(json.dumps(asdict(p['settings']))),model_sha256=p['model'].model_sha256,
        authorization_expires_utc=p['authorization_expires_utc'],distance_m=.0005,direction=[0.,0.,1.],
        max_connections=1,max_movej_entries=1,max_stop_entries=1,max_ik_calls=1,max_fk_calls=5,max_read_entries=MAX_READS,
        allowed_writes=['movej','stop_move'],feedback_ttl_ms=100,input_ttl_ms=None,movej_rpc_timeout_ms=200,
        input_policy='one_fixed_authorized_target_until_scope_deadline',planning_timeout_ms=PLAN_NS/1e6,
        snapshot_read_policy='owned_six_fields_parallel',readonly_authorization_policy='full_at_group_boundaries_binding_and_cancellation_per_read',
        write_check_policy='one_full_native_entry_check_followed_by_fresh_feedback_check',
        entry_policy='fresh_identical_start_after_planning',held_input_ttl_ms=100,
        completion_timeout_s=2,stable_samples=3,stable_span_ms=100,readonly_tail_s=2,total_budget_s=20,
        normal_budget_s=15,cleanup_reserve_s=5,stop_rpc_timeout_ms=200,stop_stable_ms=300,stop_verification_ms=500,
        native_time_argument=0,native_blend_radius=0,natural_arrival_does_not_send_stop=True,
        endpoint_error_fraction=.2,min_progress_fraction=.7,max_cross_fraction=.2,
        ready_tolerance_rad=.001,max_joint_speed_radps=p['settings'].control.max_joint_speed_radps,
        max_joint_acceleration_radps2=p['settings'].control.max_joint_acceleration_radps2,
        motion_authorized=False,hardware_verified=False,factory_calibrated=False,motion_time_stop_verified=False,
        continuous_control_admitted=False,automatic_retry=False,automatic_home=False,automatic_resume=False,
        sdk_binary_sha256=identity.SDK_SHA,runtime_policy='full_sha_before_fresh_reads_metadata_at_write',
        metadata_preserving_tampering_detected_at_write=False,requires_fresh_onsite_authorization=True)


def metadata(paths):
    return [(str(p),s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns) for p in map(Path,paths) for s in (p.stat(),)]


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
            raise ValueError('position_site_scope_expired_or_changed') from None

    def cold(self):
        self.read();paths=tuple(ROOT/x for x in self.p['runtime_hashes'])
        if not all(p.resolve(strict=True).is_relative_to(ROOT.resolve()) for p in paths):
            raise ValueError('position_site_runtime_path_invalid')
        before=metadata(paths);inputs_before=metadata((*self.p['input_hashes'],*self.copies))
        if self.runtime_metadata is not None and self.runtime_metadata!=before:raise ValueError('position_site_runtime_changed')
        if runtime_hashes()!=self.p['runtime_hashes']:raise ValueError('position_site_runtime_changed')
        if any(sha(p)!=h for p,h in (*self.p['input_hashes'].items(),*self.copies.items())):raise ValueError('position_site_input_changed')
        if before!=metadata(paths) or inputs_before!=metadata((*self.p['input_hashes'],*self.copies)):
            raise ValueError('position_site_files_changed_during_hashes')
        self.runtime_metadata=before;self.input_metadata=inputs_before;self.read()

    def write(self):
        self.read()
        if self.runtime_metadata is None or metadata(ROOT/x for x in self.p['runtime_hashes'])!=self.runtime_metadata:
            raise ValueError('position_site_runtime_identity_changed')
        self.read()


async def execute(p,*,approved,expected_hash,expires,confirm,client_factory=None,clock=time.perf_counter_ns,sleep=asyncio.sleep):
    from app.recording.commissioning import CommissioningRecorder
    if client_factory is None:
        identity.require_execution_runtime()
        if clock is not time.perf_counter_ns or sleep is not asyncio.sleep:raise ValueError('position_site_native_clock_override')
    inputs=Inputs(p,approved,expected_hash,expires,confirm);inputs.cold()
    if existing_rpc_connections()!=0:raise ValueError('position_site_existing_rpc')
    output=p['output'];output.mkdir(parents=True,exist_ok=False)
    (output/'admission.json').write_text(json.dumps(dict(scope=preview(p),approved_preview_sha256=expected_hash),indent=2),encoding='utf-8')
    for i,(name,h) in enumerate(p['input_hashes'].items()):
        copied=output/('input-'+str(i)+Path(name).suffix);copied.write_bytes(Path(name).read_bytes());inputs.copies[str(copied)]=h
    inputs.input_metadata=None
    connector=dict(factory_entries=0,native_sdk_init_entered=False,native_sdk_connect_entered=False,remote_result_unknown=False,outcome='not_started')
    recorder=CommissioningRecorder(output/'sessions',metadata=dict(workflow='single_finite_position',runtime='SIMULATION' if client_factory else 'LIVE_SINGLE_FINITE_POSITION'))
    async def factory(ip):
        inputs.cold()
        if connector['factory_entries'] or ip!=p['settings'].ip or existing_rpc_connections()!=0:raise ValueError('position_site_connection_not_admitted')
        connector.update(factory_entries=1,outcome='entered')
        try:
            if client_factory is not None:value=await client_factory(ip)
            else:
                module=load_sdk();inputs.cold();connector['native_sdk_init_entered']=True;module.init();inputs.read()
                connector['native_sdk_connect_entered']=True;value=await module.connect(ip,False)
            inputs.cold();connector['outcome']='returned';return value
        except BaseException:
            connector.update(outcome='error',remote_result_unknown=connector['native_sdk_connect_entered']);raise
    report=dict(complete=False)
    try:
        report=await run_single_position(p['settings'],recorder,factory,inputs.read,config_sha256=p['input_hashes'][str(p['config'])],
            model=p['model'],read_authorize=inputs.read,write_authorize=inputs.write,clock=clock,sleep=sleep)
        return report
    finally:
        if connector['outcome']=='entered':connector['remote_result_unknown']=True
        if connector['remote_result_unknown']:report['complete']=False
        report.update(runtime='SIMULATION' if client_factory else 'LIVE_SINGLE_FINITE_POSITION',connection_diagnostics=connector)
        (output/'result.json').write_text(json.dumps(report,indent=2,allow_nan=False),encoding='utf-8')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('config','reference-survey','ready-pose','output'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--execute-once',action='store_true');parser.add_argument('--approved-preview',type=Path)
    parser.add_argument('--approved-preview-sha256');parser.add_argument('--authorization-expires-utc');parser.add_argument('--confirm')
    a=parser.parse_args(argv)
    try:
        p=prepare(a.config,a.reference_survey,a.ready_pose,a.output,expires=a.authorization_expires_utc)
        if not a.execute_once:print(json.dumps(preview(p),indent=2,allow_nan=False));return 0
        from app.commissioning.qpc_event_loop import new_qpc_loop
        loop=new_qpc_loop();interrupted=False
        task=loop.create_task(execute(p,approved=a.approved_preview,expected_hash=a.approved_preview_sha256,
            expires=a.authorization_expires_utc,confirm=a.confirm))
        try:r=loop.run_until_complete(task)
        except KeyboardInterrupt:
            interrupted=True;task.cancel()
            try:loop.run_until_complete(asyncio.wait({task},timeout=5.))
            except KeyboardInterrupt:pass
            if task.done() and not task.cancelled():task.exception()
            path=p['output']/'result.json'
            r=json.loads(path.read_bytes()) if path.exists() else dict(complete=False,motion_result_unverified=True,native_connection_release_verified=False)
            r.update(complete=False,interrupted=True)
        finally:pending=close_loop_bounded(loop)
        r['pending_tasks_at_process_exit']=pending
        if pending:r['complete']=False
        p['output'].mkdir(parents=True,exist_ok=True)
        (p['output']/'result.json').write_text(json.dumps(r,indent=2,allow_nan=False),encoding='utf-8')
        print(json.dumps({k:r.get(k) for k in ('complete','movej_entries','stop_entries','latched_fault')}))
        return 130 if interrupted else 0 if r['complete'] else 2
    except (OSError,ValueError):parser.error('position_site_input_or_scope_rejected')


if __name__=='__main__':raise SystemExit(main())
