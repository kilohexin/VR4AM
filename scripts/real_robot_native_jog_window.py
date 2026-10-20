"""Preview by default. One separately authorized fixed +Z speedl diagnostic."""
import argparse
import asyncio
from concurrent.futures import ThreadPoolExecutor
from datetime import UTC, datetime, timedelta
import hashlib
import importlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from time import perf_counter_ns

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
CONFIRM = 'I_APPROVE_ONE_SPEEDL_Z_DIAGNOSTIC_AND_ONE_ORIGINAL_STOP'
SDK_SHA = 'ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b'
_spec = importlib.util.spec_from_file_location('jog_readonly_helpers',Path(__file__).with_name('real_robot_readonly_kinematics.py'))
readonly = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(readonly)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runtime_hashes():
    paths = [*sorted((ROOT/'backend/app').rglob('*.py')),Path(__file__).resolve(),
        ROOT/'scripts/local_rpc_connections.py',ROOT/'scripts/real_robot_readonly_kinematics.py',
        ROOT/'.venv/Lib/site-packages/lebai_sdk/__init__.py',
        ROOT/'.venv/Lib/site-packages/lebai_sdk/lebai_sdk.pyd']
    return {p.relative_to(ROOT).as_posix():sha(p) for p in paths}


def existing_rpc_connections():
    return readonly.existing_rpc_connections()


def _reference(path, settings):
    from app.commissioning.native_jog_mdh import MdhTool
    from app.commissioning.native_jog_model_guard import _pose_matches
    from app.commissioning.readonly_jog_jacobian import READ_METHODS,_query_points,_finish_analysis
    from app.schemas.messages import Pose
    from scipy.spatial.transform import Rotation
    try:
        r=json.loads(path.read_bytes());a=json.loads(path.with_name('admission.json').read_bytes())
        s=a['scope'];calls=r['calls'];samples=r['fk_samples']
        if (r['runtime']!='LIVE_READONLY' or r['complete'] is not True
                or r['feedback_fk_consistent'] is not True or r['jacobian_collected'] is not True
                or r['pending_remote_result_unknown'] is not False
                or s['scope']!='one_readonly_jog_jacobian_no_robot_writes'
                or s['target_ip']!=settings.ip or s['allowed_writes']!=[]
                or s['max_fk_calls']!=13 or s['max_ik_calls']!=0
                or len(samples)!=13 or not 1 <= len(calls) <= 29
                or sum(c['method']=='connect' for c in calls)!=1
                or sum(c['method']=='kinematics_forward' for c in calls)!=13
                or any(c['method'] not in set(READ_METHODS)|{'connect'} or c['entered'] is not True
                    or c['outcome']!='returned' or c['remote_result_unknown'] is not False for c in calls)
                or any(r.get(flag) is not False for flag in ('hardware_verified','joint_bounds_verified',
                    'braking_verified','continuous_control_admitted','motion_authorized'))):
            raise ValueError
        for state in (r['initial'],r['final']):
            if state['state'] not in ('IDLE','PAUSED') or state['velocity_factor']!=100:
                raise ValueError
            if not np.isfinite(np.array([state[k] for k in ('q','qd','qdd')],dtype=float)).all():raise ValueError
            if len(state['q'])!=6 or any(state['qd']):raise ValueError
        initial=r['initial'];tool=Pose.model_validate(initial['tcp_setting'])
        final_tool=Pose.model_validate(r['final']['tcp_setting'])
        e=settings.expected_tcp
        expected_q=Rotation.from_euler('ZYX',[e.rz,e.ry,e.rx]).as_quat()
        if (not _pose_matches(tool.p,tool.q,(e.x,e.y,e.z),expected_q,1e-9,1e-6)
                or not _pose_matches(final_tool.p,final_tool.q,tool.p,tool.q,1e-9,1e-6)
                or max(abs(x-y) for x,y in zip(initial['q'],r['final']['q'],strict=True))>.001):raise ValueError
        for row,(role,q) in zip(samples,_query_points(initial,settings),strict=True):
            if row['role']!=role or row['joints']!=q:raise ValueError
            Pose.model_validate(row['pose'])
        analysis={};_finish_analysis(initial,samples,analysis)
        if analysis['controller_jacobian_rank']!=6 or not np.isfinite(np.array(analysis['controller_jacobian'])).all():raise ValueError
        return MdhTool(tool.p,tool.q)
    except Exception:
        raise ValueError('native_jog_reference_evidence_rejected') from None


def prepare(config, reference_survey, planning_limits, output, *, expires=None):
    from app.config import Settings, REAL_ROBOT_CONFIRMATION
    from app.commissioning.native_jog_packet import NativeJogLimits
    from app.commissioning.native_jog_site_scope import SiteJogScope,site_address
    config=Path(config).resolve(strict=True);reference=Path(reference_survey).resolve(strict=True)
    planning=Path(planning_limits).resolve(strict=True);output=Path(output).resolve()
    inputs=(config,reference,reference.with_name('admission.json'),planning)
    hashes={str(p):sha(p) for p in inputs}
    # Parse only, as in the existing isolated diagnostic entries. This does not
    # grant access: execute still requires its own scope, hash and deadline.
    previous=os.environ.get('VR4ARM_REAL_ROBOT_CONFIRM')
    os.environ['VR4ARM_REAL_ROBOT_CONFIRM']=REAL_ROBOT_CONFIRMATION
    try:settings=Settings.load(config)
    finally:
        if previous is None:os.environ.pop('VR4ARM_REAL_ROBOT_CONFIRM',None)
        else:os.environ['VR4ARM_REAL_ROBOT_CONFIRM']=previous
    if settings.backend!='lebai' or settings.lebai is None or settings.lebai.mode!='control':
        raise ValueError('native_jog_control_configuration_required')
    robot=settings.lebai;site_address(robot.ip)
    if robot.ip=='127.0.0.1':raise ValueError('native_jog_site_cli_requires_rfc1918')
    if output==(ROOT/'.tmp').resolve() or not output.is_relative_to((ROOT/'.tmp').resolve()):
        raise ValueError('native_jog_output_under_repo_tmp_required')
    if output.exists():raise FileExistsError('native_jog_output_consumed')
    limits=NativeJogLimits(**json.loads(planning.read_bytes()))
    tool=_reference(reference,robot)
    scope=SiteJogScope(robot,limits,hashes[str(config)],tool,lambda:None,lambda:None,lambda:None)
    if hashes!={str(p):sha(p) for p in inputs}:raise ValueError('native_jog_input_changed_during_prepare')
    return dict(config=config,reference_survey=reference,reference_admission=reference.with_name('admission.json'),
        planning_limits=planning,output=output,settings=robot,limits=limits,tool=tool,
        config_sha256=hashes[str(config)],input_hashes=hashes,runtime_hashes=runtime_hashes(),
        model=scope.model_guard().assessment,authorization_expires_utc=expires)


def preview(plan):
    from app.commissioning.native_jog_site_client import FEEDBACK_READ_POLICY
    from dataclasses import asdict
    return dict(schema=1,scope='one_native_speedl_z_and_original_stop',target_ip=plan['settings'].ip,
        output=str(plan['output']),input_hashes=plan['input_hashes'],runtime_hashes=plan['runtime_hashes'],
        authorization_expires_utc=plan['authorization_expires_utc'],max_connections=1,
        max_read_entries=256,max_total_calls=259,max_fk_calls=1,max_ik_calls=0,max_velocity_factor_calls=1,
        allowed_writes=['speedl','stop_move'],max_speedl_entries=1,max_stop_entries=1,
        velocity_mps=[0.,0.,.005],acceleration_mps2=.01,duration_s=.08,rotation=False,
        planning_limits=json.loads(json.dumps(asdict(plan['limits']))),nominal_reserve_is_not_braking_bound=True,
        model=plan['model'],reference_joint_radius_rad=.005,initial_offset_rad=.001,
        total_budget_s=20,connect_timeout_s=5,read_timeout_ms=200,fk_timeout_ms=200,
        motion_ttl_ms=100,native_speedl_wait_ms=60,stop_rpc_timeout_ms=200,
        stable_ms=300,stop_verification_ms=500,readonly_tail_s=2,cleanup_reserve_s=5,
        hardware_verified=False,joint_bounds_verified=False,braking_verified=False,
        continuous_control_admitted=False,motion_authorized=False,automatic_retry=False,
        requires_fresh_onsite_authorization=True,sdk_binary_sha256=SDK_SHA,
        snapshot_order='preflight_parallel_six_then_motion_parallel_six',
        motion_authorization_policy='feedback_at_entry_original_input_until_rpc_terminal',
        runtime_precheck_policy='single_pre_pair_scan_original_native_write_scan',
        read_scheduling_policy=FEEDBACK_READ_POLICY,
        read_authorization_policy='inputs_and_expiry_each_read_and_permission_runtime_at_group_submission_native_write_and_return',
        event_clock='time.perf_counter_ns',
        runtime_check='full_sha_after_loading_before_lease_metadata_at_native_write_then_original_freshness',
        runtime_metadata_fields=['device','inode','size','mtime_ns','ctime_ns'],
        runtime_bytes_rehashed_at_write=False,
        metadata_preserving_content_tampering_detected_at_write=False)


class Inputs:
    def __init__(self,plan,approved,expected_hash,expires,confirm):
        self.plan,self.approved,self.digest,self.expires,self.confirm=plan,approved,expected_hash,expires,confirm
        self.metadata=None;self.copies={}
        self.input_metadata=None
        self._paths=None
        self._verified=False

    def expiration(self):
        try:
            expiry=datetime.fromisoformat(self.expires.replace('Z','+00:00'));now=datetime.now(UTC)
            if (expiry.tzinfo is None or expiry.utcoffset()!=timedelta(0) or not now<expiry<=now+timedelta(minutes=10)
                    or self.expires!=self.plan['authorization_expires_utc']):raise ValueError
        except Exception:raise ValueError('native_jog_authorization_expired_or_invalid') from None

    def paths(self):
        if self._paths is not None:return self._paths
        paths=[ROOT/name for name in self.plan['runtime_hashes']]
        if not all(p.resolve(strict=True).is_relative_to(ROOT.resolve()) for p in paths):
            raise ValueError('native_jog_runtime_path_invalid')
        self._paths=tuple(paths)
        return self._paths

    def fingerprints(self):
        paths=self.paths()
        def scan(chunk):
            return [(i,(s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns))
                for i,p in chunk for s in (p.stat(),)]
        indexed=list(enumerate(paths))
        if len(paths)<16:return [row for _,row in scan(indexed)]
        # File metadata only: no SDK work or authorization callbacks on workers.
        # Join every worker before comparing or returning, including failures.
        with ThreadPoolExecutor(max_workers=4,thread_name_prefix='jog-metadata') as pool:
            rows=[row for chunk in pool.map(scan,[indexed[i::4] for i in range(4)]) for row in chunk]
        return [row for _,row in sorted(rows)]

    def input_fingerprints(self):
        return [(str(p),s.st_dev,s.st_ino,s.st_size,s.st_mtime_ns,s.st_ctime_ns)
            for p in map(Path,(*self.plan['input_hashes'],*self.copies)) for s in (p.stat(),)]

    def read(self):
        self.expiration()
        if self.confirm!=CONFIRM or self.approved is None:raise ValueError('native_jog_confirmation_required')
        raw=Path(self.approved).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=self.digest or json.loads(raw)!=preview(self.plan):
            raise ValueError('native_jog_approved_preview_changed')
        if sha(self.plan['config'])!=self.plan['config_sha256']:
            raise ValueError('native_jog_inputs_changed')
        if self.copies and sha(next(iter(self.copies)))!=self.plan['config_sha256']:
            raise ValueError('native_jog_private_copy_changed')
        if self.input_metadata is not None and self.input_metadata!=self.input_fingerprints():
            raise ValueError('native_jog_input_metadata_changed')
        self.expiration()

    def hot(self):
        self.read()
        if self.metadata is not None and self.metadata!=self.fingerprints():
            raise ValueError('native_jog_runtime_metadata_changed')
        self.expiration()

    def cold(self):
        self._verified=False
        self.read()
        before=self.fingerprints()
        if self.metadata is not None and before!=self.metadata:
            raise ValueError('native_jog_runtime_metadata_changed')
        input_before=self.input_fingerprints()
        for path,digest in (*self.plan['input_hashes'].items(),*self.copies.items()):
            if sha(path)!=digest:raise ValueError('native_jog_inputs_changed')
        if runtime_hashes()!=self.plan['runtime_hashes']:
            raise ValueError('native_jog_runtime_changed')
        if input_before!=self.input_fingerprints():raise ValueError('native_jog_inputs_changed_during_hashes')
        self.input_metadata=input_before
        self.read()
        after=self.fingerprints()
        if before!=after:raise ValueError('native_jog_runtime_changed_during_hashes')
        self.metadata=after
        self._verified=True

    def write(self):
        # The complete byte hash finishes after loading and before the FK lease.
        # Keep the original path names so replacement through a directory link
        # also changes the file identity observed at this boundary.
        if not self._verified or self.metadata is None or self.input_metadata is None:
            raise ValueError('native_jog_runtime_not_verified')
        self.hot()


def require_execution_runtime():
    from app.commissioning.qpc_event_loop import QpcEventLoop
    if Path(sys.prefix).resolve()!=(ROOT/'.venv').resolve() or not isinstance(asyncio.get_running_loop(),QpcEventLoop):
        raise ValueError('native_jog_venv_qpc_required')


def load_sdk():
    package=(ROOT/'.venv/Lib/site-packages/lebai_sdk').resolve()
    binary=package/'lebai_sdk.pyd'
    if sha(binary)!=SDK_SHA:raise ValueError('native_jog_sdk_binary_mismatch')
    readonly.verify_sdk_source()
    module=importlib.import_module('lebai_sdk')
    native=importlib.import_module('lebai_sdk.lebai_sdk')
    if (Path(module.__file__).resolve()!=package/'__init__.py'
            or Path(native.__file__).resolve()!=binary or sha(binary)!=SDK_SHA):
        raise ValueError('native_jog_loaded_sdk_identity_mismatch')
    return module


async def execute(plan, *, approved, expected_hash, expires, confirm, client_factory=None):
    from app.commissioning.native_jog_site_scope import SiteJogScope
    from app.commissioning.native_jog_site_run import run_site_window
    from app.recording.commissioning import CommissioningRecorder
    if client_factory is None:require_execution_runtime()
    inputs=Inputs(plan,approved,expected_hash,expires,confirm);inputs.cold()
    if existing_rpc_connections()!=0:raise ValueError('native_jog_existing_rpc_connections')
    output=plan['output'];output.mkdir(parents=True,exist_ok=False)
    (output/'admission.json').write_text(json.dumps(dict(scope=preview(plan),approved_preview_sha256=expected_hash,
        started_utc=datetime.now(UTC).isoformat()),indent=2),encoding='utf-8')
    for i,(path,digest) in enumerate(plan['input_hashes'].items()):
        copied=output/('input-'+str(i)+Path(path).suffix);copied.write_bytes(Path(path).read_bytes())
        inputs.copies[str(copied)]=digest
    inputs.input_metadata=None
    report=dict(runtime='SIMULATION' if client_factory is not None else 'LIVE_SINGLE_JOG_DIAGNOSTIC')
    connector=dict(factory_entries=0,native_sdk_init_entered=False,native_sdk_connect_entered=False,
        remote_result_unknown=False,outcome='not_started',event_clock='time.perf_counter_ns')
    recorder=CommissioningRecorder(output/'sessions',metadata=dict(workflow='single_native_jog',**report))
    async def factory(ip):
        inputs.cold()
        if connector['factory_entries'] or ip!=plan['settings'].ip or existing_rpc_connections()!=0:
            raise ValueError('native_jog_connection_not_admitted')
        connector.update(factory_entries=1,started_ns=perf_counter_ns(),outcome='entered')
        try:
            if client_factory is not None:
                value=await client_factory(ip)
            else:
                module=load_sdk();inputs.cold()
                connector['native_sdk_init_entered']=True;module.init();inputs.hot()
                connector['native_sdk_connect_entered']=True
                value=await module.connect(ip,False)
            inputs.hot();connector.update(outcome='returned',completed_ns=perf_counter_ns())
            return value
        except BaseException:
            connector.update(outcome='error',remote_result_unknown=connector['native_sdk_connect_entered'])
            raise
    try:
        scope=SiteJogScope(plan['settings'],plan['limits'],plan['config_sha256'],plan['tool'],
            inputs.cold,inputs.hot,inputs.write,read_check=inputs.read)
        await run_site_window(scope,recorder,factory,clock=perf_counter_ns,report=report)
    except BaseException as error:
        report.update(complete=False,error_type=type(error).__name__,primary_rejection='native_jog_entry_failed')
        if isinstance(error,(KeyboardInterrupt,SystemExit,asyncio.CancelledError)):raise
    finally:
        if connector['outcome']=='entered':connector['remote_result_unknown']=True
        report['connection_diagnostics']=dict(connector)
        (output/'result.json').write_text(json.dumps(report,allow_nan=False,indent=2)+'\n',encoding='utf-8')
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','reference-survey','planning-limits','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--execute-once',action='store_true');parser.add_argument('--approved-preview',type=Path)
    parser.add_argument('--approved-preview-sha256');parser.add_argument('--authorization-expires-utc');parser.add_argument('--confirm')
    a=parser.parse_args(argv)
    try:
        plan=prepare(a.config,a.reference_survey,a.planning_limits,a.output,expires=a.authorization_expires_utc)
        if not a.execute_once:print(json.dumps(preview(plan),allow_nan=False,indent=2));return 0
        from app.commissioning.qpc_event_loop import new_qpc_loop
        with asyncio.Runner(loop_factory=new_qpc_loop) as runner:
            r=runner.run(execute(plan,approved=a.approved_preview,expected_hash=a.approved_preview_sha256,
                expires=a.authorization_expires_utc,confirm=a.confirm))
        print(json.dumps(dict(output=str(plan['output']),complete=r.get('complete',False),
            stop_confirmed=r.get('stop_confirmed',False),hardware_verified=False)))
        return 0 if r.get('complete') else 2
    except (ValueError,OSError):parser.error('native_jog_input_or_admission_rejected')


if __name__=='__main__':raise SystemExit(main())
