"""Preview by default. Explicit, bounded real webpage; never run on import."""
import argparse
import asyncio
from collections import Counter
from dataclasses import asdict
from datetime import UTC,datetime,timedelta
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
COMMON_CONFIG=ROOT/'config/default.yaml'
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'backend'))
from scripts import real_robot_finite_position as single

CONFIRM='I_APPROVE_ONE_BOUNDED_WEB_FINITE_POSITION_AND_PROGRESSIVE_GRIPPER_SESSION'
DURATION_NS=300_000_000_000
CLEANUP_NS=5_000_000_000
MOVE_LIMIT=21  # one preparation and at most twenty finite translation steps
CLAW_LIMIT=100
STOP_LIMIT=4
READ_LIMIT=100000


class WebWindow:
    """One connection; fixed session budget, no reset by browser reconnect."""
    READS=frozenset({'is_connected','get_robot_state','get_estop_reason','get_kin_data',
        'get_tcp','get_claw','get_running_motion','get_motion_state','get_velocity_factor',
        'kinematics_inverse','kinematics_forward'})

    def __init__(self,identity_check,*,clock=time.perf_counter_ns):
        self.identity_check=identity_check;self.clock=clock;self.started=clock()
        self.entries=Counter();self.native_entries=Counter();self.write_failed=False

    def check(self):
        if self.write_failed:raise ValueError('position_web_write_unknown')
        if not 0<=self.clock()-self.started<DURATION_NS:raise ValueError('position_web_scope_expired')
        self.identity_check()
        if not 0<=self.clock()-self.started<DURATION_NS:raise ValueError('position_web_scope_expired')

    def entry(self,name):
        if name in self.READS:
            if (not 0<=self.clock()-self.started<DURATION_NS+CLEANUP_NS
                    or self.entries['reads']>=READ_LIMIT):raise ValueError('position_web_read_budget')
            self.entries['reads']+=1
        elif name=='stop_move':
            # A late/unknown stop remains failed. This budget never retries it.
            if self.entries[name]>=STOP_LIMIT:raise ValueError('position_web_stop_budget')
        elif name in ('connect','movej','set_claw'):
            self.check()
            cap={'connect':1,'movej':MOVE_LIMIT,'set_claw':CLAW_LIMIT}[name]
            if self.entries[name]>=cap:raise ValueError('position_web_action_budget')
        else:raise ValueError('position_web_method_disabled')
        self.entries[name]+=1


class WindowClient:
    """Narrow native surface; no discover/start/reset/Home/resume/PVAT."""
    def __init__(self,native,window):self.native=native;self.window=window

    async def _position_guarded_movej(self,args,check):
        self.window.entry('movej')
        check()  # after identity/budget work, immediately before native SDK
        self.window.native_entries['movej']+=1
        try:return await self.native.movej(*args)
        except BaseException:self.window.write_failed=True;raise

    async def _position_guarded_claw(self,args,check):
        self.window.entry('set_claw');check()
        self.window.native_entries['set_claw']+=1
        try:return await self.native.set_claw(*args)
        except BaseException:self.window.write_failed=True;raise

    def __getattr__(self,name):
        if name not in WebWindow.READS|{'movej','set_claw','stop_move'}:raise AttributeError(name)
        method=getattr(self.native,name)
        async def call(*args,**kwargs):
            self.window.entry(name)
            self.window.native_entries[name]+=1
            try:return await method(*args,**kwargs)
            except BaseException:
                if name in ('movej','set_claw','stop_move'):self.window.write_failed=True
                raise
        return call


def asset_hashes(directory):
    directory=Path(directory).resolve(strict=True)
    if not (directory/'index.html').is_file():raise ValueError('position_web_build_missing')
    return {str(p.resolve()):single.sha(p) for p in sorted(directory.rglob('*')) if p.is_file()}


def preview(p,web_directory):
    policy=single.position_arrival_policy('coarse_grasp')
    runtime=dict(p['runtime_hashes']);runtime['scripts/run_finite_position_web.py']=single.sha(Path(__file__))
    return dict(schema=1,scope='bounded_web_finite_position_and_gripper',target_ip=p['settings'].ip,
        host='127.0.0.1',port=8769,output=str(p['output']),runtime='LEBAI',
        authorization_expires_utc=p['authorization_expires_utc'],duration_s=DURATION_NS/1e9,
        cleanup_reserve_s=CLEANUP_NS/1e9,max_connections=1,max_movej_entries=MOVE_LIMIT,
        max_claw_entries=CLAW_LIMIT,max_stop_entries=STOP_LIMIT,max_read_entries=READ_LIMIT,
        input_hashes={**p['input_hashes'],str(COMMON_CONFIG.resolve()):single.sha(COMMON_CONFIG)},
        application_settings_binding=json.loads(json.dumps(asdict(effective_settings(p)))),
        runtime_hashes=runtime,web_hashes=asset_hashes(web_directory),
        settings_binding=json.loads(json.dumps(asdict(p['settings']))),model_sha256=p['model'].model_sha256,
        sdk_binary_sha256=single.identity.SDK_SHA,arrival_policy=asdict(policy),step_m=.002,
        input_freshness_ms=100,feedback_freshness_ms=100,movej_reply_ms=200,planning_s=1,
        held_limit_s=10,held_step_limit=10,completion_s=5,prepare_timeout_s=60,
        prepare_tolerance_deg=.1,prepare_speed_radps=.05,prepare_acceleration_radps2=.2,
        normal_release='finish_current_step_no_next_target',unlock_moves=False,
        prepare_requires_separate_button=True,prepare_preliminary_stop=True,
        progressive_gripper=True,home_enabled=False,
        tool_spin_enabled=False,automatic_retry=False,automatic_resume=False,
        stop_limits_ms=[200,300,500],motion_time_stop_verified=False,hardware_verified=False,
        motion_authorized=False,requires_fresh_onsite_authorization=True,
        runtime_policy='full_sha_before_connect_metadata_before_write',
        metadata_preserving_tampering_detected_at_write=False)


def approve(p,row,approved,expected_hash,expires,confirm):
    """Pure validation; no SDK import, server startup, connection or writes."""
    expiry=datetime.fromisoformat(expires.replace('Z','+00:00'));now=datetime.now(UTC)
    if (confirm!=CONFIRM or expiry.utcoffset()!=timedelta(0)
            or not now<expiry<=now+timedelta(minutes=10)
            or expires!=row['authorization_expires_utc']):raise ValueError('position_web_approval_expired')
    approved=Path(approved).resolve(strict=True)
    if single.sha(approved)!=expected_hash or json.loads(approved.read_bytes())!=row:
        raise ValueError('position_web_preview_mismatch')
    paths={**row['input_hashes'],**{str(ROOT/k):v for k,v in row['runtime_hashes'].items()},
        **row['web_hashes'],str(approved):expected_hash}
    before=single.metadata(paths)
    if any(single.sha(path)!=value for path,value in paths.items()) or single.metadata(paths)!=before:
        raise ValueError('position_web_sources_changed')
    def identity_check():
        if single.metadata(paths)!=before:raise ValueError('position_web_identity_changed')
    return identity_check


def effective_settings(p):
    from dataclasses import replace
    from app.config import Settings
    return replace(Settings.load(COMMON_CONFIG),backend='lebai',lebai=p['settings'],segmented_target=None,
        transactional_target=None)


def create_web_app(p,web_directory,window,client_factory):
    from app.main import create_app
    from fastapi.staticfiles import StaticFiles
    settings=effective_settings(p)
    async def factory(ip):
        if ip!=p['settings'].ip:raise ValueError('position_web_target_changed')
        window.entry('connect')
        client=await client_factory(ip)
        window.check()
        return WindowClient(client,window)
    app=create_app(settings=settings,client_factory=factory,backend_label='LEBAI',
        real_finite_position=True,real_position_authorize=window.check,real_position_model=p['model'])
    @app.get('/control/identity')
    async def control_identity():
        return dict(runtime='LEBAI',target_strategy='finite_position_web',hardware_verified=False,
            motion_time_stop_verified=False,normal_release='finish_current_step_no_next_target',
            remaining_s=max(0,(DURATION_NS-(window.clock()-window.started))/1e9),
            native_entries=dict(window.native_entries),budget_entries=dict(window.entries),home_enabled=False)
    app.mount('/',StaticFiles(directory=web_directory,html=True),name='bounded-real-finite-position')
    return app


async def run_server_window(server,window):
    """Bound the whole server/lifespan, including cancellation-resistant jobs."""
    task=asyncio.create_task(server.serve(),name='bounded-real-web-server')
    try:
        remaining=max(0,(DURATION_NS-(window.clock()-window.started))/1e9)
        done,_=await asyncio.wait({task},timeout=remaining)
        if done:
            await task
            return dict(cleanup_unresolved=False)
        server.should_exit=True
        remaining=max(0,(DURATION_NS+CLEANUP_NS-(window.clock()-window.started))/1e9)
        done,_=await asyncio.wait({task},timeout=remaining)
        if done:
            await task
            return dict(cleanup_unresolved=False)
        task.cancel()
        return dict(cleanup_unresolved=True,native_connection_release_verified=False)
    finally:
        if not task.done():
            server.should_exit=True
            if not task.cancelling():task.cancel()


async def serve(p,row,args):
    import uvicorn
    check=approve(p,row,args.approved_preview,args.expected_preview_sha256,args.expires_utc,args.confirm)
    if single.existing_rpc_connections()!=0:raise ValueError('position_web_existing_rpc')
    module=single.load_sdk()  # verified SDK identity, no initialization until approved factory
    check()
    if datetime.now(UTC)>=datetime.fromisoformat(args.expires_utc):raise ValueError('position_web_approval_expired')
    p['output'].mkdir(parents=True,exist_ok=False)
    (p['output']/'admission.json').write_text(json.dumps(row,indent=2),encoding='utf-8')
    window=WebWindow(check)
    async def connect(ip):
        single.identity.require_execution_runtime();window.check();module.init();window.check()
        window.native_entries['connect']+=1
        return await module.connect(ip,False)
    app=create_web_app(p,args.web_directory,window,connect)
    server=uvicorn.Server(uvicorn.Config(app,host='127.0.0.1',port=8769,log_level='info',
        timeout_graceful_shutdown=1))
    report=dict(cleanup_unresolved=True)
    try:report=await run_server_window(server,window)
    finally:
        backend=getattr(app.state,'backend',None);control=getattr(app.state,'control',None)
        report.update(native_entries=dict(window.native_entries),budget_entries=dict(window.entries),
            write_failed=window.write_failed,hardware_verified=False,
            log_session_dir=getattr(app.state,'log_session_dir',None),
            control_fault=getattr(control,'_fault',None),latched_fault=getattr(backend,'_latched_fault',None),
            motion_unknown=sorted(getattr(getattr(backend,'_motion_calls',None),'unknown_methods',())),
            snapshot_unknown=sorted(getattr(getattr(backend,'_snapshot_calls',None),'unknown_methods',())))
        (p['output']/'web-result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return report


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('config','reference-survey','ready-pose','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--web-directory',type=Path,default=ROOT/'web/dist')
    p.add_argument('--execute',action='store_true')
    p.add_argument('--approved-preview',type=Path);p.add_argument('--expected-preview-sha256')
    p.add_argument('--expires-utc');p.add_argument('--confirm')
    return p


def main(argv=None):
    args=parser().parse_args(argv)
    p=single.prepare(args.config,args.reference_survey,args.ready_pose,args.output,
        expires=args.expires_utc,arrival_policy='coarse_grasp')
    row=preview(p,args.web_directory)
    if not args.execute:print(json.dumps(row,indent=2));return 0
    if not all((args.approved_preview,args.expected_preview_sha256,args.expires_utc,args.confirm)):
        raise ValueError('position_web_explicit_approval_required')
    # Validate before any server or SDK side effects.
    approve(p,row,args.approved_preview,args.expected_preview_sha256,args.expires_utc,args.confirm)
    from app.commissioning.qpc_event_loop import new_qpc_loop
    loop=new_qpc_loop();task=loop.create_task(serve(p,row,args));interrupted=False
    try:report=loop.run_until_complete(task)
    except KeyboardInterrupt:
        interrupted=True;task.cancel()
        try:loop.run_until_complete(asyncio.wait({task},timeout=5.))
        except KeyboardInterrupt:pass
        report=dict(cleanup_unresolved=True,interrupted=True,native_connection_release_verified=False)
    finally:pending=single.close_loop_bounded(loop)
    report.update(pending_tasks_at_process_exit=pending)
    (p['output']/'web-result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    return 130 if interrupted else 2 if (pending or report.get('cleanup_unresolved') or report.get('control_fault')
        or report.get('latched_fault') or report.get('write_failed')) else 0


if __name__=='__main__':raise SystemExit(main())
