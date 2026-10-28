"""Real selector exercised only with explicitly injected in-memory clients."""
from dataclasses import replace
import math
import pytest
from app.control.finite_position_types import PositionIntent
from app.robots.base import BackendCommandError
from tests.robots.finite_position_fixture import fixture


def profile():
    s,c,clock=fixture()
    s=replace(s,ip='192.168.42.10',control=replace(s.control,
        max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    return s,c,clock


async def make(authorize=lambda:None):
    from app.robots.finite_position_web import RealFinitePositionAdapter
    s,c,clock=profile()
    async def factory(ip):
        assert ip==s.ip
        return c
    a=RealFinitePositionAdapter(s,client_factory=factory,position_model=c.model,
        authorize=authorize,clock=clock.now_ns,sleep=clock.sleep)
    return a,c,clock


async def test_real_selector_natural_arrival_and_explicit_identity():
    a,c,clock=await make()
    try:
        await a.connect();a.bind_position_generation(1)
        intent=PositionIntent('web',1,1,clock.now_ns(),'held',(1.,0.,0.))
        plan=await a.plan_step(intent,authorize=lambda:None)
        result=await a.execute_step(plan,before_entry=lambda:None,
            monitor_authorize=lambda:None,on_entered=lambda:None)
        assert result.complete,result
        assert [r[0] for r in c.write_calls]==['movej']
        assert a.position_policy=='finite_position_web' and a.position_candidate
        assert a.position_arrival_policy.name=='coarse_grasp'
        assert (await a.get_state()).backend=='LEBAI'
    finally:await a.disconnect()


async def test_expired_scope_denies_connect_and_home_never_moves():
    def denied():raise ValueError('position_web_scope_expired')
    a,c,_=await make(denied)
    with pytest.raises(ValueError,match='scope_expired'):await a.connect()
    assert not c.write_calls
    a,_,_=await make()
    with pytest.raises(BackendCommandError,match='home_disabled'):await a.home(None,None)
    with pytest.raises(BackendCommandError,match='stream_disabled'):await a.submit_tcp(None,1)


async def test_revocation_after_planning_is_zero_native_move():
    allowed=[True]
    def guard():
        if not allowed[0]:raise ValueError('position_web_scope_expired')
    a,c,clock=await make(guard)
    try:
        await a.connect();a.bind_position_generation(1)
        plan=await a.plan_step(PositionIntent('web',1,1,clock.now_ns(),'held',(1,0,0)),authorize=lambda:None)
        allowed[0]=False
        result=await a.execute_step(plan,before_entry=lambda:None,
            monitor_authorize=lambda:None,on_entered=lambda:None)
        assert not result.complete and not result.entered
        assert not c.write_calls
    finally:await a.disconnect()


async def test_actual_native_boundary_rechecks_age_after_scope_check_cost():
    a,c,clock=await make()
    async def guarded(args,check):
        await clock.sleep(.101)
        check()
        return await c.movej(*args)
    c._position_guarded_movej=guarded
    try:
        await a.connect();a.bind_position_generation(1)
        p=await a.plan_step(PositionIntent('web',1,1,clock.now_ns(),'held',(1,0,0)),authorize=lambda:None)
        result=await a.execute_step(p,before_entry=lambda:None,monitor_authorize=lambda:None,on_entered=lambda:None)
        assert not result.entered and result.reason=='position_snapshot_expired'
        assert c.write_calls==[]
    finally:await a.disconnect()


async def test_claw_final_boundary_keeps_one_hundred_ms_feedback_limit():
    a,c,clock=await make()
    async def guarded(args,check):
        await clock.sleep(.101);check()
        return await c.set_claw(*args)
    c._position_guarded_claw=guarded
    try:
        await a.connect()
        with pytest.raises(BackendCommandError):await a.set_gripper(.2)
        assert not any(r[0]=='set_claw' for r in c.write_calls)
    finally:await a.disconnect()


async def test_prepare_final_boundary_rechecks_authority_after_identity_cost():
    from app.robots.base import HomeOptions
    a,c,clock=await make()
    allowed=[True]
    def authorize():
        if not allowed[0]:raise RuntimeError('revoked')
    async def guarded(args,check):
        allowed[0]=False;check()
        return await c.movej(*args)
    c._position_guarded_movej=guarded
    try:
        await a.connect()
        with pytest.raises(BackendCommandError):
            await a.prepare(HomeOptions(.05,60.,math.radians(.1),.02,.3),lambda phase:None,authorize=authorize)
        assert not any(r[0]=='movej' for r in c.write_calls)
    finally:await a.disconnect()


def test_explicit_selection_preserves_fake_and_default_backend():
    from app.main import build_backend
    from app.recording.noop import NoopRecorder
    from app.config import Settings
    from app.robots.finite_position_web import RealFinitePositionAdapter
    s,c,_=profile();settings=replace(Settings.load(),backend='lebai',lebai=s)
    async def factory(ip):return c
    default=build_backend(settings,NoopRecorder(),factory)
    assert type(default).__name__=='RealLebaiAdapter'
    web=build_backend(settings,NoopRecorder(),factory,real_finite_position=True,
        real_position_authorize=lambda:None,real_position_model=c.model)
    assert isinstance(web,RealFinitePositionAdapter)
    with pytest.raises(ValueError):build_backend(settings,NoopRecorder(),factory,real_finite_position=True)


async def test_controller_status_marks_real_web_policy():
    from app.control.robot_control import RobotControl,LatestVRFrame
    from app.recording.noop import NoopRecorder
    a,c,clock=await make()
    control=RobotControl(backend=a,latest=LatestVRFrame(),clock=clock,recorder=NoopRecorder())
    assert control._position_status_message().policy=='finite_position_web'


async def test_ordinary_state_read_can_cross_normal_prepare_stop_generation():
    a,c,clock=await make()
    try:
        await a.connect();original=c.get_kin_data
        async def changed():
            a._pump.invalidate()
            return await original()
        c.get_kin_data=changed
        state=await a.get_state()
        assert state.backend=='LEBAI' and not a._snapshot_calls.unknown_methods
    finally:await a.disconnect()


def test_production_diagnostics_wrapper_preserves_recording_health(tmp_path):
    from app.diagnostics.recorder import DiagnosticsRecorder
    from app.diagnostics.store import DiagnosticsStore
    from app.recording.commissioning import CommissioningRecorder
    primary=CommissioningRecorder(tmp_path,metadata={})
    recorder=DiagnosticsRecorder(primary,DiagnosticsStore())
    primary.dropped_normal_events=1
    assert recorder.dropped_normal_events==1
    primary.fatal_error='disk_failed'
    assert recorder.fatal_error=='disk_failed'


async def test_wrapped_recording_drop_prevents_next_step(tmp_path):
    from app.diagnostics.recorder import DiagnosticsRecorder
    from app.diagnostics.store import DiagnosticsStore
    from app.recording.commissioning import CommissioningRecorder
    from app.control.finite_position_session import FinitePositionSession
    from tests.robots.test_finite_position_adapter import prepared
    a,c,clock,_=await prepared()
    primary=CommissioningRecorder(tmp_path,metadata={});primary.dropped_normal_events=1
    recorder=DiagnosticsRecorder(primary,DiagnosticsStore())
    session=FinitePositionSession(a,a.position_binding,clock=clock.now_ns,recorder=recorder,authorize=lambda:None)
    try:
        session.offer(PositionIntent('web',1,1,clock.now_ns(),'held',(1,0,0)))
        await session.advance()
        assert session.phase=='fault' and session.reason=='position_recording_unavailable'
        assert c.write_calls==[]
    finally:await a.disconnect()


@pytest.mark.parametrize('planning_delay_s',[0.,.12])
async def test_app_unlock_prepare_translation_release_and_progressive_claw(monkeypatch,planning_delay_s):
    """Production app/controller and real selector, with no native SDK loaded."""
    import asyncio,time
    from unittest.mock import AsyncMock
    from app.main import create_app
    import app.main as main_module
    from app.config import Settings,DEFAULT_CONFIG
    from app.control.robot_control import RobotControl
    from app.schemas.messages import VRFrame
    from app.api.teleop_ws import _parse_message
    from app.recording.noop import NoopRecorder
    from tests.robots.finite_position_fixture import PositionClient
    class Clock:
        def now_ns(self):return time.monotonic_ns()
    c=PositionClient(Clock());s,_,_=profile()
    s=replace(s,teleop_ready_q=tuple(c.kin_data['actual_joint_pose']))
    settings=replace(Settings.load(DEFAULT_CONFIG),backend='lebai',lebai=s)
    monkeypatch.setattr(main_module,'build_recorder',lambda *a,**kw:NoopRecorder())
    monkeypatch.setattr(RobotControl,'start',AsyncMock())
    async def factory(ip):assert ip==s.ip;return c
    app=create_app(settings=settings,client_factory=factory,real_finite_position=True,
        real_position_authorize=lambda:None,real_position_model=c.model)
    async with app.router.lifespan_context(app):
        control=app.state.control;seq=0;intent_seq=0;held=False
        def position(event):
            nonlocal intent_seq
            intent_seq+=1
            control.accept_position_input(_parse_message(dict(v=1,type='finite_position_input',
                session_id='web',generation=gen,seq=intent_seq,kind=event,
                vector=[1,0,0] if event=='held' else [0,0,0])),control.clock.now_ns())
        original_plan=app.state.backend.plan_step
        async def delayed_plan(*args,**kwargs):
            await asyncio.sleep(planning_delay_s)
            return await original_plan(*args,**kwargs)
        app.state.backend.plan_step=delayed_plan
        def heartbeat():
            nonlocal seq
            seq+=1
            frame=VRFrame(v=1,type='vr_frame',session_id='desktop',seq=seq,client_mono_ms=seq,
                tracking_valid=True,visibility='visible',right=dict(p=[0,0,0],q=[0,0,0,1],grip=False,trigger=0))
            control.latest.publish(frame,control.clock.now_ns())
            if held:position('held')  # finite intent has its own lease, separate from neutral VR health frames
        async def keep_fresh():
            while True:heartbeat();await asyncio.sleep(.02)
        heartbeat();refresh=asyncio.create_task(keep_fresh())
        try:
            await control.tick()
            await control.arm()
            assert c.write_calls==[] and control._preparation_unlocked()
            await control.prepare()
            assert [r[0] for r in c.write_calls]==['stop_move','movej']
            status=await control.state_message()
            assert status.manual_input_ready and status.finite_position.policy=='finite_position_web'
            gen=control.control_generation
            held=True;position('held')
            released=False
            for _ in range(200):
                assert control._fault is None,control._fault
                if sum(r[0]=='movej' for r in c.write_calls)==2 and not released:
                    held=False;position('release');released=True
                await control.tick();await asyncio.sleep(.03)
                if released and not control.finite_position_session.busy:break
            assert released and not control._fault
            if planning_delay_s:assert intent_seq>2, 'the slow plan must receive new held samples before release'
            assert not control.finite_position_session.busy,str((control.finite_position_session.view(),
                [(f.f_code.co_name,f.f_lineno) for f in control.finite_position_session.worker.get_stack()],
                c.robot_state,c.motion_state,c.kin_data['actual_joint_pose'],
                (time.monotonic_ns()-c.start_ns)/1e9,app.state.backend._snapshot.actual_tcp.p))
            assert [r[0] for r in c.write_calls]==['stop_move','movej','movej']
            await control.command_gripper(.2);await control.command_gripper(.35)
            assert [r[0] for r in c.write_calls][-2:]==['set_claw','set_claw']
            assert c.write_calls[-2][2]!=c.write_calls[-1][2]
        finally:
            held=False;refresh.cancel();await asyncio.gather(refresh,return_exceptions=True)
