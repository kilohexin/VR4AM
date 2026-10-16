"""Two explicit operator actions; entirely synthetic, no robot connection."""
import asyncio
import pytest
from app.control.robot_control import LatestVRFrame,RobotControl
from app.recording.noop import NoopRecorder
from app.schemas.messages import TeleopMode
from tests.control.test_prepare_before_arm import PreparingBackend
from tests.control.test_robot_control import FakeClock,frame

async def rig():
    b,c,l=PreparingBackend(),FakeClock(),LatestVRFrame()
    control=RobotControl(backend=b,latest=l,clock=c,recorder=NoopRecorder(),
        prepare_on_arm=True,explicit_preparation=True)
    await control.connect();l.publish(frame(1,False),c.now_ns());await control.tick()
    return control,b,c,l

async def test_unlock_only_grants_preparation_without_motion_or_manual_authority():
    ctrl,b,_,_=await rig();before=len(b.stops)
    await ctrl.arm()
    assert b.prepare_count==0 and len(b.stops)==before
    state=await ctrl.state_message()
    assert state.mode is TeleopMode.READY and not state.manual_input_ready
    assert state.preparation_required and state.preparation_unlocked

async def test_prepare_requires_separate_unlock_and_consumes_it_once():
    ctrl,b,c,l=await rig()
    with pytest.raises(RuntimeError,match='prepare_requires_unlock'):await ctrl.prepare()
    await ctrl.arm();task=asyncio.create_task(ctrl.prepare());await b.entered.wait()
    try:
        assert b.prepare_count==1 and not (await ctrl.state_message()).manual_input_ready
        with pytest.raises(RuntimeError):await ctrl.prepare()
        l.publish(frame(2,False),c.now_ns());b.finish.set();await task
        assert ctrl.mode is TeleopMode.ARMED and (await ctrl.state_message()).manual_input_ready
        with pytest.raises(RuntimeError):await ctrl.prepare()
    finally:b.finish.set();await asyncio.gather(task,return_exceptions=True)

@pytest.mark.parametrize('revoke',['stop','stale','session','grip'])
async def test_unlock_cannot_survive_revocation(revoke):
    ctrl,b,c,l=await rig();await ctrl.arm()
    if revoke=='stop':await ctrl.disarm()
    else:
        if revoke=='stale':c.advance_ms(101)
        else:l.publish(frame(2,revoke=='grip',session_id='other' if revoke=='session' else 'test'),c.now_ns())
        await ctrl.tick()
    with pytest.raises(RuntimeError):await ctrl.prepare()
    assert b.prepare_count==0

async def test_preparation_has_separate_timeout_and_speed_from_home():
    ctrl,b,c,l=await rig();captured=[]
    original=b.prepare
    async def measured(options,on_phase,**kwargs):captured.append(options);await original(options,on_phase,**kwargs)
    b.prepare=measured
    await ctrl.arm();task=asyncio.create_task(ctrl.prepare());await b.entered.wait()
    l.publish(frame(2,False),c.now_ns());b.finish.set();await task
    assert captured[0].timeout_s==60 and captured[0].max_speed_radps==.05
    assert captured[0].position_tolerance_rad==.001
    assert ctrl.home_options.timeout_s==15
