import pytest
from app.api.teleop_ws import _parse_message
from tests.control.test_grip_release_intent import active_control,frame

def test_explicit_web_claw_request_uses_a_separate_bounded_message():
    message=_parse_message(dict(v=1,type='gripper_request',request_id='claw-1',value=0.))
    assert message.value==0.

@pytest.mark.parametrize('value',[True,-.1,1.1,float('nan')])
def test_invalid_claw_values_are_not_accepted(value):
    with pytest.raises(ValueError):
        _parse_message(dict(v=1,type='gripper_request',request_id='claw-1',value=value))

async def test_claw_command_can_open_in_hold_only_after_the_existing_release_stop():
    control,latest,backend,clock=await active_control()
    latest.publish(frame(3,False),clock.now_ns())
    await control.tick()
    assert control.mode.value=='HOLD'
    await control.command_gripper(0.)
    assert backend.gripper_commands==[0.]

async def test_web_claw_backend_fault_keeps_the_control_fault_path():
    import asyncio
    from types import SimpleNamespace
    from unittest.mock import AsyncMock
    from fastapi import WebSocketDisconnect
    from app.api import teleop_ws
    from app.robots.base import BackendCommandError
    faulted=asyncio.Event()
    control=SimpleNamespace(command_gripper=AsyncMock(side_effect=BackendCommandError('stop_unverified')),
        _enter_fault=AsyncMock(side_effect=lambda _:faulted.set()))
    first=True
    async def receive():
        nonlocal first
        if first:
            first=False
            return dict(v=1,type='gripper_request',request_id='claw-1',value=1.)
        await asyncio.wait_for(faulted.wait(),1)
        raise WebSocketDisconnect()
    websocket=SimpleNamespace(app=SimpleNamespace(state=object()),send_json=AsyncMock(),receive_json=receive)
    await teleop_ws._receive_messages(websocket,control,asyncio.Event(),asyncio.Lock())
    control._enter_fault.assert_awaited_once_with('stop_unverified')

async def test_hold_claw_reaches_the_real_adapter_final_bound_guard_with_fake_sdk():
    from app.control.robot_control import LatestVRFrame,RobotControl
    from app.control.coordinate_mapper import CoordinateMapper
    from app.control.transactional_settings import SegmentedSettings
    from app.recording.noop import NoopRecorder
    from tests.control.test_segmented_control import rig,limiter,OPTIONS
    adapter,client,clock,_box,_strategy=await rig()
    latest=LatestVRFrame()
    control=RobotControl(backend=adapter,latest=latest,clock=clock,recorder=NoopRecorder(),
        mapper=CoordinateMapper(),limiter=limiter(),segmented_settings=SegmentedSettings(OPTIONS))
    try:
        await control.connect()
        latest.publish(frame(1,False),clock.now_ns());await control.tick();await control.arm()
        latest.publish(frame(2,True),clock.now_ns());await control.tick()
        latest.publish(frame(3,False),clock.now_ns());await control.tick()
        latest.publish(frame(4,False),clock.now_ns());await control.tick()
        assert control.mode.value=='HOLD'
        await control.command_gripper(0.)
        assert [c[0] for c in client.write_calls]==['stop_move','set_claw']
    finally:await adapter.disconnect()

async def test_manual_input_is_unavailable_during_the_original_release_stop():
    import asyncio
    control,latest,backend,clock=await active_control()
    state=await backend.get_state()
    assert control._compose_state_message(state).manual_input_ready
    entered,finish=asyncio.Event(),asyncio.Event()
    original=backend.stop
    async def stop(reason):
        entered.set();await finish.wait();await original(reason)
    backend.stop=stop
    latest.publish(frame(3,False),clock.now_ns())
    tick=asyncio.create_task(control.tick())
    try:
        await asyncio.wait_for(entered.wait(),1)
        assert not control._compose_state_message(state).manual_input_ready
    finally:
        finish.set();await tick
    assert control._compose_state_message(state).manual_input_ready
