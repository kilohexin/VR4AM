from types import SimpleNamespace as NS
import pytest
from app.digital_twin.virtual_claw import DockerArmWithVirtualClaw
from app.robots.finite_position_docker import DockerFinitePositionAdapter
from tests.control.test_grip_release_intent import active_control
from tests.control.test_robot_control import frame


async def rig():
    c,latest,b,clock=await active_control()
    latest.publish(frame(3,True),clock.now_ns())
    a=object.__new__(DockerFinitePositionAdapter)
    a._docker_native=True;a.position_pvat_vr=True
    a._docker_ready_client=DockerArmWithVirtualClaw(object(),clock=clock.now_ns)
    a.position_binding=object();a._position_health=lambda _:None
    b.parallel_virtual_claw_available=a.parallel_virtual_claw_available
    b.command_parallel_virtual_claw=a.command_parallel_virtual_claw
    c.finite_position_session=NS(busy=True)
    return c,latest,b,clock,a


async def test_local_virtual_claw_can_move_without_stopping_active_arm():
    c,_,b,clock,a=await rig()
    await c.command_gripper(.4)
    clock.advance_ms(300)
    assert (await a._docker_ready_client.get_claw())['amplitude']==60
    assert b.stops==[] and b.gripper_commands==[] and c.mode.value=='ACTIVE'


async def test_parallel_model_still_requires_fresh_input_and_current_request():
    c,_,_,clock,a=await rig();clock.advance_ms(101)
    with pytest.raises(RuntimeError):await c.command_gripper(.8)
    assert a._docker_ready_client._target==0
    with pytest.raises(RuntimeError):await c.command_gripper(.8,is_current=lambda:False)
    assert a._docker_ready_client._target==0


async def test_unverified_or_nonlocal_claw_does_not_gain_parallel_capability():
    c,_,b,_,a=await rig();a._docker_ready_client=object()
    assert not b.parallel_virtual_claw_available()
    with pytest.raises(RuntimeError,match='position_busy'):await c.command_gripper(.8)
