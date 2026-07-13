import asyncio
import builtins
import importlib

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import BackendState
from app.sim.ik import IKError
from app.sim.kinematics import forward_pose


@pytest.mark.asyncio
async def test_sim_adapter_accepts_tcp_and_gripper_commands() -> None:
    adapter = SimRobotAdapter()
    await adapter.connect()
    try:
        target = forward_pose(adapter.robot.model.home_q, adapter.robot.model)
        await adapter.command_tcp(target, command_id=1)
        await adapter.set_gripper(0.7)

        state = await adapter.get_state()

        assert state.ack_seq == 1
        assert state.gripper == pytest.approx(0.7)
        assert state.actual_tcp == target
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "error_code",
    ["ik_unreachable", "ik_singular", "joint_safety_window"],
)
async def test_sim_adapter_translates_ik_errors_without_mutating_command(
    monkeypatch, error_code: str
) -> None:
    import app.robots.sim_adapter as sim_adapter_module

    adapter = SimRobotAdapter()
    original_target = adapter.robot.q + 0.1
    adapter.robot.set_target_q(original_target)
    adapter.command_id = 7
    target = forward_pose(adapter.model.home_q, adapter.model)

    def fail_ik(*args, **kwargs):
        raise IKError(error_code)

    monkeypatch.setattr(sim_adapter_module, "solve_ik", fail_ik)

    with pytest.raises(BackendCommandError, match=f"^{error_code}$"):
        await adapter.command_tcp(target, command_id=99)

    assert adapter.robot.target_q == pytest.approx(original_target)
    assert adapter.command_id == 7


@pytest.mark.asyncio
async def test_sim_adapter_normalizes_gripper_to_unit_interval() -> None:
    adapter = SimRobotAdapter()

    await adapter.set_gripper(-0.3)
    assert (await adapter.get_state()).gripper == 0.0
    await adapter.set_gripper(1.4)
    assert (await adapter.get_state()).gripper == 1.0


@pytest.mark.asyncio
async def test_sim_adapter_stop_decelerates_and_reports_idle() -> None:
    adapter = SimRobotAdapter()
    adapter.robot.set_target_q(adapter.robot.q + 0.5)
    for _ in range(20):
        adapter.robot.step(0.02)

    assert (await adapter.get_state()).robot_state is BackendState.MOVING
    await adapter.stop(StopReason.GRIP_RELEASED)
    for _ in range(30):
        adapter.robot.step(0.02)

    assert adapter.robot.target_q is None
    assert (await adapter.get_state()).robot_state is BackendState.IDLE


@pytest.mark.asyncio
async def test_sim_adapter_run_uses_fixed_steps_and_absolute_deadlines(monkeypatch) -> None:
    import app.robots.sim_adapter as sim_adapter_module

    class FakeLoop:
        def __init__(self) -> None:
            self.times = iter((10.0, 10.005, 10.026))

        def time(self) -> float:
            return next(self.times)

    adapter = SimRobotAdapter()
    steps: list[float] = []
    sleeps: list[float] = []

    def record_step(dt: float) -> None:
        steps.append(dt)

    async def record_sleep(delay: float) -> None:
        sleeps.append(delay)
        if len(sleeps) == 2:
            raise asyncio.CancelledError

    monkeypatch.setattr(adapter.robot, "step", record_step)
    monkeypatch.setattr(sim_adapter_module.asyncio, "get_running_loop", FakeLoop)
    monkeypatch.setattr(sim_adapter_module.asyncio, "sleep", record_sleep)

    with pytest.raises(asyncio.CancelledError):
        await adapter._run()

    assert steps == [0.02, 0.02]
    assert sleeps == pytest.approx([0.015, 0.014])


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("method_name", "args"),
    [
        ("connect", ()),
        ("disconnect", ()),
        ("command_tcp", (None, 1)),
        ("set_gripper", (0.5,)),
        ("stop", (StopReason.SHUTDOWN,)),
        ("get_state", ()),
    ],
)
async def test_real_adapter_is_hard_disabled(method_name: str, args: tuple[object, ...]) -> None:
    method = getattr(RealLebaiAdapter(), method_name)

    with pytest.raises(BackendCommandError, match="^real_robot_disabled$"):
        await method(*args)


def test_real_adapter_module_discovery_does_not_import_lebai_sdk(monkeypatch) -> None:
    real_import = builtins.__import__

    def reject_lebai_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name.startswith("lebai"):
            raise AssertionError("Milestone 1 must not import the Lebai SDK")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", reject_lebai_import)
    module = importlib.import_module("app.robots.lebai_adapter")
    importlib.reload(module)
