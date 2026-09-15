from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
from typing import Awaitable, Callable
from unittest.mock import AsyncMock

import pytest

from app.robots.base import HomeOptions, RobotBackend, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import Pose
from app.sim.kinematics import forward_pose
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings


HOME_OPTIONS = HomeOptions(
    max_speed_radps=0.1,
    timeout_s=2.0,
    position_tolerance_rad=0.01,
    velocity_tolerance_radps=0.02,
    stable_seconds=0.04,
)


class FakeClock:
    def __init__(self) -> None:
        self.value_ns = 0

    def now_ns(self) -> int:
        return self.value_ns

    async def sleep(self, seconds: float) -> None:
        self.value_ns += int(seconds * 1_000_000_000)
        await asyncio.sleep(0)


@dataclass
class BackendHarness:
    name: str
    backend: RobotBackend
    reachable_target: Pose
    required_capabilities: frozenset[str]
    wait_for_command: Callable[[], Awaitable[None]]
    assert_command_observed: Callable[[], None]
    assert_stopped: Callable[[], None]
    assert_gripper_observed: Callable[[], Awaitable[None]]
    close: Callable[[], Awaitable[None]]


async def make_harness(name: str) -> BackendHarness:
    if name == "simulator":
        adapter = SimRobotAdapter()
        target = forward_pose(
            adapter.robot.q
            + (0.02, -0.01, 0.015, 0.01, -0.005, 0.01),
            adapter.model,
        )

        async def wait_for_command() -> None:
            adapter.robot.step(0.02)

        def assert_command_observed() -> None:
            assert adapter.command_id == 17
            assert adapter.robot.target_qd is not None

        def assert_stopped() -> None:
            assert adapter.robot.target_q is None
            assert adapter.robot.target_qd is None

        async def assert_gripper_observed() -> None:
            assert (await adapter.get_state()).gripper == pytest.approx(0.7)

        return BackendHarness(
            name=name,
            backend=adapter,
            reachable_target=target,
            required_capabilities=frozenset(
                {"command_tcp", "home", "gripper"}
            ),
            wait_for_command=wait_for_command,
            assert_command_observed=assert_command_observed,
            assert_stopped=assert_stopped,
            assert_gripper_observed=assert_gripper_observed,
            close=adapter.disconnect,
        )

    if name == "fake_lebai":
        client = FakeLebaiClient.idle()
        client.ik_results = deque(
            [[0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]]
        )
        clock = FakeClock()
        adapter = RealLebaiAdapter(
            control_settings(),
            client_factory=AsyncMock(return_value=client),
            clock=clock.now_ns,
            sleep=clock.sleep,
        )
        await adapter.connect()

        async def wait_for_command() -> None:
            await client.wait_for_write("move_pvat")

        def assert_command_observed() -> None:
            assert any(call[0] == "move_pvat" for call in client.write_calls)

        def assert_stopped() -> None:
            methods = [call[0] for call in client.write_calls]
            assert methods.count("stop_move") == 1
            last_stop = max(
                index
                for index, method in enumerate(methods)
                if method == "stop_move"
            )
            assert "move_pvat" not in methods[last_stop + 1 :]

        async def assert_gripper_observed() -> None:
            assert ("set_claw", 30, 30) in client.write_calls

        return BackendHarness(
            name=name,
            backend=adapter,
            reachable_target=Pose(
                p=(0.301, 0.0, 0.4),
                q=(0.0, 0.0, 0.0, 1.0),
            ),
            required_capabilities=frozenset({"home", "gripper", "pvat"}),
            wait_for_command=wait_for_command,
            assert_command_observed=assert_command_observed,
            assert_stopped=assert_stopped,
            assert_gripper_observed=assert_gripper_observed,
            close=adapter.disconnect,
        )

    raise AssertionError(f"unknown harness: {name}")


@pytest.mark.asyncio
@pytest.mark.parametrize("backend_name", ["simulator", "fake_lebai"])
async def test_backend_contract_preflight_is_read_only_and_capable(
    backend_name: str,
) -> None:
    harness = await make_harness(backend_name)
    try:
        before = await harness.backend.get_state()

        preflight = await harness.backend.preflight()
        after = await harness.backend.get_state()

        assert preflight.ready is True
        assert preflight.actual_q == before.actual_q
        assert after.actual_q == before.actual_q
        assert harness.required_capabilities <= frozenset(
            preflight.capabilities
        )
    finally:
        await harness.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("backend_name", ["simulator", "fake_lebai"])
async def test_backend_contract_command_is_observable_and_acknowledged(
    backend_name: str,
) -> None:
    harness = await make_harness(backend_name)
    try:
        await harness.backend.command_tcp(
            harness.reachable_target,
            command_id=17,
        )
        await harness.wait_for_command()

        harness.assert_command_observed()
        assert (await harness.backend.get_state()).ack_seq == 17
    finally:
        await harness.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("backend_name", ["simulator", "fake_lebai"])
async def test_backend_contract_repeated_stop_leaves_no_pending_motion(
    backend_name: str,
) -> None:
    harness = await make_harness(backend_name)
    try:
        await harness.backend.command_tcp(
            harness.reachable_target,
            command_id=17,
        )
        await harness.wait_for_command()

        await harness.backend.stop(StopReason.GRIP_RELEASED)
        await harness.backend.stop(StopReason.GRIP_RELEASED)

        harness.assert_stopped()
    finally:
        await harness.close()


@pytest.mark.asyncio
@pytest.mark.parametrize("backend_name", ["simulator", "fake_lebai"])
async def test_backend_contract_gripper_and_home_use_common_semantics(
    backend_name: str,
) -> None:
    harness = await make_harness(backend_name)
    phases: list[str] = []
    try:
        await harness.backend.set_gripper(0.7)
        await harness.assert_gripper_observed()
        await harness.backend.home(HOME_OPTIONS, phases.append)

        assert phases[0] == "homing"
        assert phases[-1] == "stabilizing"
        assert set(phases) <= {"homing", "stabilizing"}
    finally:
        await harness.close()
