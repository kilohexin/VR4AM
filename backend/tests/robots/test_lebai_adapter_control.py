from __future__ import annotations

import asyncio
from collections import deque
from unittest.mock import AsyncMock

import pytest

from app.robots.base import (
    BackendCommandError,
    HomeOptions,
    StopReason,
)
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import Pose
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings


HOME_OPTIONS = HomeOptions(
    max_speed_radps=0.1,
    timeout_s=10.0,
    position_tolerance_rad=0.01,
    velocity_tolerance_radps=0.02,
    stable_seconds=0.3,
)


class FakeClock:
    def __init__(self) -> None:
        self.value = 0

    def now_ns(self) -> int:
        return self.value

    def advance_ms(self, value: float) -> None:
        self.value += int(value * 1_000_000)

    async def sleep(self, seconds: float) -> None:
        self.value += int(seconds * 1_000_000_000)
        await asyncio.sleep(0)


def _target(x: float = 0.3) -> Pose:
    return Pose(p=(x, 0.0, 0.4), q=(0.0, 0.0, 0.0, 1.0))


async def _wait_until(predicate, timeout: float = 1.0) -> None:
    async def wait() -> None:
        while not predicate():
            await asyncio.sleep(0)

    await asyncio.wait_for(wait(), timeout)


async def _connected_control_adapter(
    *,
    block_ik: bool = False,
    clock: FakeClock | None = None,
) -> tuple[RealLebaiAdapter, FakeLebaiClient, FakeClock]:
    fake = FakeLebaiClient.idle()
    fake.block_ik = block_ik
    test_clock = clock or FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=fake),
        clock=test_clock.now_ns,
        sleep=test_clock.sleep,
    )
    await adapter.connect()
    return adapter, fake, test_clock


@pytest.mark.asyncio
async def test_control_command_uses_vendor_ik_and_pvat_in_order() -> None:
    adapter, client, _ = await _connected_control_adapter()
    target = _target(0.31)
    client.ik_results = deque(
        [[0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    await adapter.command_tcp(target, command_id=7)
    await client.wait_for_write("move_pvat")

    assert client.ik_calls == [(pose_to_lebai(target), list(IDLE_Q))]
    method, p, v, a, horizon = client.write_calls[-1]
    assert method == "move_pvat"
    assert p[0] == pytest.approx(0.0032)
    assert v[0] == pytest.approx(0.04)
    assert a[0] == pytest.approx(0.5)
    assert horizon == pytest.approx(0.08)
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_invalidates_delayed_ik_before_any_pvat_write() -> None:
    adapter, client, _ = await _connected_control_adapter(block_ik=True)
    client.ik_results = deque(
        [[0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    await adapter.command_tcp(_target(), command_id=9)
    await client.ik_started.wait()

    stop_task = asyncio.create_task(
        adapter.stop(StopReason.GRIP_RELEASED)
    )
    await asyncio.sleep(0)
    client.release_ik.set()
    await stop_task

    assert [call[0] for call in client.write_calls].count("move_pvat") == 0
    assert [call[0] for call in client.write_calls].count("stop_move") == 1
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_command_rejects_stale_cached_robot_state() -> None:
    clock = FakeClock()
    adapter, client, _ = await _connected_control_adapter(clock=clock)
    clock.advance_ms(81)

    with pytest.raises(BackendCommandError, match="^robot_state_stale$"):
        await adapter.command_tcp(_target(), command_id=1)

    assert client.ik_calls == []
    assert client.write_calls == []
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_one_ik_miss_recovers_but_five_consecutive_misses_fault() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.ik_results = deque(
        [
            None,
            [0.0032, -1.0, 1.0, 0.0, 1.57, 0.0],
            None,
            None,
            None,
            None,
            None,
        ]
    )

    await _send_and_wait_for_ik(adapter, client, 1)
    assert adapter.constraint == "ik_boundary"
    assert adapter.pump_fault is None

    await _send_and_wait_for_ik(adapter, client, 2)
    await client.wait_for_write("move_pvat")
    assert adapter.constraint is None

    for command_id in range(3, 8):
        await _send_and_wait_for_ik(adapter, client, command_id)
    await _wait_until(lambda: adapter.pump_fault is not None)

    assert str(adapter.pump_fault) == "ik_failure_persistent"
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_move_requires_three_hundred_ms_stationary_confirmation() -> None:
    adapter, client, clock = await _connected_control_adapter()

    await adapter.stop(StopReason.GRIP_RELEASED)

    assert client.write_calls == [("stop_move",)]
    assert clock.value >= 300_000_000
    assert "stop_sys" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_escalates_once_when_joint_speed_never_settles() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.kin_data["actual_joint_speed"] = [0.1] * 6

    with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
        await adapter.stop(StopReason.STALE)

    assert client.write_calls[0] == ("stop_move",)
    assert [call[0] for call in client.write_calls].count("stop_sys") == 1
    state = await adapter.get_state()
    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_incomplete"
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_home_uses_configured_joint_pose_and_acceleration() -> None:
    adapter, client, _ = await _connected_control_adapter()
    phases: list[str] = []
    options = HOME_OPTIONS

    await adapter.home(options, phases.append)

    movej = next(call for call in client.write_calls if call[0] == "movej")
    assert movej[1] == list(adapter.settings.home_q)
    assert movej[2] == pytest.approx(0.5)
    assert movej[3] == pytest.approx(0.1)
    assert movej[4:] == (0.0, 0.0)
    assert phases == ["homing", "stabilizing"]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_home_rejects_non_idle_without_writing() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.robot_state = "MOVING"

    with pytest.raises(BackendCommandError, match="^home_requires_idle$"):
        await adapter.home(HOME_OPTIONS, lambda phase: None)

    assert client.write_calls == []
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_gripper_force_and_amplitude_are_config_bounded() -> None:
    adapter, client, _ = await _connected_control_adapter()

    await adapter.set_gripper(0.25)
    await adapter.set_gripper(2.0)

    assert client.write_calls[-2:] == [
        ("set_claw", 30, 75),
        ("set_claw", 30, 0),
    ]
    assert "init_claw" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


async def _send_and_wait_for_ik(
    adapter: RealLebaiAdapter,
    client: FakeLebaiClient,
    command_id: int,
) -> None:
    previous_calls = len(client.ik_calls)
    try:
        await adapter.command_tcp(_target(0.3 + command_id * 0.001), command_id)
    except BackendCommandError as error:
        assert str(error) in {
            "ik_unreachable",
            "ik_joint_limit",
            "ik_joint_jump",
            "joint_speed_limit",
        }
    await _wait_until(lambda: len(client.ik_calls) > previous_calls)
