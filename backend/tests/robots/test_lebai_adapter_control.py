from __future__ import annotations

import asyncio
from collections import deque
from typing import Literal
from unittest.mock import AsyncMock

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import (
    BackendCommandError,
    HomeOptions,
    StopReason,
)
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import (
    ControllerState,
    Pose,
    TeleopMode,
    VRFrame,
)
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


def _frame(seq: int, *, grip: bool, x: float = 0.0) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id="runtime-safety",
        seq=seq,
        client_mono_ms=float(seq * 20),
        tracking_valid=True,
        visibility="visible",
        right=ControllerState(
            p=(x, 0.0, 0.0),
            q=(0.0, 0.0, 0.0, 1.0),
            grip=grip,
            trigger=0.0,
        ),
    )


async def _wait_until(predicate, timeout: float = 1.0) -> None:
    async def wait() -> None:
        while not predicate():
            await asyncio.sleep(0)

    await asyncio.wait_for(wait(), timeout)


async def _connected_control_adapter(
    *,
    block_ik: bool = False,
    clock: FakeClock | None = None,
    backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI",
) -> tuple[RealLebaiAdapter, FakeLebaiClient, FakeClock]:
    fake = FakeLebaiClient.idle()
    fake.block_ik = block_ik
    test_clock = clock or FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=fake),
        clock=test_clock.now_ns,
        sleep=test_clock.sleep,
        backend_label=backend_label,
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
@pytest.mark.parametrize(
    ("runtime_state", "estop_reason", "expected_fault"),
    [
        ("ERROR", 0, "robot_state_fault"),
        ("MOVING", "hardestop", "estop:hard_estop"),
    ],
)
async def test_runtime_error_or_estop_closes_production_pump_before_ik_or_pvat(
    runtime_state: object,
    estop_reason: object,
    expected_fault: str,
) -> None:
    adapter, client, clock = await _connected_control_adapter()
    latest = LatestVRFrame()
    control = RobotControl(
        backend=adapter,
        latest=latest,
        clock=clock,
        recorder=NoopRecorder(),
    )
    await control.connect()
    latest.publish(_frame(1, grip=False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(_frame(2, grip=True), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE

    client.robot_state = runtime_state
    client.estop_reason = estop_reason
    latest.publish(_frame(3, grip=True, x=0.01), clock.now_ns())
    await control.tick()
    await _wait_until(lambda: adapter.pump_fault is not None)

    state = await control.state_message()

    assert state.mode is TeleopMode.FAULT
    assert state.fault == expected_fault
    assert client.ik_calls == []
    assert "move_pvat" not in [call[0] for call in client.write_calls]
    assert "stop_move" in [call[0] for call in client.write_calls]

    client.robot_state = "IDLE"
    client.estop_reason = 0
    latest.publish(_frame(4, grip=True, x=0.02), clock.now_ns())
    await control.tick()
    assert "move_pvat" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_adapter_forwards_injected_pump_scheduler() -> None:
    client = FakeLebaiClient.idle()
    clock = FakeClock()
    pump_sleeps: list[float] = []

    async def pump_sleep(seconds: float) -> None:
        pump_sleeps.append(seconds)
        await clock.sleep(seconds)

    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        pump_clock=lambda: clock.value / 1_000_000_000,
        pump_sleep=pump_sleep,
    )
    client.ik_results = deque(
        [
            [0.0032, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.0040, -1.0, 1.0, 0.0, 1.57, 0.0],
        ]
    )
    await adapter.connect()
    try:
        await adapter.command_tcp(_target(0.301), command_id=1)
        await _wait_until(
            lambda: [call[0] for call in client.write_calls].count(
                "move_pvat"
            )
            == 1
        )
        await adapter.command_tcp(_target(0.302), command_id=2)
        await _wait_until(
            lambda: [call[0] for call in client.write_calls].count(
                "move_pvat"
            )
            == 2
        )

        assert pump_sleeps[:2] == pytest.approx([0.04, 0.04])
    finally:
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
async def test_fake_repeated_ik_misses_remain_soft_and_keep_pump_running() -> None:
    adapter, client, _ = await _connected_control_adapter(
        backend_label="LEBAI_FAKE",
    )
    client.ik_results = deque([None] * 10)

    for command_id in range(1, 11):
        await _send_and_wait_for_ik(adapter, client, command_id)

    assert adapter.constraint == "ik_boundary"
    assert adapter.pump_fault is None
    assert adapter._pump.running is True
    await adapter.get_state()
    await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("reason", "expected_constraint"),
    [
        ("ik_unreachable", "ik_boundary"),
        ("ik_invalid", "ik_boundary"),
        ("ik_joint_limit", "joint_boundary"),
        ("ik_joint_jump", "motion_continuity_boundary"),
        ("joint_speed_limit", "motion_continuity_boundary"),
    ],
)
async def test_soft_ik_errors_keep_their_specific_constraint_category(
    reason: str,
    expected_constraint: str,
) -> None:
    adapter, _, _ = await _connected_control_adapter(
        backend_label="LEBAI_FAKE",
    )

    adapter._note_soft_constraint(reason, persistent=False)

    assert adapter.constraint == expected_constraint
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_fake_ik_fallback_sends_nearest_feasible_candidate() -> None:
    events: list[dict[str, object]] = []

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        events.append(event)

    client = FakeLebaiClient.idle()
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=recorder,
        backend_label="LEBAI_FAKE",
    )
    await adapter.connect()
    initial = _target(0.30)
    requested = _target(0.34)
    initial_solution = [0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]
    fallback_solution = [0.0040, -1.0, 1.0, 0.0, 1.57, 0.0]
    client.ik_results = deque(
        [
            initial_solution,
            None,
            fallback_solution,
        ]
    )

    await adapter.command_tcp(initial, command_id=1)
    await client.wait_for_write("move_pvat")
    await adapter.command_tcp(requested, command_id=2)
    await _wait_until(lambda: len(client.ik_calls) == 3)
    await _wait_until(
        lambda: [call[0] for call in client.write_calls].count("move_pvat")
        == 2
    )

    assert client.ik_calls[-2][0] == pytest.approx(pose_to_lebai(requested))
    assert client.ik_calls[-1][0] == pytest.approx(
        pose_to_lebai(_target(0.33))
    )
    pvat_events = [event for event in events if event["kind"] == "pvat_sent"]
    assert pvat_events[-1]["recovery_fraction"] == pytest.approx(0.75)
    assert pvat_events[-1]["requested_tcp"] == requested.model_dump()
    assert pvat_events[-1]["target_tcp"] == _target(0.33).model_dump()
    assert adapter.pump_fault is None
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_clears_fake_fallback_pose_history() -> None:
    adapter, client, _ = await _connected_control_adapter(
        backend_label="LEBAI_FAKE",
    )
    client.ik_results = deque(
        [[0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    await adapter.command_tcp(_target(0.30), command_id=1)
    await client.wait_for_write("move_pvat")

    await adapter.stop(StopReason.GRIP_RELEASED)
    assert adapter._last_sent_tcp is None

    client.ik_results = deque([None])
    previous_calls = len(client.ik_calls)
    await adapter.preflight()
    await adapter.command_tcp(_target(0.34), command_id=2)
    await _wait_until(lambda: len(client.ik_calls) > previous_calls)

    assert len(client.ik_calls) == previous_calls + 1
    assert adapter.pump_fault is None
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_fake_unknown_ik_exception_still_faults_pump() -> None:
    adapter, client, _ = await _connected_control_adapter(
        backend_label="LEBAI_FAKE",
    )
    client.kinematics_inverse = AsyncMock(  # type: ignore[method-assign]
        side_effect=RuntimeError("unexpected_ik_failure")
    )

    await adapter.command_tcp(_target(0.31), command_id=1)
    await _wait_until(lambda: adapter.pump_fault is not None)

    assert str(adapter.pump_fault) == "sdk_call_failed:ik"
    with pytest.raises(BackendCommandError, match="^sdk_call_failed:ik$"):
        await adapter.get_state()
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
async def test_stop_move_failure_escalates_after_the_failed_stop_move() -> None:
    adapter, client, _ = await _connected_control_adapter()

    async def failed_stop_move() -> None:
        client.write_calls.append(("stop_move",))
        raise RuntimeError("simulated_stop_move_failure")

    client.stop_move = failed_stop_move  # type: ignore[method-assign]

    try:
        with pytest.raises(
            BackendCommandError,
            match="^sdk_call_failed:stop_move$",
        ):
            await adapter.stop(StopReason.STALE)

        assert client.write_calls == [("stop_move",), ("stop_sys",)]
        state = await adapter.get_state()
        assert state.robot_state.value == "FAULT"
        assert state.fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_move_cancellation_escalates_and_latches_unverified_stop() -> None:
    adapter, client, _ = await _connected_control_adapter()
    stop_started = asyncio.Event()

    async def blocked_stop_move() -> None:
        client.write_calls.append(("stop_move",))
        stop_started.set()
        await asyncio.Event().wait()

    client.stop_move = blocked_stop_move  # type: ignore[method-assign]
    stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
    await stop_started.wait()
    stop_task.cancel()

    with pytest.raises(asyncio.CancelledError):
        await stop_task

    assert client.write_calls == [("stop_move",), ("stop_sys",)]
    state = await adapter.get_state()
    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_unverified"
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_verified_disconnect_clears_only_a_cancelled_stop_fault() -> None:
    adapter, client, _ = await _connected_control_adapter()
    original_stop_move = client.stop_move
    stop_started = asyncio.Event()

    async def blocked_stop_move() -> None:
        client.write_calls.append(("stop_move",))
        stop_started.set()
        await asyncio.Event().wait()

    client.stop_move = blocked_stop_move  # type: ignore[method-assign]
    stop_task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    await stop_started.wait()
    stop_task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await stop_task

    client.stop_move = original_stop_move  # type: ignore[method-assign]
    await adapter.stop(StopReason.DISCONNECT)
    state = await adapter.get_state()

    assert state.robot_state.value == "IDLE"
    assert state.fault is None
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_verified_disconnect_keeps_a_real_stop_failure_latched() -> None:
    adapter, client, _ = await _connected_control_adapter()
    original_stop_move = client.stop_move

    async def failed_stop_move() -> None:
        client.write_calls.append(("stop_move",))
        raise RuntimeError("simulated_stop_move_failure")

    client.stop_move = failed_stop_move  # type: ignore[method-assign]
    with pytest.raises(BackendCommandError, match="^sdk_call_failed:stop_move$"):
        await adapter.stop(StopReason.GRIP_RELEASED)

    client.stop_move = original_stop_move  # type: ignore[method-assign]
    await adapter.stop(StopReason.DISCONNECT)
    state = await adapter.get_state()

    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_unverified"
    await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("stop_sys_error", [RuntimeError("failed"), TimeoutError()])
async def test_stop_move_failure_retains_primary_error_when_escalation_fails(
    stop_sys_error: Exception,
) -> None:
    adapter, client, _ = await _connected_control_adapter()

    async def failed_stop_move() -> None:
        client.write_calls.append(("stop_move",))
        raise RuntimeError("simulated_stop_move_failure")

    async def failed_stop_sys() -> None:
        client.write_calls.append(("stop_sys",))
        raise stop_sys_error

    client.stop_move = failed_stop_move  # type: ignore[method-assign]
    client.stop_sys = failed_stop_sys  # type: ignore[method-assign]

    try:
        with pytest.raises(
            BackendCommandError,
            match="^sdk_call_failed:stop_move$",
        ):
            await adapter.stop(StopReason.STALE)

        assert client.write_calls == [("stop_move",), ("stop_sys",)]
        state = await adapter.get_state()
        assert state.robot_state.value == "FAULT"
        assert state.fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("verification_failure", ["state", "recorder"])
async def test_stop_verification_failure_escalates_and_latches_unverified_stop(
    verification_failure: str,
) -> None:
    fail_recorder = False

    async def recorder(_event: dict[str, object], _timestamp: int) -> None:
        if fail_recorder:
            raise RuntimeError("recorder_failed")

    client = FakeLebaiClient.idle()
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=recorder,
    )
    await adapter.connect()
    if verification_failure == "state":
        client.get_kin_data = AsyncMock(  # type: ignore[method-assign]
            side_effect=RuntimeError("state_read_failed")
        )
    else:
        fail_recorder = True

    with pytest.raises(BackendCommandError):
        await adapter.stop(StopReason.STALE)

    assert client.write_calls[:2] == [("stop_move",), ("stop_sys",)]
    if verification_failure == "state":
        client.get_kin_data = AsyncMock(  # type: ignore[method-assign]
            return_value=dict(client.kin_data)
        )
    fail_recorder = False
    state = await adapter.get_state()
    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_unverified"
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

    with pytest.raises(
        BackendCommandError,
        match="^preflight_not_ready:robot_not_idle$",
    ):
        await adapter.home(HOME_OPTIONS, lambda phase: None)

    assert client.write_calls == []
    await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "failure",
    [
        "tcp_mismatch",
        "joint_outside_soft_limits",
        "tcp_outside_startup_envelope",
        "robot_disconnected",
        "robot_error",
        "estop",
    ],
)
async def test_home_rechecks_full_preflight_and_never_calls_movej(
    failure: str,
) -> None:
    adapter, client, _ = await _connected_control_adapter()
    if failure == "tcp_mismatch":
        client.tcp["x"] = 0.1
    elif failure == "joint_outside_soft_limits":
        client.kin_data["actual_joint_pose"] = [4.0, *IDLE_Q[1:]]
    elif failure == "tcp_outside_startup_envelope":
        tcp = dict(client.kin_data["actual_tcp_pose"])
        tcp["x"] = 0.9
        client.kin_data["actual_tcp_pose"] = tcp
    elif failure == "robot_disconnected":
        client.connected = False
    elif failure == "robot_error":
        client.robot_state = "ERROR"
    else:
        client.estop_reason = "hardestop"

    with pytest.raises(BackendCommandError):
        await adapter.home(HOME_OPTIONS, lambda phase: None)

    assert "movej" not in [call[0] for call in client.write_calls]
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


@pytest.mark.asyncio
async def test_gripper_rechecks_current_preflight_before_set_claw() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.tcp["x"] = 0.1

    with pytest.raises(BackendCommandError, match="preflight_not_ready:tcp_mismatch"):
        await adapter.set_gripper(0.5)

    assert "set_claw" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_snapshot_timestamp_is_conservative_and_total_read_deadline_is_enforced() -> None:
    adapter, client, clock = await _connected_control_adapter()
    original_get_tcp = client.get_tcp

    async def delayed_get_tcp() -> dict[str, object]:
        clock.advance_ms(50)
        return await original_get_tcp()

    client.get_tcp = delayed_get_tcp  # type: ignore[method-assign]
    started_ns = clock.now_ns()
    state = await adapter.get_state()
    assert state.server_mono_ns == started_ns
    assert clock.now_ns() - state.server_mono_ns == 50_000_000

    async def blocked_get_tcp() -> dict[str, object]:
        await asyncio.Event().wait()
        raise AssertionError("unreachable")

    client.get_tcp = blocked_get_tcp  # type: ignore[method-assign]
    with pytest.raises(BackendCommandError, match="^robot_state_stale$"):
        await adapter.get_state()

    async def expired_get_tcp() -> dict[str, object]:
        clock.advance_ms(90)
        return await original_get_tcp()

    client.get_tcp = expired_get_tcp  # type: ignore[method-assign]
    with pytest.raises(BackendCommandError, match="^robot_state_stale$"):
        await adapter.get_state()
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_snapshot_deadline_starts_after_waiting_for_sdk_lock() -> None:
    adapter, client, clock = await _connected_control_adapter(block_ik=True)
    client.ik_results = deque(
        [[0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    state_task: asyncio.Task | None = None
    try:
        await adapter.command_tcp(_target(0.31), command_id=7)
        await client.ik_started.wait()

        state_task = asyncio.create_task(adapter.get_state())
        await asyncio.sleep(0)
        clock.advance_ms(81)
        client.release_ik.set()

        state = await state_task

        assert state.actual_tcp == _target()
        assert state.server_mono_ns == 81_000_000
    finally:
        client.release_ik.set()
        if state_task is not None and not state_task.done():
            state_task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await state_task
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
