from __future__ import annotations

import asyncio
from dataclasses import replace
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
from app.robots.lebai_ik_policy import evaluate_ik_candidate
from app.schemas.messages import (
    ControllerState,
    JointVector,
    Pose,
    TeleopMode,
    VRFrame,
)
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings, readonly_settings


@pytest.mark.asyncio
@pytest.mark.parametrize("gap_ms", [80, 125, 172])
async def test_expired_pvat_velocity_does_not_push_field_target_past_ik(gap_ms):
    # 20260908T105538Z-5a4675df, command 20: actual state is already
    # decelerated, while the previous commanded J2 velocity is -0.15 rad/s.
    actual = (-0.02233859522358465, -1.5843145324881387, 0.31408256631958503,
              -1.577603366541139, 0.4081347633768234, 0.003163835375014135)
    solution = (-0.023437552134708103, -1.591424341099449, 0.3216762730733289,
                -1.5778736444670027, 0.40918539179789826, 0.003974225602916637)
    actual_speed = (0., -0.019174759848570515, 0.009587379924285258, 0., 0., 0.)
    previous = (-0.013736960075216376, -0.1499921353971312, 0.1404613709191302,
                -0.0033784743139919637, 0.013132854005969141, 0.010129876891947165)
    clock = FakeClock()
    client = FakeLebaiClient.idle(q=list(actual))
    client.kin_data["actual_joint_speed"] = list(actual_speed)
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
                               clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    try:
        adapter._previous_sent_qd = previous
        adapter._last_pvat_started_ns = clock.now_ns()
        clock.advance_ms(gap_ms)
        snapshot = replace(adapter._snapshot, captured_ns=clock.now_ns())
        metrics = evaluate_ik_candidate(solution, actual, previous_solution_q=solution)
        candidate = adapter._build_advancing_candidate(snapshot, _target(), metrics, 1.)
        for q, start, goal in zip(candidate.point.q, actual, solution):
            assert min(start, goal) - 1e-12 <= q <= max(start, goal) + 1e-12
        assert max(abs(a) for a in candidate.point.qdd) <= .5
        for v, measured in zip(candidate.point.qd, actual_speed):
            assert abs(v - measured) <= .5 * .08 + 1e-12
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(("gap_ms", "expected_speed"), [(40, .15), (80, .04)])
async def test_pvat_velocity_history_is_used_only_within_its_horizon(gap_ms, expected_speed):
    adapter, client, clock = await _connected_control_adapter()
    try:
        adapter._previous_sent_qd = (.12, 0., 0., 0., 0., 0.)
        adapter._last_pvat_started_ns = clock.now_ns()
        clock.advance_ms(gap_ms)
        solution = (.024, -1., 1., 0., 1.57, 0.)
        snapshot = replace(adapter._snapshot, captured_ns=clock.now_ns())
        metrics = evaluate_ik_candidate(solution, snapshot.actual_q, previous_solution_q=solution)
        candidate = adapter._build_advancing_candidate(snapshot, _target(), metrics, 1.)
        assert candidate.point.qd[0] == pytest.approx(expected_speed)
    finally:
        await adapter.disconnect()


HOME_OPTIONS = HomeOptions(
    max_speed_radps=0.1,
    timeout_s=10.0,
    position_tolerance_rad=0.01,
    velocity_tolerance_radps=0.02,
    stable_seconds=0.3,
)

FIELD_ROLL_SOLUTIONS: tuple[JointVector, ...] = (
    (
        -0.0030881,
        -1.5400230,
        0.1983114,
        -1.5647952,
        0.4032416,
        0.0189356,
    ),
    (
        -0.0048767,
        -1.5215283,
        0.1622087,
        -1.5568198,
        0.4041157,
        0.0293936,
    ),
    (
        -0.0049698,
        -1.5204396,
        0.1600690,
        -1.5562742,
        0.4041618,
        0.0299422,
    ),
)
FIELD_ROLL_ACTUAL_Q: JointVector = (
    0.0,
    -1.5669614,
    0.2494636,
    -1.5721386,
    0.4016153,
    -0.0010546,
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


async def _wait_for_pvat_count(
    client: FakeLebaiClient,
    expected: int,
) -> None:
    await _wait_until(
        lambda: [call[0] for call in client.write_calls].count("move_pvat")
        == expected
    )


def _last_pvat_event(events: list[dict[str, object]]) -> dict[str, object]:
    return [event for event in events if event.get("kind") == "pvat_sent"][-1]


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
@pytest.mark.parametrize("mode", ["readonly", "control"])
async def test_state_reports_configured_real_robot_mode(mode: str) -> None:
    settings = readonly_settings() if mode == "readonly" else control_settings()
    adapter = RealLebaiAdapter(
        settings,
        client_factory=AsyncMock(return_value=FakeLebaiClient.idle()),
    )
    await adapter.connect()
    try:
        state = await adapter.get_state()
        assert state.backend == "LEBAI"
        assert state.real_robot_mode == mode
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_pvat_timing_separates_snapshot_candidate_and_sdk_costs():
    adapter, client, clock = await _connected_control_adapter()
    events = []

    async def record(event, _timestamp):
        events.append(event)

    original_read = client.get_kin_data
    original_ik = client.kinematics_inverse
    original_send = client.move_pvat

    async def delayed_read():
        clock.advance_ms(7)
        return await original_read()

    async def delayed_ik(pose, joints):
        clock.advance_ms(11)
        return await original_ik(pose, joints)

    async def delayed_send(p, v, a, t):
        clock.advance_ms(5)
        return await original_send(p, v, a, t)

    adapter._event_callback = record
    client.get_kin_data = delayed_read
    client.kinematics_inverse = delayed_ik
    client.move_pvat = delayed_send
    try:
        await adapter.command_tcp(_target(), command_id=1)
        await _wait_until(lambda: any(e.get("kind") == "pvat_sent" for e in events))
        timing = _last_pvat_event(events)["timing_ms"]
        assert timing["snapshot_read"] == pytest.approx(7)
        assert timing["command_lock_wait"] == pytest.approx(0)
        assert timing["candidate_selection"] == pytest.approx(11)
        assert timing["send_sdk"] == pytest.approx(5)
        assert timing["handler_to_send_complete"] == pytest.approx(23)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_snapshot_timing_includes_contended_sdk_lock_wait():
    adapter, client, clock = await _connected_control_adapter()
    await adapter._sdk_lock.acquire()
    task = asyncio.create_task(adapter._read_snapshot())
    try:
        await asyncio.sleep(0)
        clock.advance_ms(9)
        adapter._sdk_lock.release()
        snapshot = await task
        assert snapshot.sdk_lock_wait_ms == pytest.approx(9)
    finally:
        if adapter._sdk_lock.locked():
            adapter._sdk_lock.release()
        await adapter.disconnect()


async def _adapter_with_accepted_history(
    *,
    actual_q: JointVector = tuple(IDLE_Q),  # type: ignore[assignment]
    solution: JointVector,
    target: Pose,
    backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI",
) -> tuple[RealLebaiAdapter, FakeLebaiClient, list[dict[str, object]]]:
    events: list[dict[str, object]] = []

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        events.append(event)

    client = FakeLebaiClient.idle(q=list(actual_q))
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=recorder,
        backend_label=backend_label,
    )
    await adapter.connect()
    adapter._last_accepted_solution_q = solution
    adapter._last_sent_tcp = target.model_copy(deep=True)
    adapter._accepted_target_command_id = 1
    return adapter, client, events


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
async def test_second_ik_uses_previous_accepted_solution_as_seed() -> None:
    adapter, client, _ = await _connected_control_adapter()
    first_solution = [0.04, -1.0, 1.0, 0.0, 1.57, 0.0]
    second_solution = [0.08, -1.0, 1.0, 0.0, 1.57, 0.0]
    client.ik_results = deque([first_solution, second_solution])

    await adapter.command_tcp(_target(0.301), command_id=1)
    await _wait_for_pvat_count(client, 1)
    await adapter.command_tcp(_target(0.302), command_id=2)
    await _wait_for_pvat_count(client, 2)

    assert client.ik_calls[1][1] == first_solution
    assert adapter._last_accepted_solution_q == tuple(second_solution)
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_pvat_event_distinguishes_full_solution_from_physical_point() -> None:
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
    )
    await adapter.connect()
    solution = [0.04, -1.0, 1.0, 0.0, 1.57, 0.0]
    client.ik_results = deque([solution])

    await adapter.command_tcp(_target(0.301), command_id=7)
    await _wait_for_pvat_count(client, 1)
    await _wait_until(
        lambda: any(event.get("kind") == "pvat_sent" for event in events)
    )

    event = _last_pvat_event(events)
    assert event["ik_solution_q"] == solution
    assert event["p"][0] == pytest.approx(0.0032)  # type: ignore[index]
    assert event["solution_step_rad"] == pytest.approx(0.04)
    assert event["tracking_error_rad"] == pytest.approx(0.04)
    assert event["pvat_mode"] == "advance"
    assert event["accepted_target_command_id"] == 7
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_failed_pvat_write_does_not_commit_solution_history() -> None:
    adapter, client, _ = await _connected_control_adapter()
    solution = [0.04, -1.0, 1.0, 0.0, 1.57, 0.0]
    client.ik_results = deque([solution])
    client.move_pvat = AsyncMock(  # type: ignore[method-assign]
        side_effect=RuntimeError("write_failed")
    )

    await adapter.command_tcp(_target(0.301), command_id=8)
    await _wait_until(lambda: adapter.pump_fault is not None)

    assert adapter._last_accepted_solution_q is None
    assert adapter._last_sent_tcp is None
    assert adapter._accepted_target_command_id is None
    await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("operation", ["stop", "home", "disconnect"])
async def test_lifecycle_operation_clears_accepted_solution_history(
    operation: str,
) -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.ik_results = deque(
        [[0.04, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    await adapter.command_tcp(_target(0.301), command_id=9)
    await _wait_for_pvat_count(client, 1)

    if operation == "stop":
        await adapter.stop(StopReason.GRIP_RELEASED)
    elif operation == "home":
        await adapter.home(HOME_OPTIONS, lambda _phase: None)
    else:
        await adapter.disconnect()

    assert adapter._last_accepted_solution_q is None
    assert adapter._last_sent_tcp is None
    assert adapter._previous_sent_qd is None
    assert adapter._accepted_target_command_id is None
    if operation != "disconnect":
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_runtime_fault_clears_accepted_solution_history() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.ik_results = deque(
        [[0.04, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    await adapter.command_tcp(_target(0.301), command_id=10)
    await _wait_for_pvat_count(client, 1)

    client.robot_state = "ERROR"
    await adapter.command_tcp(_target(0.302), command_id=11)
    await _wait_until(lambda: adapter.pump_fault is not None)

    assert adapter._last_accepted_solution_q is None
    assert adapter._last_sent_tcp is None
    assert adapter._previous_sent_qd is None
    assert adapter._accepted_target_command_id is None
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_real_target_uses_proportional_cartesian_recovery() -> None:
    adapter, client, events = await _adapter_with_accepted_history(
        solution=(0.04, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [
            [0.14, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.08, -1.0, 1.0, 0.0, 1.57, 0.0],
        ]
    )

    try:
        await adapter.command_tcp(_target(0.311), command_id=2)
        await _wait_until(
            lambda: len(client.ik_calls) >= 2 or adapter.constraint is not None
        )
        assert len(client.ik_calls) == 2
        await _wait_for_pvat_count(client, 1)
        await _wait_until(
            lambda: any(
                event.get("kind") == "pvat_sent" for event in events
            )
        )

        event = _last_pvat_event(events)
        assert event["pvat_mode"] == "interpolated_advance"
        assert event["recovery_fraction"] == pytest.approx(0.4)
        assert event["solution_step_rad"] == pytest.approx(0.04)
        assert client.ik_calls[1][0] == pytest.approx(
            pose_to_lebai(_target(0.305))
        )
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_real_target_halves_fraction_for_second_recovery_solve() -> None:
    adapter, client, events = await _adapter_with_accepted_history(
        solution=(0.04, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [
            [0.14, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.10, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.08, -1.0, 1.0, 0.0, 1.57, 0.0],
        ]
    )

    try:
        await adapter.command_tcp(_target(0.311), command_id=2)
        await _wait_until(
            lambda: len(client.ik_calls) >= 3 or adapter.constraint is not None
        )
        assert len(client.ik_calls) == 3
        await _wait_for_pvat_count(client, 1)
        await _wait_until(
            lambda: any(
                event.get("kind") == "pvat_sent" for event in events
            )
        )

        event = _last_pvat_event(events)
        assert event["recovery_fraction"] == pytest.approx(0.2)
        assert client.ik_calls[2][0] == pytest.approx(
            pose_to_lebai(_target(0.303))
        )
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_real_discontinuous_target_stops_after_three_ik_attempts() -> None:
    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.04, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [
            [0.14, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.11, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.10, -1.0, 1.0, 0.0, 1.57, 0.0],
        ]
    )

    try:
        await adapter.command_tcp(_target(0.311), command_id=2)
        await _wait_until(
            lambda: len(client.ik_calls) >= 3 or adapter.constraint is not None
        )

        assert len(client.ik_calls) == 3
        assert adapter.constraint == "motion_continuity_boundary"
        assert adapter.pump_fault is None
        assert "move_pvat" not in [
            call[0] for call in client.write_calls
        ]
        assert adapter._last_accepted_solution_q[0] == pytest.approx(0.04)
        assert adapter._last_sent_tcp == _target(0.301)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_tracking_lag_reuses_last_solution_without_advancing_history() -> None:
    adapter, client, events = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    try:
        await adapter.command_tcp(_target(0.302), command_id=2)
        await _wait_for_pvat_count(client, 1)
        await _wait_until(
            lambda: any(
                event.get("kind") == "pvat_sent" for event in events
            )
        )

        assert adapter._last_accepted_solution_q[0] == pytest.approx(0.22)
        assert adapter._last_sent_tcp == _target(0.301)
        assert adapter._accepted_target_command_id == 1
        assert adapter.constraint == "motion_continuity_boundary"
        assert adapter.pump_fault is None
        assert _last_pvat_event(events)["pvat_mode"] == "catch_up"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_repeated_tracking_lag_never_becomes_persistent_ik_failure() -> None:
    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]] * 7
    )

    try:
        for command_id in range(2, 8):
            if command_id == 2:
                await adapter.command_tcp(_target(0.302), command_id)
            else:
                with pytest.raises(
                    BackendCommandError,
                    match="^ik_tracking_lag$",
                ):
                    await adapter.command_tcp(_target(0.302), command_id)
            await _wait_for_pvat_count(client, command_id - 1)

        assert adapter.pump_fault is None
        assert adapter._consecutive_ik_failures == 0
        assert adapter._last_accepted_solution_q[0] == pytest.approx(0.22)

        client.kin_data["actual_joint_pose"] = [
            0.04,
            -1.0,
            1.0,
            0.0,
            1.57,
            0.0,
        ]
        client.ik_results.append(
            [0.26, -1.0, 1.0, 0.0, 1.57, 0.0]
        )
        with pytest.raises(
            BackendCommandError,
            match="^ik_tracking_lag$",
        ):
            await adapter.command_tcp(_target(0.302), command_id=8)
        await _wait_for_pvat_count(client, 7)
        await _wait_until(lambda: adapter.constraint is None)

        assert adapter._last_accepted_solution_q[0] == pytest.approx(0.26)
        assert adapter._accepted_target_command_id == 8
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_accepted_solution_outside_tracking_envelope_is_hard_fault() -> None:
    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.26, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.ik_results = deque(
        [[0.27, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    try:
        await adapter.command_tcp(_target(0.302), command_id=2)
        await _wait_until(lambda: adapter.pump_fault is not None)

        assert str(adapter.pump_fault) == "ik_tracking_diverged"
        assert "move_pvat" not in [
            call[0] for call in client.write_calls
        ]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_tracking_backpressure_diagnostic_is_best_effort() -> None:
    events: list[dict[str, object]] = []

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        events.append(event)
        if event.get("kind") == "ik_tracking_backpressure":
            raise RuntimeError("diagnostic_sink_failed")

    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    adapter._event_callback = recorder
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    try:
        await adapter.command_tcp(_target(0.302), command_id=2)
        await _wait_for_pvat_count(client, 1)
        await _wait_until(
            lambda: any(
                event.get("kind") == "ik_tracking_backpressure"
                for event in events
            )
        )

        assert adapter.pump_fault is None
        assert adapter.constraint == "motion_continuity_boundary"
        assert adapter._last_accepted_solution_q[0] == pytest.approx(0.22)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_tracking_diagnostic_does_not_hold_sdk_lock_from_stop() -> None:
    diagnostic_started = asyncio.Event()
    release_diagnostic = asyncio.Event()

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        if event.get("kind") == "ik_tracking_backpressure":
            diagnostic_started.set()
            await release_diagnostic.wait()

    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    adapter._event_callback = recorder
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    stop_task: asyncio.Task[None] | None = None

    try:
        await adapter.command_tcp(_target(0.302), command_id=2)
        await asyncio.wait_for(diagnostic_started.wait(), timeout=0.5)

        stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
        await client.wait_for_write("stop_move", timeout=0.5)
    finally:
        release_diagnostic.set()
        if stop_task is not None:
            await stop_task
        assert adapter._last_accepted_solution_q is None
        assert adapter._last_sent_tcp is None
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_during_catch_up_ik_cannot_restore_history_or_write_pvat() -> None:
    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    client.block_ik = True
    client.ik_results = deque(
        [[0.26, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    stop_task: asyncio.Task[None] | None = None

    try:
        await adapter.command_tcp(_target(0.302), command_id=2)
        await client.ik_started.wait()

        stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
        await asyncio.sleep(0)
        client.release_ik.set()
        await stop_task

        assert "move_pvat" not in [
            call[0] for call in client.write_calls
        ]
        assert adapter._last_accepted_solution_q is None
        assert adapter._last_sent_tcp is None
        assert adapter._motion_accepted is False
        assert adapter.pump_fault is None
    finally:
        client.release_ik.set()
        if stop_task is not None and not stop_task.done():
            await stop_task
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_during_pvat_write_prevents_late_history_commit() -> None:
    adapter, client, _ = await _adapter_with_accepted_history(
        solution=(0.22, -1.0, 1.0, 0.0, 1.57, 0.0),
        target=_target(0.301),
    )
    pvat_started = asyncio.Event()
    release_pvat = asyncio.Event()

    async def blocked_move_pvat(
        p: list[float],
        v: list[float],
        a: list[float],
        t: float,
    ) -> object:
        client.write_calls.append(("move_pvat", p, v, a, t))
        pvat_started.set()
        await release_pvat.wait()
        return 1

    client.move_pvat = blocked_move_pvat  # type: ignore[method-assign]
    client.ik_results = deque(
        [[0.24, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )
    stop_task: asyncio.Task[None] | None = None

    try:
        await adapter.command_tcp(_target(0.302), command_id=2)
        await asyncio.wait_for(pvat_started.wait(), timeout=0.5)

        stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
        await asyncio.sleep(0)
        release_pvat.set()
        await stop_task

        assert [call[0] for call in client.write_calls].count("move_pvat") == 1
        assert [call[0] for call in client.write_calls].count("stop_move") == 1
        assert adapter._last_accepted_solution_q is None
        assert adapter._last_sent_tcp is None
        assert adapter._motion_accepted is False
        assert adapter.pump_fault is None
    finally:
        release_pvat.set()
        if stop_task is not None and not stop_task.done():
            await stop_task
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_field_roll_sequence_separates_continuity_from_tracking_lag() -> None:
    first_metrics = evaluate_ik_candidate(
        FIELD_ROLL_SOLUTIONS[1],
        actual_q=FIELD_ROLL_ACTUAL_Q,
        previous_solution_q=FIELD_ROLL_SOLUTIONS[0],
    )
    assert first_metrics.solution_step_rad == pytest.approx(0.0361, abs=1e-4)
    assert first_metrics.tracking_error_rad == pytest.approx(0.0873, abs=1e-4)

    adapter, client, events = await _adapter_with_accepted_history(
        actual_q=FIELD_ROLL_ACTUAL_Q,
        solution=FIELD_ROLL_SOLUTIONS[0],
        target=_target(0.301),
    )
    client.ik_results = deque(FIELD_ROLL_SOLUTIONS[1:])

    try:
        await adapter.command_tcp(_target(0.302), command_id=12)
        await _wait_for_pvat_count(client, 1)
        await adapter.command_tcp(_target(0.303), command_id=14)
        await _wait_for_pvat_count(client, 2)
        await _wait_until(
            lambda: len(
                [
                    event
                    for event in events
                    if event.get("kind") == "pvat_sent"
                ]
            )
            == 2
        )

        pvat_events = [
            event for event in events if event.get("kind") == "pvat_sent"
        ]
        assert all(
            float(event["tracking_error_rad"]) > 0.05
            for event in pvat_events
        )
        assert all(event["pvat_mode"] == "advance" for event in pvat_events)
        assert not any(
            event.get("kind") == "ik_candidate_rejected"
            for event in events
        )
        assert client.ik_calls[0][1] == pytest.approx(
            FIELD_ROLL_SOLUTIONS[0]
        )
        assert client.ik_calls[1][1] == pytest.approx(
            FIELD_ROLL_SOLUTIONS[1]
        )
        assert all(
            max(abs(float(value)) for value in event["v"]) <= 0.15
            for event in pvat_events
        )
        assert all(
            max(abs(float(value)) for value in event["a"]) <= 0.5
            for event in pvat_events
        )
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_rejected_ik_candidate_records_runtime_joint_delta_without_pvat() -> None:
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
    )
    await adapter.connect()
    target = Pose(
        p=(0.301, 0.002, 0.4),
        q=(0.0, 0.0, 0.0087265355, 0.9999619231),
    )
    rejected_solution = [0.20, -1.02, 1.01, 0.03, 1.55, -0.01]
    client.ik_results = deque([rejected_solution])

    await adapter.command_tcp(target, command_id=17)
    await _wait_until(
        lambda: any(
            event.get("kind") == "ik_candidate_rejected"
            for event in events
        )
    )

    rejected = next(
        event
        for event in events
        if event.get("kind") == "ik_candidate_rejected"
    )
    assert rejected == {
        "kind": "ik_candidate_rejected",
        "reason": "ik_joint_jump",
        "command_id": 17,
        "target_translation_delta_m": pytest.approx(
            0.00223606797749979
        ),
        "target_rotation_delta_deg": pytest.approx(1.0),
        "actual_q": list(IDLE_Q),
        "solution_q": rejected_solution,
        "delta_q": pytest.approx([0.20, -0.02, 0.01, 0.03, -0.02, -0.01]),
        "max_abs_delta_q": pytest.approx(0.20),
        "max_joint_step_rad": pytest.approx(0.05),
        "prior_pvat_sent": False,
        "previous_command_id": None,
    }
    assert "move_pvat" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_first_frame_solution_over_continuity_limit_is_rejected() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.ik_results = deque(
        [[0.051, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    await adapter.command_tcp(_target(0.301), command_id=18)
    await _wait_until(
        lambda: adapter.constraint == "motion_continuity_boundary"
    )

    assert "move_pvat" not in [call[0] for call in client.write_calls]
    assert adapter.pump_fault is None
    await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("solution", "actual_speed", "expected_reason"),
    [
        ([3.0, -1.0, 1.0, 0.0, 1.57, 0.0], [0.0] * 6, "ik_joint_limit"),
        (list(IDLE_Q), [0.16, 0.0, 0.0, 0.0, 0.0, 0.0], "joint_speed_limit"),
    ],
)
async def test_other_rejected_ik_candidate_reasons_are_diagnosed(
    solution: list[float],
    actual_speed: list[float],
    expected_reason: str,
) -> None:
    events: list[dict[str, object]] = []

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        events.append(event)

    client = FakeLebaiClient.idle()
    client.kin_data["actual_joint_speed"] = actual_speed
    client.ik_results = deque([solution])
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=recorder,
    )
    await adapter.connect()

    await adapter.command_tcp(_target(0.301), command_id=22)
    await _wait_until(
        lambda: any(
            event.get("kind") == "ik_candidate_rejected"
            for event in events
        )
    )

    rejected = next(
        event
        for event in events
        if event.get("kind") == "ik_candidate_rejected"
    )
    assert rejected["reason"] == expected_reason
    assert "move_pvat" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_accepted_ik_candidate_does_not_record_rejection() -> None:
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
    )
    await adapter.connect()
    client.ik_results = deque(
        [[0.0032, -1.0, 1.0, 0.0, 1.57, 0.0]]
    )

    await adapter.command_tcp(_target(0.301), command_id=18)
    await client.wait_for_write("move_pvat")

    assert not any(
        event.get("kind") == "ik_candidate_rejected"
        for event in events
    )
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_rejected_ik_candidate_identifies_previous_successful_pvat() -> None:
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
    )
    await adapter.connect()
    client.ik_results = deque(
        [
            [0.0032, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.20, -1.02, 1.01, 0.03, 1.55, -0.01],
        ]
    )

    await adapter.command_tcp(_target(0.301), command_id=18)
    await client.wait_for_write("move_pvat")
    await adapter.command_tcp(_target(0.302), command_id=19)
    await _wait_until(
        lambda: any(
            event.get("kind") == "ik_candidate_rejected"
            for event in events
        )
    )

    rejected = next(
        event
        for event in events
        if event.get("kind") == "ik_candidate_rejected"
    )
    assert rejected["prior_pvat_sent"] is True
    assert rejected["previous_command_id"] == 18
    assert [call[0] for call in client.write_calls].count("move_pvat") == 1
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_rejected_ik_diagnostic_failure_preserves_soft_constraint() -> None:
    async def broken_recorder(
        event: dict[str, object],
        _timestamp: int,
    ) -> None:
        if event.get("kind") == "ik_candidate_rejected":
            raise RuntimeError("diagnostic_sink_failed")

    client = FakeLebaiClient.idle()
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=broken_recorder,
    )
    await adapter.connect()
    client.ik_results = deque(
        [[0.20, -1.02, 1.01, 0.03, 1.55, -0.01]]
    )

    await adapter.command_tcp(_target(0.301), command_id=20)
    await _wait_until(lambda: len(client.ik_calls) == 1)
    await asyncio.sleep(0)

    assert adapter.constraint == "motion_continuity_boundary"
    assert adapter.pump_fault is None
    assert "move_pvat" not in [call[0] for call in client.write_calls]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_rejected_ik_diagnostic_does_not_hold_sdk_lock_from_stop() -> None:
    diagnostic_started = asyncio.Event()
    release_diagnostic = asyncio.Event()

    async def blocking_recorder(
        event: dict[str, object],
        _timestamp: int,
    ) -> None:
        if event.get("kind") == "ik_candidate_rejected":
            diagnostic_started.set()
            await release_diagnostic.wait()

    client = FakeLebaiClient.idle()
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=blocking_recorder,
    )
    await adapter.connect()
    client.ik_results = deque(
        [[0.20, -1.02, 1.01, 0.03, 1.55, -0.01]]
    )
    stop_task: asyncio.Task[None] | None = None
    try:
        await adapter.command_tcp(_target(0.301), command_id=21)
        await asyncio.wait_for(diagnostic_started.wait(), timeout=0.5)

        stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
        await client.wait_for_write("stop_move", timeout=0.5)
    finally:
        release_diagnostic.set()
        if stop_task is not None:
            await stop_task
        assert adapter._motion_accepted is False
        assert adapter._last_sent_tcp is None
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_fake_fallback_diagnostic_cannot_restore_state_after_stop() -> None:
    diagnostic_started = asyncio.Event()
    release_diagnostic = asyncio.Event()
    diagnostic_finished = asyncio.Event()

    async def blocking_recorder(
        event: dict[str, object],
        _timestamp: int,
    ) -> None:
        if event.get("kind") == "ik_candidate_rejected":
            diagnostic_started.set()
            await release_diagnostic.wait()
            diagnostic_finished.set()

    client = FakeLebaiClient.idle()
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=blocking_recorder,
        backend_label="LEBAI_FAKE",
    )
    await adapter.connect()
    client.ik_results = deque(
        [
            [0.0032, -1.0, 1.0, 0.0, 1.57, 0.0],
            [0.20, -1.02, 1.01, 0.03, 1.55, -0.01],
            [0.0040, -1.0, 1.0, 0.0, 1.57, 0.0],
        ]
    )
    stop_task: asyncio.Task[None] | None = None
    try:
        await adapter.command_tcp(_target(0.300), command_id=30)
        await client.wait_for_write("move_pvat")
        await adapter.command_tcp(_target(0.304), command_id=31)
        await asyncio.wait_for(diagnostic_started.wait(), timeout=0.5)

        stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
        await _wait_until(
            lambda: [call[0] for call in client.write_calls].count(
                "stop_move"
            )
            == 1
        )
    finally:
        release_diagnostic.set()
        if stop_task is not None:
            await stop_task
        await asyncio.wait_for(diagnostic_finished.wait(), timeout=0.5)
        await asyncio.sleep(0)
        assert adapter._motion_accepted is False
        assert adapter._last_sent_tcp is None
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_invalidates_persistent_ik_fault_waiting_on_diagnostic() -> None:
    diagnostic_count = 0
    fifth_diagnostic_started = asyncio.Event()
    release_fifth_diagnostic = asyncio.Event()

    async def recorder(event: dict[str, object], _timestamp: int) -> None:
        nonlocal diagnostic_count
        if event.get("kind") != "ik_candidate_rejected":
            return
        diagnostic_count += 1
        if diagnostic_count == 5:
            fifth_diagnostic_started.set()
            await release_fifth_diagnostic.wait()

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
    client.ik_results = deque(
        [[0.20, -1.02, 1.01, 0.03, 1.55, -0.01]] * 5
    )
    stop_task: asyncio.Task[None] | None = None
    try:
        for command_id in range(1, 5):
            await _send_and_wait_for_ik(adapter, client, command_id)
            await _wait_until(lambda: diagnostic_count == command_id)
        with pytest.raises(BackendCommandError, match="^ik_joint_jump$"):
            await adapter.command_tcp(_target(0.305), command_id=5)
        await asyncio.wait_for(fifth_diagnostic_started.wait(), timeout=0.5)

        stop_task = asyncio.create_task(adapter.stop(StopReason.STALE))
        await client.wait_for_write("stop_move", timeout=0.5)
    finally:
        release_fifth_diagnostic.set()
        if stop_task is not None:
            await stop_task
        await asyncio.sleep(0)
        assert adapter.pump_fault is None
        assert adapter._motion_accepted is False
        assert adapter._last_sent_tcp is None
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
    stop_events = []

    async def record_stop(event, _timestamp):
        if event.get("kind") == "stop_diagnostics":
            stop_events.append(event)

    adapter._event_callback = record_stop
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
    # A fault/estop with zero speed is not evidence that the controller
    # completed stopping. Preserve the originating fault in the audit record.
    assert state.fault == "stop_incomplete"
    assert any(event["initial_fault"] == expected_fault for event in stop_events)
    assert client.ik_calls == []
    assert "move_pvat" not in [call[0] for call in client.write_calls]
    assert "stop_move" in [call[0] for call in client.write_calls]
    assert "stop_sys" not in [call[0] for call in client.write_calls]

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
    clock.advance_ms(341)

    with pytest.raises(BackendCommandError, match="^robot_state_stale$"):
        await adapter.command_tcp(_target(), command_id=1)

    assert client.ik_calls == []
    assert client.write_calls == []
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_command_accepts_cached_state_within_aligned_budget() -> None:
    clock = FakeClock()
    adapter, client, _ = await _connected_control_adapter(clock=clock)
    clock.advance_ms(340)

    await adapter.command_tcp(_target(), command_id=1)
    await client.wait_for_write("move_pvat")

    assert adapter.pump_fault is None
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
async def test_stop_latches_incomplete_without_system_stop_when_speed_never_settles() -> None:
    adapter, client, _ = await _connected_control_adapter()
    client.kin_data["actual_joint_speed"] = [0.1] * 6

    with pytest.raises(BackendCommandError, match="^stop_incomplete$"):
        await adapter.stop(StopReason.STALE)

    assert client.write_calls == [("stop_move",)]
    state = await adapter.get_state()
    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_incomplete"
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_move_failure_latches_without_system_stop() -> None:
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

        assert client.write_calls == [("stop_move",)]
        state = await adapter.get_state()
        assert state.robot_state.value == "FAULT"
        assert state.fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_move_cancellation_latches_without_system_stop() -> None:
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

    assert client.write_calls == [("stop_move",)]
    state = await adapter.get_state()
    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_unverified"
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_disconnect_does_not_clear_cancelled_stop_or_resend_pending_request() -> None:
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
    with pytest.raises(BackendCommandError, match="sdk_timeout:stop_move"):
        await adapter.stop(StopReason.DISCONNECT)
    state = await adapter.get_state()

    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_unverified"
    assert client.write_calls == [("stop_move",)]
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
    with pytest.raises(BackendCommandError, match="sdk_call_failed:stop_move"):
        await adapter.stop(StopReason.DISCONNECT)
    state = await adapter.get_state()

    assert state.robot_state.value == "FAULT"
    assert state.fault == "stop_unverified"
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_move_failure_retains_primary_error_without_system_stop() -> None:
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

        assert client.write_calls == [("stop_move",)]
        state = await adapter.get_state()
        assert state.robot_state.value == "FAULT"
        assert state.fault == "stop_unverified"
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize("verification_failure", ["state", "recorder"])
async def test_stop_state_read_failure_latches_but_recorder_failure_is_isolated(
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

    try:
        if verification_failure == "state":
            with pytest.raises(BackendCommandError):
                await adapter.stop(StopReason.STALE)
            assert client.write_calls == [("stop_move",)]
            client.get_kin_data = AsyncMock(  # type: ignore[method-assign]
                return_value=dict(client.kin_data)
            )
        else:
            # Losing diagnostics must not turn a physically verified stop into
            # an automatic system shutdown. Ordinary recording remains guarded.
            await adapter.stop(StopReason.STALE)
            assert client.write_calls == [("stop_move",)]
            assert clock.value >= 300_000_000
        fail_recorder = False
        state = await adapter.get_state()
        if verification_failure == "state":
            assert state.robot_state.value == "FAULT"
            assert state.fault == "stop_unverified"
        else:
            assert state.fault is None
    finally:
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
@pytest.mark.parametrize(
    "q",
    [
        [0.0, -1.0, 0.0, 0.0, 0.2, 0.0],
        [0.0, -1.0, 0.08726646259971647, 0.0, 0.2, 0.0],
        [0.0, -1.0, 0.2, 0.0, 0.08726646259971647, 0.0],
    ],
)
async def test_control_preflight_rejects_cartesian_singularity(
    q: list[float],
) -> None:
    client = FakeLebaiClient.idle(q=q)
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
    )
    await adapter.connect()

    preflight = await adapter.preflight()

    assert preflight.ready is False
    assert preflight.reason == "singular_configuration"
    assert client.write_calls == []
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_prepare_moves_from_singular_home_to_configured_ready_pose() -> None:
    client = FakeLebaiClient.idle(q=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
    )
    original_movej = client.movej

    async def converging_movej(
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object:
        result = await original_movej(p, a, v, t, r)
        client.kin_data["actual_joint_pose"] = list(p)
        client.kin_data["target_joint_pose"] = list(p)
        return result

    client.movej = converging_movej  # type: ignore[method-assign]
    await adapter.connect()
    phases: list[str] = []

    await adapter.prepare(HOME_OPTIONS, phases.append)

    movej = next(call for call in client.write_calls if call[0] == "movej")
    assert movej[1] == list(adapter.settings.teleop_ready_q)
    assert phases == ["homing", "stabilizing"]
    assert (await adapter.preflight()).ready is True
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_prepare_timeout_stops_ambiguously_accepted_movej() -> None:
    client = FakeLebaiClient.idle(q=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    clock = FakeClock()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
    )

    async def accepted_but_no_reply(
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object:
        client.write_calls.append(("movej", list(p), a, v, t, r))
        await asyncio.Event().wait()
        raise AssertionError("unreachable")

    client.movej = accepted_but_no_reply  # type: ignore[method-assign]
    await adapter.connect()

    with pytest.raises(BackendCommandError, match="^sdk_timeout:movej$"):
        await adapter.prepare(HOME_OPTIONS, lambda _phase: None)

    assert [call[0] for call in client.write_calls] == ["movej", "stop_move"]
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_home_remains_available_from_singular_pose() -> None:
    client = FakeLebaiClient.idle(q=[0.0, -1.0, 0.0, 0.0, 0.0, 0.0])
    clock = FakeClock()
    settings = control_settings(
        home_q=(0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    )
    adapter = RealLebaiAdapter(
        settings,
        client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns,
        sleep=clock.sleep,
    )
    await adapter.connect()

    await adapter.home(HOME_OPTIONS, lambda _phase: None)

    assert any(call[0] == "movej" for call in client.write_calls)
    preflight = await adapter.preflight()
    assert preflight.reason == "singular_configuration"
    with pytest.raises(BackendCommandError, match="^preflight_not_ready$"):
        await adapter.command_tcp(_target(), command_id=1)
    assert "move_pvat" not in [call[0] for call in client.write_calls]
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

    async def slow_get_tcp() -> dict[str, object]:
        clock.advance_ms(90)
        return await original_get_tcp()

    client.get_tcp = slow_get_tcp  # type: ignore[method-assign]
    state = await adapter.get_state()
    assert clock.now_ns() - state.server_mono_ns == 90_000_000

    async def acquisition_budget_get_tcp() -> dict[str, object]:
        clock.advance_ms(250)
        return await original_get_tcp()

    client.get_tcp = acquisition_budget_get_tcp  # type: ignore[method-assign]
    state = await adapter.get_state()
    assert clock.now_ns() - state.server_mono_ns == 250_000_000

    async def expired_get_tcp() -> dict[str, object]:
        clock.advance_ms(301)
        return await original_get_tcp()

    client.get_tcp = expired_get_tcp  # type: ignore[method-assign]
    with pytest.raises(BackendCommandError, match="^robot_state_stale$"):
        await adapter.get_state()
    await adapter.disconnect()


@pytest.mark.asyncio
async def test_pvat_accepts_state_after_eighty_one_ms_ik_jitter() -> None:
    adapter, client, clock = await _connected_control_adapter()

    async def slow_ik(
        pose: dict[str, float],
        joints: list[float],
    ) -> object:
        client.ik_calls.append((dict(pose), list(joints)))
        clock.advance_ms(81)
        return list(joints)

    client.kinematics_inverse = slow_ik  # type: ignore[method-assign]

    try:
        await adapter.command_tcp(_target(0.31), command_id=8)
        await _wait_until(
            lambda: adapter.pump_fault is not None
            or any(call[0] == "move_pvat" for call in client.write_calls)
        )

        assert adapter.pump_fault is None
        assert len(client.ik_calls) == 1
        assert "move_pvat" in [call[0] for call in client.write_calls]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_pvat_rejects_successful_ik_after_aligned_budget_expires() -> None:
    adapter, client, clock = await _connected_control_adapter()

    async def expired_ik(
        pose: dict[str, float],
        joints: list[float],
    ) -> object:
        client.ik_calls.append((dict(pose), list(joints)))
        clock.advance_ms(341)
        return list(joints)

    client.kinematics_inverse = expired_ik  # type: ignore[method-assign]

    try:
        await adapter.command_tcp(_target(0.31), command_id=9)
        await _wait_until(lambda: adapter.pump_fault is not None)

        assert str(adapter.pump_fault) == "robot_state_stale"
        assert len(client.ik_calls) == 1
        assert "move_pvat" not in [call[0] for call in client.write_calls]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_recovery_does_not_start_another_ik_after_state_goes_stale() -> None:
    adapter, client, clock = await _connected_control_adapter(
        backend_label="LEBAI_FAKE",
    )
    await adapter.command_tcp(_target(0.30), command_id=8)
    await client.wait_for_write("move_pvat")
    client.write_calls.clear()
    client.ik_calls.clear()

    async def stale_failed_ik(
        pose: dict[str, float],
        joints: list[float],
    ) -> object:
        client.ik_calls.append((dict(pose), list(joints)))
        clock.advance_ms(341)
        return None

    client.kinematics_inverse = stale_failed_ik  # type: ignore[method-assign]

    try:
        await adapter.command_tcp(_target(0.31), command_id=9)
        await _wait_until(
            lambda: adapter.pump_fault is not None
            or len(client.ik_calls) > 1
        )

        assert str(adapter.pump_fault) == "robot_state_stale"
        assert len(client.ik_calls) == 1
        assert "move_pvat" not in [call[0] for call in client.write_calls]
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_pvat_accepts_full_read_and_ik_within_aligned_command_budget() -> None:
    adapter, client, clock = await _connected_control_adapter()
    original_get_tcp = client.get_tcp

    async def slow_get_tcp() -> dict[str, object]:
        clock.advance_ms(250)
        return await original_get_tcp()

    async def jittered_ik(
        pose: dict[str, float],
        joints: list[float],
    ) -> object:
        client.ik_calls.append((dict(pose), list(joints)))
        clock.advance_ms(80)
        return list(joints)

    client.get_tcp = slow_get_tcp  # type: ignore[method-assign]
    client.kinematics_inverse = jittered_ik  # type: ignore[method-assign]
    client.read_calls.clear()

    await adapter.command_tcp(_target(0.31), command_id=7)
    await client.wait_for_write("move_pvat")

    assert adapter.pump_fault is None
    assert "get_running_motion" in client.read_calls
    assert len(client.ik_calls) == 1
    assert "move_pvat" in [call[0] for call in client.write_calls]
    client.get_tcp = original_get_tcp  # type: ignore[method-assign]
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
