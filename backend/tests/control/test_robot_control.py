import asyncio
from collections.abc import Awaitable, Callable
from unittest.mock import AsyncMock

import numpy as np
import pytest

import app.control.robot_control as robot_control_module
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.safety import SafetyLimiter
from app.recording.noop import NoopRecorder
from app.recording.commissioning import RecorderUnavailable
from app.robots.base import BackendCommandError, BackendPreflight, StopReason
from app.schemas.messages import (
    BackendState,
    ControllerState,
    Pose,
    RobotStateMessage,
    TeleopMode,
    VRFrame,
)


class FakeClock:
    def __init__(self) -> None:
        self.value = 1_000_000_000

    def now_ns(self) -> int:
        return self.value

    def advance_ms(self, value: float) -> None:
        self.value += int(value * 1_000_000)


class FakeBackend:
    def __init__(self) -> None:
        self.targets: list[tuple[int, Pose]] = []
        self.stops: list[StopReason] = []
        self.gripper = 0.0
        self.gripper_commands: list[float] = []
        self.connect_count = 0
        self.disconnect_count = 0
        self.robot_state = BackendState.IDLE
        self.home_phases: list[str] = []
        self.actual_tcp = Pose(p=(0.3, 0.0, 0.3), q=(0, 0, 0, 1))
        self.actual_q = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.preflight_result = BackendPreflight(
            ready=True,
            reason=None,
            robot_state=BackendState.IDLE,
            actual_tcp=self.actual_tcp,
            actual_q=self.actual_q,
            tcp_matches=True,
            capabilities=("command_tcp", "home", "gripper"),
        )
        self.backend_label = "LEBAI_FAKE"

    async def connect(self) -> None:
        self.connect_count += 1

    async def disconnect(self) -> None:
        self.disconnect_count += 1

    async def command_tcp(self, target: Pose, command_id: int) -> None:
        self.targets.append((command_id, target))

    async def set_gripper(self, value: float) -> None:
        self.gripper = value
        self.gripper_commands.append(value)

    async def stop(self, reason: StopReason) -> None:
        self.stops.append(reason)

    async def home(self, options, on_phase) -> None:
        on_phase("homing")
        self.home_phases.append("homing")
        on_phase("stabilizing")
        self.home_phases.append("stabilizing")

    async def get_state(self) -> RobotStateMessage:
        return RobotStateMessage(
            server_mono_ns=1,
            mode=TeleopMode.READY,
            robot_state=self.robot_state,
            actual_tcp=self.actual_tcp,
            actual_q=self.actual_q,
            gripper=self.gripper,
            backend=self.backend_label,
        )

    async def preflight(self) -> BackendPreflight:
        return self.preflight_result


class RecordingRecorder(NoopRecorder):
    def __init__(self) -> None:
        self.events: list[dict[str, object]] = []
        self.fail_critical_with: RecorderUnavailable | None = None

    async def write_critical_event(
        self,
        kind: str,
        payload: object,
        server_mono_ns: int,
    ) -> None:
        if self.fail_critical_with is not None:
            raise self.fail_critical_with
        self.events.append(
            {
                "kind": kind,
                "payload": payload,
                "server_mono_ns": server_mono_ns,
            }
        )


def frame(
    seq: int,
    grip: bool,
    p: tuple[float, float, float] = (0.0, 1.2, -0.3),
    *,
    trigger: float = 0.0,
    session_id: str = "test",
    tracking_valid: bool = True,
    visibility: str = "visible",
    client_mono_ms: float | None = None,
) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id=session_id,
        seq=seq,
        client_mono_ms=float(seq) if client_mono_ms is None else client_mono_ms,
        tracking_valid=tracking_valid,
        visibility=visibility,
        right=ControllerState(p=p, q=(0, 0, 0, 1), grip=grip, trigger=trigger),
    )


def make_control(
    *,
    recorder: NoopRecorder | None = None,
    limiter=None,
) -> tuple[RobotControl, LatestVRFrame, FakeBackend, FakeClock]:
    latest = LatestVRFrame()
    backend = FakeBackend()
    clock = FakeClock()
    control = RobotControl(
        backend=backend,
        latest=latest,
        clock=clock,
        recorder=recorder or NoopRecorder(),
        limiter=limiter,
    )
    return control, latest, backend, clock


async def connect_release_arm(
    control: RobotControl,
    latest: LatestVRFrame,
    clock: FakeClock,
) -> None:
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    await control.arm()


@pytest.mark.asyncio
async def test_simulation_scale_accepts_only_stopped_fake_runtime_and_records_change() -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()

    result = await control.set_simulation_scale(1.5, automation_active=False)

    assert result.accepted is True
    assert result.translation_scale == pytest.approx(1.5)
    assert control.mapper.translation_scale == pytest.approx(1.5)
    assert [event for event in recorder.events if event["kind"] == "simulation_scale_changed"] == [
        {
            "kind": "simulation_scale_changed",
            "payload": {"previous": 1.0, "current": 1.5},
            "server_mono_ns": 1_000_000_000,
        }
    ]

    backend.backend_label = "LEBAI"
    rejected = await control.set_simulation_scale(1.6, automation_active=False)
    assert rejected.accepted is False
    assert rejected.reason == "not_simulation"
    assert rejected.translation_scale == pytest.approx(1.5)


@pytest.mark.asyncio
async def test_simulation_scale_accepts_armed_idle_fake_before_grip_anchor() -> None:
    control, latest, _, clock = make_control()
    await connect_release_arm(control, latest, clock)

    result = await control.set_simulation_scale(1.5, automation_active=False)

    assert control.mode is TeleopMode.ARMED
    assert control.mapper.has_anchor is False
    assert result == robot_control_module.SimulationScaleResult(True, 1.5)
    assert control.mapper.translation_scale == pytest.approx(1.5)


@pytest.mark.asyncio
async def test_simulation_scale_accepts_ten_and_rejects_above_ten() -> None:
    control, latest, _, clock = make_control()
    await connect_release_arm(control, latest, clock)

    accepted = await control.set_simulation_scale(10.0, automation_active=False)
    rejected = await control.set_simulation_scale(10.1, automation_active=False)

    assert accepted == robot_control_module.SimulationScaleResult(True, 10.0)
    assert (rejected.accepted, rejected.reason) == (False, "invalid_scale")
    assert control.mapper.translation_scale == pytest.approx(10.0)


@pytest.mark.asyncio
async def test_simulation_scale_rejects_active_anchor_automation_and_invalid_values() -> None:
    control, latest, _, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()

    active = await control.set_simulation_scale(1.6, automation_active=False)
    automated = await control.set_simulation_scale(1.6, automation_active=True)
    invalid = await control.set_simulation_scale(1.55, automation_active=False)

    assert (active.accepted, active.reason) == (False, "not_stopped")
    assert (automated.accepted, automated.reason) == (False, "automation_active")
    assert (invalid.accepted, invalid.reason) == (False, "invalid_scale")
    assert control.mapper.translation_scale == pytest.approx(1.0)


async def activate(
    control: RobotControl,
    latest: LatestVRFrame,
    clock: FakeClock,
) -> None:
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()


async def enter_published_recoverable_fault(
    control: RobotControl,
    fault: str = "ik_unreachable",
) -> tuple[str, int]:
    await control.connect()
    actual_tcp = Pose(p=(0.3, 0.0, 0.3), q=(0, 0, 0, 1))
    hand = Pose(p=(0.0, 1.2, -0.3), q=(0, 0, 0, 1))
    control.mapper.capture(hand, actual_tcp)
    control.filter.reset(actual_tcp)
    control.limiter.set_anchor(actual_tcp.p)
    control.limiter.linear_velocity[:] = (0.1, 0.2, 0.3)
    control.limiter.angular_velocity[:] = (0.4, 0.5, 0.6)
    control.last_target = actual_tcp
    control._last_frame_id = ("fault-session", 7)

    await control._enter_fault(fault)

    assert control.mode is TeleopMode.FAULT
    published = await control.state_message()
    assert published.mode is TeleopMode.FAULT
    assert published.fault == fault
    assert control.mode is TeleopMode.DISARMED
    return control._last_frame_id


def assert_fault_motion_state_is_retained(control: RobotControl) -> None:
    assert control._fault == "ik_unreachable"
    assert control.mapper._hand_anchor is not None
    assert control.filter.value is not None
    assert control.limiter.anchor is not None
    assert np.allclose(control.limiter.linear_velocity, (0.1, 0.2, 0.3))
    assert np.allclose(control.limiter.angular_velocity, (0.4, 0.5, 0.6))
    assert control.last_target is not None


@pytest.mark.asyncio
async def test_manual_home_requires_released_grip_and_ends_disarmed() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    result = await control.home()

    assert result == robot_control_module.HomeResult(True)
    assert backend.stops[-1] is StopReason.HOME
    assert backend.home_phases == ["homing", "stabilizing"]
    assert control.mode is TeleopMode.DISARMED
    assert control.mapper._hand_anchor is None
    assert control.last_target is None


@pytest.mark.asyncio
async def test_manual_home_allows_only_singularity_preflight_exception() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    backend.preflight_result = BackendPreflight(
        ready=False,
        reason="singular_configuration",
        robot_state=BackendState.IDLE,
        actual_tcp=backend.actual_tcp,
        actual_q=backend.actual_q,
        tcp_matches=True,
        capabilities=("command_tcp", "home", "gripper"),
    )

    result = await control.home()

    assert result == robot_control_module.HomeResult(True)
    assert backend.home_phases == ["homing", "stabilizing"]


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "reason",
    [
        "tcp_mismatch",
        "tcp_outside_startup_envelope",
        "joint_outside_soft_limits",
        "robot_disconnected",
        "robot_not_idle",
        "estop:hard_estop",
    ],
)
async def test_manual_home_records_fresh_preflight_and_never_moves_when_it_fails(
    reason: str,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    backend.preflight_result = BackendPreflight(
        ready=False,
        reason=reason,
        robot_state=(
            BackendState.FAULT
            if reason in {"robot_disconnected", "robot_not_idle"}
            else BackendState.IDLE
        ),
        actual_tcp=backend.actual_tcp,
        actual_q=backend.actual_q,
        tcp_matches=reason != "tcp_mismatch",
        capabilities=("command_tcp", "home", "gripper"),
    )

    result = await control.home()

    assert result.accepted is False
    assert result.reason == "home_failed"
    assert backend.home_phases == []
    preflight_events = [
        event for event in recorder.events if event["kind"] == "preflight_result"
    ]
    assert preflight_events[-1]["payload"] == {
        "ready": False,
        "reason": reason,
        "robot_state": backend.preflight_result.robot_state.value,
        "tcp_matches": reason != "tcp_mismatch",
        "capabilities": ["command_tcp", "home", "gripper"],
    }


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("grip", "tracking_valid", "visibility", "age_ms"),
    [
        (False, True, "visible", 100.0),
        (False, False, "visible", 0.0),
        (False, True, "hidden", 0.0),
        (True, True, "visible", 0.0),
    ],
)
async def test_manual_home_rejects_invalid_release_sample_without_backend_calls(
    grip: bool,
    tracking_valid: bool,
    visibility: str,
    age_ms: float,
) -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(
        frame(
            1,
            grip,
            tracking_valid=tracking_valid,
            visibility=visibility,
        ),
        clock.now_ns(),
    )
    clock.advance_ms(age_ms)

    result = await control.home()

    assert result.reason == "grip_pressed"
    assert backend.stops == []
    assert backend.home_phases == []


@pytest.mark.asyncio
async def test_recovery_tick_does_not_stale_stop_delayed_home() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    home_started = asyncio.Event()
    allow_home = asyncio.Event()

    async def delayed_home(options, on_phase) -> None:
        on_phase("homing")
        home_started.set()
        await allow_home.wait()
        on_phase("stabilizing")

    backend.home = delayed_home  # type: ignore[method-assign]
    home = asyncio.create_task(control.home())
    try:
        await home_started.wait()
        clock.advance_ms(150)

        await control.tick()

        assert StopReason.STALE not in backend.stops
        assert control.mode is TeleopMode.READY
    finally:
        allow_home.set()

    assert await home == robot_control_module.HomeResult(True)
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_manual_home_generation_aba_cannot_clear_newer_motion_state() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    actual_tcp = await backend.get_state()
    hand = Pose(p=(0.0, 1.2, -0.3), q=(0, 0, 0, 1))
    control.mapper.capture(hand, actual_tcp.actual_tcp)
    control.last_target = actual_tcp.actual_tcp
    home_started = asyncio.Event()
    allow_home = asyncio.Event()

    async def delayed_home(options, on_phase) -> None:
        on_phase("homing")
        home_started.set()
        await allow_home.wait()

    backend.home = delayed_home  # type: ignore[method-assign]
    home = asyncio.create_task(control.home())
    await home_started.wait()
    control._advance_control_generation()
    control._advance_control_generation()
    allow_home.set()

    result = await home

    assert result.reason == "not_stopped"
    assert control.mapper._hand_anchor is not None
    assert control.last_target == actual_tcp.actual_tcp
    assert backend.stops == [StopReason.HOME, StopReason.HOME]


@pytest.mark.asyncio
async def test_manual_home_and_fault_reset_share_one_recovery_operation() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    first_home_started = asyncio.Event()
    allow_first_home = asyncio.Event()
    home_calls = 0
    active_homes = 0
    max_active_homes = 0

    async def serialized_home(options, on_phase) -> None:
        nonlocal home_calls, active_homes, max_active_homes
        home_calls += 1
        active_homes += 1
        max_active_homes = max(max_active_homes, active_homes)
        try:
            on_phase("homing")
            if home_calls == 1:
                first_home_started.set()
                await allow_first_home.wait()
            on_phase("stabilizing")
        finally:
            active_homes -= 1

    backend.home = serialized_home  # type: ignore[method-assign]
    manual_home = asyncio.create_task(control.home())
    await first_home_started.wait()
    await control._enter_fault("ik_unreachable")
    await control.state_message()
    reset = asyncio.create_task(control.reset_fault())
    await asyncio.sleep(0)

    assert home_calls == 1
    assert not reset.done()

    allow_first_home.set()
    manual_result = await manual_home
    reset_result = await reset

    assert manual_result.reason == "fault_present"
    assert reset_result == robot_control_module.FaultResetResult(True)
    assert max_active_homes == 1


@pytest.mark.asyncio
async def test_manual_home_exception_stops_started_home_motion() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()

    async def failing_home(options, on_phase) -> None:
        on_phase("homing")
        raise RuntimeError("private home failure")

    backend.home = failing_home  # type: ignore[method-assign]

    result = await control.home()

    assert result == robot_control_module.HomeResult(
        False,
        "home_failed",
        "仿真无法返回初始姿态，请稍后重试。",
    )
    assert backend.stops == [StopReason.HOME, StopReason.HOME]
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_manual_home_cancellation_stops_started_home_motion() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    home_started = asyncio.Event()

    async def cancellable_home(options, on_phase) -> None:
        on_phase("homing")
        home_started.set()
        await asyncio.Future()

    backend.home = cancellable_home  # type: ignore[method-assign]
    home = asyncio.create_task(control.home())
    await home_started.wait()
    home.cancel()

    with pytest.raises(asyncio.CancelledError):
        await home

    assert backend.stops == [StopReason.HOME, StopReason.HOME]
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_fault_reset_home_exception_stops_started_home_motion() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)

    async def failing_home(options, on_phase) -> None:
        on_phase("homing")
        raise RuntimeError("private reset home failure")

    backend.home = failing_home  # type: ignore[method-assign]

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(
        False,
        "stop_incomplete",
        "仿真无法返回初始姿态，故障保持锁定。",
    )
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT, StopReason.HOME]
    assert control._fault == "ik_unreachable"
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_manual_home_bounds_state_read_and_rejects_idle_at_deadline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    loop_times = iter((0.0, 0.25, 1.0))
    timeouts: list[float] = []

    class FakeLoop:
        def time(self) -> float:
            return next(loop_times)

    async def recording_wait_for(awaitable, timeout: float):
        timeouts.append(timeout)
        return await awaitable

    monkeypatch.setattr(
        robot_control_module.asyncio,
        "get_running_loop",
        lambda: FakeLoop(),
    )
    monkeypatch.setattr(robot_control_module.asyncio, "wait_for", recording_wait_for)

    result = await control.home()

    assert result == robot_control_module.HomeResult(
        False,
        "home_failed",
        "等待仿真停止超时。",
    )
    assert timeouts == [pytest.approx(0.75)]
    assert backend.home_phases == []


@pytest.mark.asyncio
async def test_reset_recoverable_fault_atomically_clears_motion_state() -> None:
    control, _latest, backend, _clock = make_control()
    last_frame_id = await enter_published_recoverable_fault(control)

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(accepted=True)
    assert control.mode is TeleopMode.DISARMED
    assert control._fault is None
    assert control._pending_stop_completion is False
    assert control._hard_stop_completion is False
    assert control.last_target is None
    assert control.mapper._hand_anchor is None
    assert control.mapper._tcp_anchor is None
    assert control.filter.value is None
    assert control.limiter.anchor is None
    assert np.allclose(control.limiter.linear_velocity, 0)
    assert np.allclose(control.limiter.angular_velocity, 0)
    assert control._last_frame_id == last_frame_id
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT]
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        await control.arm()


@pytest.mark.asyncio
async def test_latched_fault_ignores_released_grip_and_remains_resettable() -> None:
    control, latest, _backend, clock = make_control()
    await enter_published_recoverable_fault(control)
    latest.publish(
        frame(8, False, session_id="fault-session"),
        clock.now_ns(),
    )

    await control.tick()

    assert control.mode is TeleopMode.DISARMED
    assert control._fault == "ik_unreachable"
    assert await control.reset_fault() == robot_control_module.FaultResetResult(True)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("fault", "loop_failed", "shutdown_started", "error"),
    [
        ("workspace_violation", False, False, "arm_blocked_by_fault"),
        ("backend_fault", False, False, "arm_blocked_by_fault"),
        ("control_loop_error", True, False, "control_faulted"),
        (None, False, True, "control_shutdown"),
    ],
)
async def test_arm_explicitly_rejects_latched_fault_and_unavailable_control(
    fault: str | None,
    loop_failed: bool,
    shutdown_started: bool,
    error: str,
) -> None:
    control, _latest, _backend, _clock = make_control()
    control.machine.mode = TeleopMode.READY
    control.machine._grip_released = True
    control._fault = fault
    control._loop_failed = loop_failed
    control._shutdown_started = shutdown_started

    with pytest.raises(RuntimeError, match=error):
        await control.arm()

    assert control.mode is TeleopMode.READY


@pytest.mark.asyncio
async def test_reset_success_requires_a_new_grip_release_frame_before_arm() -> None:
    control, latest, _backend, clock = make_control()
    await enter_published_recoverable_fault(control)
    latest.publish(
        frame(8, False, session_id="fault-session"),
        clock.now_ns(),
    )

    assert await control.reset_fault() == robot_control_module.FaultResetResult(True)
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        await control.arm()

    latest.publish(
        frame(9, False, session_id="fault-session"),
        clock.now_ns(),
    )
    await control.tick()

    assert control.mode is TeleopMode.READY
    await control.arm()
    assert control.mode is TeleopMode.ARMED


@pytest.mark.asyncio
async def test_reset_cuts_off_queued_held_frame_until_later_release() -> None:
    control, latest, _backend, clock = make_control()
    await enter_published_recoverable_fault(control)
    latest.publish(
        frame(8, True, session_id="fault-session"),
        clock.now_ns(),
    )

    assert await control.reset_fault() == robot_control_module.FaultResetResult(True)
    await control.tick()
    assert control.mode is TeleopMode.DISARMED

    latest.publish(
        frame(9, True, session_id="fault-session"),
        clock.now_ns(),
    )
    await control.tick()
    assert control.mode is TeleopMode.DISARMED

    latest.publish(
        frame(10, False, session_id="fault-session"),
        clock.now_ns(),
    )
    await control.tick()
    assert control.mode is TeleopMode.READY


def test_fault_reset_recoverable_whitelist_is_exact() -> None:
    assert robot_control_module.RECOVERABLE_FAULTS == frozenset(
        {
            "workspace_violation",
            "ik_unreachable",
            "ik_singular",
            "joint_safety_window",
            "backend_command_failed",
        }
    )


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("fault", "reason", "message"),
    [
        (None, "no_fault", "当前没有可复位故障。"),
        (
            "control_loop_error",
            "unrecoverable_fault",
            "该故障无法在线复位，请重启后端并重新检查。",
        ),
        (
            "backend_fault",
            "unrecoverable_fault",
            "该故障无法在线复位，请重启后端并重新检查。",
        ),
    ],
)
async def test_reset_fault_rejects_absent_and_unrecoverable_faults_first(
    fault: str | None,
    reason: str,
    message: str,
) -> None:
    control, _latest, backend, _clock = make_control()
    control._fault = fault
    control._loop_failed = True
    control._shutdown_started = True
    control._pending_stop_completion = True
    control.machine.mode = TeleopMode.FAULT
    backend.get_state = AsyncMock(side_effect=AssertionError("must not inspect backend"))

    result = await control.reset_fault()

    assert result.accepted is False
    assert result.reason == reason
    assert result.message == message
    backend.get_state.assert_not_awaited()
    assert backend.stops == []
    assert control._fault == fault


@pytest.mark.asyncio
@pytest.mark.parametrize("unavailable_field", ["_loop_failed", "_shutdown_started"])
async def test_reset_fault_rejects_unavailable_control_loop_before_stop_state(
    unavailable_field: str,
) -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    setattr(control, unavailable_field, True)
    control._pending_stop_completion = True
    backend.get_state = AsyncMock(side_effect=AssertionError("must not inspect backend"))

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(
        False,
        "control_loop_unavailable",
        "控制循环不可用，请重启后端并重新检查。",
    )
    backend.get_state.assert_not_awaited()
    assert backend.stops == [StopReason.FAULT]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("pending", "mode"),
    [(True, TeleopMode.DISARMED), (False, TeleopMode.FAULT)],
)
async def test_reset_fault_requires_completed_stop_and_disarmed_mode(
    pending: bool,
    mode: TeleopMode,
) -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    control._pending_stop_completion = pending
    control.machine.mode = mode
    backend.get_state = AsyncMock(side_effect=AssertionError("must not inspect backend"))

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(
        False,
        "stop_incomplete",
        "停止尚未完成，请稍后重试。",
    )
    backend.get_state.assert_not_awaited()
    assert backend.stops == [StopReason.FAULT]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
async def test_reset_fault_rejects_backend_while_moving_without_second_stop() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    backend.robot_state = BackendState.MOVING

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(
        False,
        "backend_moving",
        "仿真仍在运动，请稍后重试。",
    )
    assert backend.stops == [StopReason.FAULT]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("backend_state", "reason", "message"),
    [
        (
            BackendState.DISCONNECTED,
            "control_loop_unavailable",
            "控制循环不可用，请重启后端并重新检查。",
        ),
        (
            BackendState.FAULT,
            "stop_incomplete",
            "仿真后端仍处于故障状态，故障保持锁定。",
        ),
        (
            BackendState.HOLD,
            "stop_incomplete",
            "仿真尚未完全停止，请稍后重试。",
        ),
    ],
)
async def test_reset_fault_requires_backend_to_be_explicitly_idle(
    backend_state: BackendState,
    reason: str,
    message: str,
) -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    backend.robot_state = backend_state

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(False, reason, message)
    assert backend.stops == [StopReason.FAULT]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
async def test_reset_fault_preserves_new_unrecoverable_fault_during_state_read() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    state_read_started = asyncio.Event()
    allow_state_read = asyncio.Event()
    original_get_state = backend.get_state

    async def blocked_get_state() -> RobotStateMessage:
        state_read_started.set()
        await allow_state_read.wait()
        return await original_get_state()

    backend.get_state = blocked_get_state  # type: ignore[method-assign]
    reset = asyncio.create_task(control.reset_fault())
    await state_read_started.wait()

    await control._enter_fault("control_loop_error")
    allow_state_read.set()
    result = await reset

    assert result == robot_control_module.FaultResetResult(
        False,
        "unrecoverable_fault",
        "该故障无法在线复位，请重启后端并重新检查。",
    )
    assert control._fault == "control_loop_error"
    assert control.mode is TeleopMode.FAULT
    assert control._pending_stop_completion is True
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT]
    assert control.mapper._hand_anchor is not None
    assert control.filter.value is not None
    assert control.limiter.anchor is not None
    assert control.last_target is not None


@pytest.mark.asyncio
async def test_reset_fault_rejects_same_named_new_episode_during_state_read() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    state_read_started = asyncio.Event()
    allow_state_read = asyncio.Event()
    original_get_state = backend.get_state
    get_state_calls = 0

    async def block_first_get_state() -> RobotStateMessage:
        nonlocal get_state_calls
        get_state_calls += 1
        if get_state_calls == 1:
            state_read_started.set()
            await allow_state_read.wait()
        return await original_get_state()

    backend.get_state = block_first_get_state  # type: ignore[method-assign]
    reset = asyncio.create_task(control.reset_fault())
    await state_read_started.wait()

    await control._enter_fault("ik_unreachable")
    new_episode = await control.state_message()
    assert new_episode.mode is TeleopMode.FAULT
    assert control.mode is TeleopMode.DISARMED
    allow_state_read.set()
    result = await reset

    assert result == robot_control_module.FaultResetResult(
        False,
        "stop_incomplete",
        "停止尚未完成，请稍后重试。",
    )
    assert control._fault == "ik_unreachable"
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
async def test_reset_fault_preserves_new_fault_during_second_stop() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    reset_stop_started = asyncio.Event()
    allow_reset_stop = asyncio.Event()
    stop_calls = 0

    async def block_first_stop(reason: StopReason) -> None:
        nonlocal stop_calls
        stop_calls += 1
        backend.stops.append(reason)
        if stop_calls == 1:
            reset_stop_started.set()
            await allow_reset_stop.wait()

    backend.stop = block_first_stop  # type: ignore[method-assign]
    reset = asyncio.create_task(control.reset_fault())
    await reset_stop_started.wait()

    await control._enter_fault("control_loop_error")
    allow_reset_stop.set()
    result = await reset

    assert result == robot_control_module.FaultResetResult(
        False,
        "unrecoverable_fault",
        "该故障无法在线复位，请重启后端并重新检查。",
    )
    assert control._fault == "control_loop_error"
    assert control.mode is TeleopMode.FAULT
    assert control._pending_stop_completion is True
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT, StopReason.FAULT]
    assert control.mapper._hand_anchor is not None
    assert control.filter.value is not None
    assert control.limiter.anchor is not None
    assert control.last_target is not None


@pytest.mark.asyncio
async def test_reset_fault_preserves_fault_when_shutdown_starts_during_second_stop() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    reset_stop_started = asyncio.Event()
    allow_reset_stop = asyncio.Event()
    stop_calls = 0

    async def block_first_stop(reason: StopReason) -> None:
        nonlocal stop_calls
        stop_calls += 1
        backend.stops.append(reason)
        if stop_calls == 1:
            reset_stop_started.set()
            await allow_reset_stop.wait()

    backend.stop = block_first_stop  # type: ignore[method-assign]
    reset = asyncio.create_task(control.reset_fault())
    await reset_stop_started.wait()

    await control.stop()
    allow_reset_stop.set()
    result = await reset

    assert result == robot_control_module.FaultResetResult(
        False,
        "control_loop_unavailable",
        "控制循环不可用，请重启后端并重新检查。",
    )
    assert control._fault == "ik_unreachable"
    assert control._shutdown_started is True
    assert control._shutdown_stopped is True
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT, StopReason.SHUTDOWN]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
async def test_reset_fault_contains_get_state_exception_without_partial_clear() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)
    backend.get_state = AsyncMock(side_effect=RuntimeError("secret state detail"))

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(
        False,
        "control_loop_unavailable",
        "控制循环不可用，请重启后端并重新检查。",
    )
    assert "secret" not in (result.message or "")
    assert backend.stops == [StopReason.FAULT]
    assert_fault_motion_state_is_retained(control)


@pytest.mark.asyncio
async def test_reset_fault_contains_second_stop_exception_without_partial_clear() -> None:
    control, _latest, backend, _clock = make_control()
    await enter_published_recoverable_fault(control)

    async def fail_second_stop(reason: StopReason) -> None:
        backend.stops.append(reason)
        raise RuntimeError("secret stop detail")

    backend.stop = fail_second_stop  # type: ignore[method-assign]

    result = await control.reset_fault()

    assert result == robot_control_module.FaultResetResult(
        False,
        "stop_incomplete",
        "无法确认仿真已停止，故障保持锁定。",
    )
    assert "secret" not in (result.message or "")
    assert backend.stops == [StopReason.FAULT, StopReason.FAULT]
    assert control.mode is TeleopMode.FAULT
    assert control._fault == "stop_unverified"
    assert control.mapper._hand_anchor is None
    assert control.filter.value is None
    assert control.limiter.anchor is None
    assert control.last_target is None


def test_latest_frame_has_capacity_one_and_rejects_same_session_rollback() -> None:
    latest = LatestVRFrame()
    latest.publish(frame(2, False), 20)
    latest.publish(frame(1, False), 30)

    received = latest.snapshot()

    assert received is not None
    assert received.frame.seq == 2
    assert received.received_ns == 20


def test_latest_frame_depth_is_zero_when_empty() -> None:
    latest = LatestVRFrame()

    assert latest.depth == 0


def test_latest_frame_depth_stays_one_for_repeats_bursts_and_new_sessions() -> None:
    latest = LatestVRFrame()
    first = frame(1, False, session_id="old")

    latest.publish(first, 10)
    assert latest.depth == 1
    latest.publish(first, 20)
    assert latest.depth == 1

    for seq in range(2, 20):
        latest.publish(frame(seq, False, session_id="old"), seq * 10)
        assert latest.depth == 1

    latest.publish(frame(1, False, session_id="new"), 200)
    assert latest.depth == 1


def test_latest_frame_accepts_new_session_even_with_lower_sequence() -> None:
    latest = LatestVRFrame()
    latest.publish(frame(100, False, session_id="old"), 20)
    latest.publish(frame(1, False, session_id="new"), 30)

    received = latest.snapshot()

    assert received is not None
    assert received.frame.session_id == "new"
    assert received.frame.seq == 1
    assert received.received_ns == 30


def test_retired_session_cannot_take_back_latest_value() -> None:
    latest = LatestVRFrame()
    latest.publish(frame(100, False, session_id="old"), 20)
    latest.publish(frame(1, False, session_id="new"), 30)

    latest.publish(frame(101, False, session_id="old"), 40)

    received = latest.snapshot()
    assert received is not None
    assert received.frame.session_id == "new"
    assert received.frame.seq == 1
    assert received.received_ns == 30


@pytest.mark.asyncio
async def test_only_latest_frame_is_consumed() -> None:
    control, latest, _backend, clock = make_control()
    latest.publish(frame(seq=1, grip=False), clock.now_ns())
    latest.publish(frame(seq=2, grip=False), clock.now_ns())

    await control.tick()

    assert control.last_seq == 2


@pytest.mark.asyncio
async def test_active_frame_captures_anchor_without_motion_then_commands_target() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)

    latest.publish(frame(seq=2, grip=True), clock.now_ns())
    await control.tick()
    assert backend.targets == []

    latest.publish(frame(seq=3, grip=True, p=(0, 1.2, -0.31)), clock.now_ns())
    await control.tick()

    assert [command_id for command_id, _target in backend.targets] == [3]


@pytest.mark.asyncio
async def test_active_repeated_latest_frame_does_not_repeat_tcp_command() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(seq=2, grip=True), clock.now_ns())
    await control.tick()
    latest.publish(frame(seq=3, grip=True, p=(0, 1.2, -0.31)), clock.now_ns())

    await control.tick()
    await control.tick()
    await control.tick()

    assert [command_id for command_id, _target in backend.targets] == [3]


@pytest.mark.asyncio
async def test_new_session_fails_closed_and_requires_release_rearm_and_fresh_press() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(seq=1, grip=False, session_id="old"), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(seq=2, grip=True, session_id="old"), clock.now_ns())
    await control.tick()
    latest.publish(
        frame(seq=3, grip=True, p=(0, 1.2, -0.31), session_id="old"),
        clock.now_ns(),
    )
    await control.tick()
    recorder = AsyncMock()
    control.recorder = recorder

    new_session_frame = frame(
        seq=3,
        grip=True,
        p=(0, 1.2, -0.32),
        session_id="new",
    )
    latest.publish(new_session_frame, clock.now_ns())
    await control.tick()
    await control.tick()

    assert backend.stops.count(StopReason.DISCONNECT) == 1
    assert control.mode == TeleopMode.DISARMED
    assert [command_id for command_id, _target in backend.targets] == [3]
    assert control.last_target is None
    with pytest.raises(RuntimeError, match="anchor_not_captured"):
        control.mapper.target(Pose(p=(0, 1.2, -0.32), q=(0, 0, 0, 1)))
    recorder.write_vr_frame.assert_awaited_once_with(
        new_session_frame,
        clock.now_ns(),
    )
    assert control.last_seq == 3

    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        await control.arm()
    latest.publish(frame(seq=4, grip=False, session_id="new"), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(
        frame(seq=5, grip=True, p=(0.4, 1.5, -0.6), session_id="new"),
        clock.now_ns(),
    )
    await control.tick()

    assert control.mode == TeleopMode.ACTIVE
    assert [command_id for command_id, _target in backend.targets] == [3]

    latest.publish(
        frame(seq=6, grip=True, p=(0.4, 1.5, -0.61), session_id="new"),
        clock.now_ns(),
    )
    await control.tick()

    assert [command_id for command_id, _target in backend.targets] == [3, 6]


@pytest.mark.asyncio
async def test_reconnect_new_session_does_not_repeat_completed_disconnect_stop() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, session_id="old"), clock.now_ns())
    await control.tick()

    await control.on_disconnect()
    await control.connect()
    latest.publish(frame(1, False, session_id="new"), clock.now_ns())
    await control.tick()

    assert backend.stops.count(StopReason.DISCONNECT) == 1


@pytest.mark.asyncio
async def test_same_session_reconnect_consumes_disconnect_stop_credit() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, session_id="old"), clock.now_ns())
    await control.tick()
    await control.on_disconnect()
    await control.connect()

    latest.publish(frame(2, False, session_id="old"), clock.now_ns())
    await control.tick()
    assert control._disconnect_stop_pending_frame is False

    latest.publish(frame(1, False, session_id="replacement"), clock.now_ns())
    await control.tick()

    assert backend.stops.count(StopReason.DISCONNECT) == 2


@pytest.mark.asyncio
async def test_failed_disconnect_stop_does_not_create_stop_credit() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, session_id="old"), clock.now_ns())
    await control.tick()
    original_stop = backend.stop

    async def fail_disconnect(reason: StopReason) -> None:
        raise RuntimeError(f"failed_{reason}")

    backend.stop = fail_disconnect  # type: ignore[method-assign]
    with pytest.raises(RuntimeError, match="failed_disconnect"):
        await control.on_disconnect()
    assert control._disconnect_stop_pending_frame is False

    backend.stop = original_stop  # type: ignore[method-assign]
    latest.publish(frame(1, False, session_id="new"), clock.now_ns())
    await control.tick()

    assert backend.stops.count(StopReason.DISCONNECT) == 1


@pytest.mark.asyncio
async def test_failed_disconnect_stop_revokes_authority_and_latches_unrecoverable_fault() -> None:
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)

    async def fail_disconnect(reason: StopReason) -> None:
        backend.stops.append(reason)
        raise BackendCommandError("sdk_call_failed:stop_move")

    backend.stop = fail_disconnect  # type: ignore[method-assign]

    with pytest.raises(BackendCommandError, match="^sdk_call_failed:stop_move$"):
        await control.on_disconnect()

    assert control.mode is TeleopMode.FAULT
    assert control._fault == "stop_unverified"
    assert control.mapper._hand_anchor is None
    assert control.last_target is None
    assert control._pending_stop_completion is True
    with pytest.raises(RuntimeError, match="^arm_blocked_by_fault$"):
        await control.arm()


@pytest.mark.asyncio
async def test_verified_disconnect_recovers_only_a_cancelled_overlapping_disarm_stop() -> None:
    control, _latest, backend, _clock = make_control()
    await control.connect()
    grip_stop_started = asyncio.Event()

    async def cancellable_then_verified_stop(reason: StopReason) -> None:
        backend.stops.append(reason)
        if reason is StopReason.GRIP_RELEASED:
            grip_stop_started.set()
            await asyncio.Future()

    backend.stop = cancellable_then_verified_stop  # type: ignore[method-assign]
    disarm = asyncio.create_task(control.disarm())
    await grip_stop_started.wait()
    disarm.cancel()
    with pytest.raises(asyncio.CancelledError):
        await disarm

    assert control.mode is TeleopMode.FAULT
    assert control._fault == "stop_unverified"

    await control.on_disconnect()
    await control.connect()
    state = await control.state_message()

    assert backend.stops == [StopReason.GRIP_RELEASED, StopReason.DISCONNECT]
    assert state.mode is TeleopMode.READY
    assert state.fault is None
    assert control._pending_stop_completion is False


@pytest.mark.asyncio
async def test_verified_disconnect_does_not_clear_a_real_disarm_stop_failure() -> None:
    control, _latest, backend, _clock = make_control()
    await control.connect()

    async def fail_only_grip_stop(reason: StopReason) -> None:
        backend.stops.append(reason)
        if reason is StopReason.GRIP_RELEASED:
            raise BackendCommandError("sdk_call_failed:stop_move")

    backend.stop = fail_only_grip_stop  # type: ignore[method-assign]
    with pytest.raises(BackendCommandError, match="^sdk_call_failed:stop_move$"):
        await control.disarm()

    await control.on_disconnect()
    await control.connect()
    state = await control.state_message()

    assert backend.stops == [StopReason.GRIP_RELEASED, StopReason.DISCONNECT]
    assert state.mode is TeleopMode.FAULT
    assert state.fault == "stop_unverified"
    assert control._pending_stop_completion is True


@pytest.mark.asyncio
async def test_grip_release_holds_and_stops_motion() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()

    latest.publish(frame(3, False), clock.now_ns())
    await control.tick()

    assert control.mode == TeleopMode.HOLD
    assert backend.stops[-1] == StopReason.GRIP_RELEASED


@pytest.mark.asyncio
async def test_stale_input_stops_and_is_observable_before_disarmed() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)

    await control.tick()

    assert backend.stops[-1] == StopReason.STALE
    assert control.mode == TeleopMode.STALE
    stale_state = await control.state_message()
    assert stale_state.mode == TeleopMode.STALE
    assert stale_state.sample_age_ms == pytest.approx(100)
    assert control.mode == TeleopMode.DISARMED
    assert (await control.state_message()).mode == TeleopMode.DISARMED


@pytest.mark.asyncio
@pytest.mark.parametrize("stale_age_ms", [100, 251])
async def test_completed_stale_stop_is_not_repeated_for_unattended_input(
    stale_age_ms: int,
) -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    clock.advance_ms(stale_age_ms)
    await control.tick()
    assert (await control.state_message()).mode == TeleopMode.STALE
    assert control.mode == TeleopMode.DISARMED

    for _ in range(8):
        clock.advance_ms(20)
        await control.tick()
        await control.state_message()

    assert control.mode == TeleopMode.DISARMED
    assert backend.stops == [StopReason.STALE]

    latest.publish(frame(2, False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(3, True), clock.now_ns())
    await control.tick()
    assert control.mode == TeleopMode.ACTIVE

    clock.advance_ms(stale_age_ms)
    await control.tick()
    assert backend.stops == [StopReason.STALE, StopReason.STALE]


@pytest.mark.asyncio
async def test_soft_stale_waits_for_backend_to_stop_after_publishing_stale() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    backend.robot_state = BackendState.MOVING
    latest.publish(frame(1, False), clock.now_ns())
    clock.advance_ms(100)
    await control.tick()

    first_stale = await control.state_message()

    assert first_stale.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.STALE
    backend.robot_state = BackendState.IDLE
    second_stale = await control.state_message()
    assert second_stale.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.DISARMED
    assert (await control.state_message()).mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_received_pc_monotonic_timestamp_is_age_authority() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, client_mono_ms=10_000_000.0), clock.now_ns())
    clock.advance_ms(100)

    await control.tick()

    assert backend.stops[-1] == StopReason.STALE


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("tracking_valid", "visibility"),
    [(False, "visible"), (True, "hidden"), (True, "visible-blurred")],
)
async def test_tracking_or_visibility_loss_immediately_stops(
    tracking_valid: bool,
    visibility: str,
) -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(
        frame(1, False, tracking_valid=tracking_valid, visibility=visibility),
        clock.now_ns(),
    )

    await control.tick()

    assert backend.stops[-1] == StopReason.STALE
    assert control.mode == TeleopMode.STALE


@pytest.mark.asyncio
async def test_hard_stale_publishes_stale_before_stop_complete_disarms() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    backend.robot_state = BackendState.MOVING
    latest.publish(frame(1, False), clock.now_ns())
    clock.advance_ms(251)

    await control.tick()

    assert backend.stops[-1] == StopReason.STALE
    assert control.mode == TeleopMode.STALE
    hard_stale = await control.state_message()
    assert hard_stale.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.DISARMED
    assert (await control.state_message()).mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_hard_stale_preserves_fault_until_fault_is_published() -> None:
    control, latest, backend, clock = make_control()
    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("backend_command_failed")
    )
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, True, p=(0, 1.2, -0.31)), clock.now_ns())
    await control.tick()
    assert control.mode == TeleopMode.FAULT
    backend.robot_state = BackendState.MOVING
    clock.advance_ms(251)

    await control.tick()

    assert control.mode == TeleopMode.FAULT
    hard_fault = await control.state_message()
    assert hard_fault.mode == TeleopMode.FAULT
    assert control.mode == TeleopMode.DISARMED
    assert (await control.state_message()).mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_disarm_cannot_bypass_unpublished_stale_mode() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    clock.advance_ms(100)
    await control.tick()
    assert control.mode == TeleopMode.STALE

    await control.disarm()

    assert control.mode == TeleopMode.STALE
    assert backend.stops == [StopReason.STALE]
    stale_state = await control.state_message()
    assert stale_state.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_disarm_cannot_bypass_unpublished_fault_mode() -> None:
    control, latest, backend, clock = make_control()
    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("backend_command_failed")
    )
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, True, p=(0, 1.2, -0.31)), clock.now_ns())
    await control.tick()
    assert control.mode == TeleopMode.FAULT

    await control.disarm()

    assert control.mode == TeleopMode.FAULT
    assert backend.stops[-1] == StopReason.FAULT
    fault_state = await control.state_message()
    assert fault_state.mode == TeleopMode.FAULT
    assert control.mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_new_session_preserves_unpublished_fault_priority_and_detail() -> None:
    control, latest, backend, clock = make_control()
    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("backend_command_failed")
    )
    await control.connect()
    latest.publish(frame(1, False, session_id="old"), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(2, True, session_id="old"), clock.now_ns())
    await control.tick()
    latest.publish(
        frame(3, True, p=(0, 1.2, -0.31), session_id="old"),
        clock.now_ns(),
    )
    await control.tick()
    assert control.mode == TeleopMode.FAULT
    assert control._pending_stop_completion is True

    latest.publish(frame(1, True, session_id="new"), clock.now_ns())
    await control.tick()

    assert backend.stops[-1] == StopReason.DISCONNECT
    assert control.mode == TeleopMode.FAULT
    assert control._fault == "backend_command_failed"
    assert control._pending_stop_completion is True
    assert control.last_target is None
    fault_state = await control.state_message()
    assert fault_state.mode == TeleopMode.FAULT
    assert fault_state.fault == "backend_command_failed"
    assert control.mode == TeleopMode.DISARMED
    with pytest.raises(RuntimeError, match="arm_blocked_by_fault"):
        await control.arm()

    latest.publish(frame(2, False, session_id="new"), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    assert control._fault == "backend_command_failed"
    with pytest.raises(RuntimeError, match="arm_blocked_by_fault"):
        await control.arm()


@pytest.mark.asyncio
async def test_disconnect_and_reconnect_preserve_published_fault_latch() -> None:
    control, latest, _backend, clock = make_control()
    await enter_published_recoverable_fault(control)

    await control.on_disconnect()
    assert control.mode is TeleopMode.DISCONNECTED
    assert control._fault == "ik_unreachable"
    await control.connect()

    assert control.mode is TeleopMode.DISARMED
    assert control._fault == "ik_unreachable"
    latest.publish(frame(8, False, session_id="fault-session"), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    with pytest.raises(RuntimeError, match="arm_blocked_by_fault"):
        await control.arm()


@pytest.mark.asyncio
async def test_disconnect_preserves_unpublished_fault_until_reconnect_publishes_it() -> None:
    control, _latest, _backend, _clock = make_control()
    await control.connect()
    await control._enter_fault("ik_unreachable")
    assert control.mode is TeleopMode.FAULT
    assert control._pending_stop_completion is True

    await control.on_disconnect()
    assert control.mode is TeleopMode.DISCONNECTED
    assert control._fault == "ik_unreachable"
    assert control._pending_stop_completion is True
    await control.connect()

    assert control.mode is TeleopMode.FAULT
    published = await control.state_message()
    assert published.mode is TeleopMode.FAULT
    assert published.fault == "ik_unreachable"
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_new_session_preserves_published_fault_latch() -> None:
    control, latest, _backend, clock = make_control()
    await enter_published_recoverable_fault(control)
    latest.publish(frame(1, False, session_id="replacement"), clock.now_ns())

    await control.tick()

    assert control.mode is TeleopMode.DISARMED
    assert control._fault == "ik_unreachable"
    with pytest.raises(RuntimeError, match="arm_blocked_by_fault"):
        await control.arm()


@pytest.mark.asyncio
async def test_new_session_preserves_stale_until_backend_stop_completion() -> None:
    control, latest, backend, clock = make_control()
    backend.robot_state = BackendState.MOVING
    await control.connect()
    latest.publish(frame(1, False, session_id="old"), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    await control.tick()
    assert control.mode == TeleopMode.STALE
    assert control._pending_stop_completion is True

    latest.publish(frame(1, True, session_id="new"), clock.now_ns())
    await control.tick()

    assert backend.stops[-1] == StopReason.DISCONNECT
    assert control.mode == TeleopMode.STALE
    assert control._pending_stop_completion is True
    stale_while_moving = await control.state_message()
    assert stale_while_moving.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.STALE
    backend.robot_state = BackendState.IDLE
    stopped = await control.state_message()
    assert stopped.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.DISARMED
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        await control.arm()

    latest.publish(frame(2, False, session_id="new"), clock.now_ns())
    await control.tick()
    await control.arm()
    assert control.mode == TeleopMode.ARMED


@pytest.mark.asyncio
async def test_disconnect_clears_old_hard_stop_episode_before_new_soft_stale() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    backend.robot_state = BackendState.MOVING
    latest.publish(frame(1, False, session_id="old"), clock.now_ns())
    clock.advance_ms(251)
    await control.tick()
    assert control.mode == TeleopMode.STALE

    await control.on_disconnect()
    await control.connect()
    latest.publish(frame(1, False, session_id="new"), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    await control.tick()

    new_stale_state = await control.state_message()
    assert new_stale_state.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.STALE
    backend.robot_state = BackendState.IDLE
    second_stale_state = await control.state_message()
    assert second_stale_state.mode == TeleopMode.STALE
    assert control.mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_reconnect_requires_release_and_explicit_arm() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    assert control.mode == TeleopMode.ARMED
    await control.on_disconnect()

    await control.connect()
    latest.publish(frame(2, True, session_id="reconnected"), clock.now_ns())
    await control.tick()
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        await control.arm()
    assert control.mode == TeleopMode.DISARMED

    latest.publish(frame(3, False, session_id="reconnected"), clock.now_ns())
    await control.tick()
    assert control.mode == TeleopMode.READY
    await control.arm()

    assert control.mode == TeleopMode.ARMED
    assert backend.stops[-1] == StopReason.DISCONNECT


@pytest.mark.asyncio
async def test_gripper_has_strict_10hz_ceiling_delta_threshold_and_no_queue() -> None:
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)

    latest.publish(frame(3, True, trigger=0.70), clock.now_ns())
    await control.tick()
    clock.advance_ms(50)
    latest.publish(frame(4, True, trigger=0.90), clock.now_ns())
    await control.tick()
    clock.advance_ms(49)
    latest.publish(frame(5, True, trigger=0.80), clock.now_ns())
    await control.tick()
    clock.advance_ms(1)
    latest.publish(frame(6, True, trigger=0.75), clock.now_ns())
    await control.tick()

    assert backend.gripper_commands == pytest.approx([0.70, 0.75])

    clock.advance_ms(100)
    latest.publish(frame(7, True, trigger=0.77), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.70, 0.75])

    clock.advance_ms(100)
    latest.publish(frame(8, True, trigger=0.771), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.70, 0.75, 0.771])


@pytest.mark.asyncio
async def test_control_connect_initializes_observed_gripper_without_a_write() -> None:
    control, latest, backend, clock = make_control()
    backend.gripper = 0.35
    await control.connect()

    assert backend.gripper_commands == []

    latest.publish(frame(1, False, trigger=0.35), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(2, True, trigger=0.35), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    latest.publish(frame(3, True, trigger=1.0), clock.now_ns())
    await control.tick()

    assert backend.gripper_commands == pytest.approx([1.0])


@pytest.mark.asyncio
async def test_arm_does_not_open_closed_gripper_until_trigger_changes() -> None:
    control, latest, backend, clock = make_control()
    backend.gripper = 1.0
    await connect_release_arm(control, latest, clock)

    latest.publish(frame(2, False, trigger=0.0), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    latest.publish(frame(3, True, trigger=0.0), clock.now_ns())
    await control.tick()

    assert backend.gripper == pytest.approx(1.0)
    assert backend.gripper_commands == []

    clock.advance_ms(100)
    latest.publish(frame(4, True, trigger=0.5), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.5])

    clock.advance_ms(100)
    latest.publish(frame(5, True, trigger=0.0), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.5, 0.0])


@pytest.mark.asyncio
async def test_arm_ignores_small_trigger_noise_before_deliberate_press() -> None:
    control, latest, backend, clock = make_control()
    backend.gripper = 0.01
    await connect_release_arm(control, latest, clock)

    latest.publish(frame(2, False, trigger=0.0), clock.now_ns())
    await control.tick()
    for seq, trigger in ((3, 0.0388), (4, 0.0), (5, 0.08)):
        clock.advance_ms(100)
        latest.publish(frame(seq, False, trigger=trigger), clock.now_ns())
        await control.tick()

    assert backend.gripper_commands == []
    assert backend.gripper == pytest.approx(0.01)

    clock.advance_ms(100)
    latest.publish(frame(6, False, trigger=0.15), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.15])

    clock.advance_ms(100)
    latest.publish(frame(7, False, trigger=0.0), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.15, 0.0])


@pytest.mark.asyncio
async def test_releasing_trigger_held_at_arm_does_not_activate_gripper() -> None:
    control, latest, backend, clock = make_control()
    backend.gripper = 0.6
    await control.connect()
    latest.publish(frame(1, False, trigger=0.35), clock.now_ns())
    await control.tick()
    await control.arm()

    latest.publish(frame(2, False, trigger=0.35), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    latest.publish(frame(3, False, trigger=0.0), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == []

    clock.advance_ms(100)
    latest.publish(frame(4, False, trigger=0.5), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.5])


@pytest.mark.asyncio
async def test_unarmed_frames_never_write_gripper_even_when_trigger_changes() -> None:
    control, latest, backend, clock = make_control()
    backend.gripper = 0.25
    await control.connect()

    latest.publish(frame(1, False, trigger=0.60), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    latest.publish(frame(2, False, trigger=0.90), clock.now_ns())
    await control.tick()

    assert control.mode is TeleopMode.READY
    assert backend.gripper_commands == []
    assert backend.gripper == pytest.approx(0.25)


@pytest.mark.asyncio
async def test_failed_preflight_session_never_writes_gripper() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, trigger=0.0), clock.now_ns())
    await control.tick()
    backend.preflight_result = BackendPreflight(
        ready=False,
        reason="tcp_mismatch",
        robot_state=BackendState.IDLE,
        actual_tcp=backend.actual_tcp,
        actual_q=backend.actual_q,
        tcp_matches=False,
        capabilities=("command_tcp", "home", "gripper"),
    )

    with pytest.raises(
        RuntimeError,
        match="^arm_blocked_by_preflight:tcp_mismatch$",
    ):
        await control.arm()
    clock.advance_ms(100)
    latest.publish(frame(2, False, trigger=1.0), clock.now_ns())
    await control.tick()

    assert control.mode is TeleopMode.READY
    assert backend.gripper_commands == []


@pytest.mark.asyncio
async def test_current_preflight_and_active_session_allow_one_gripper_write() -> None:
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)
    clock.advance_ms(100)

    latest.publish(frame(3, True, trigger=1.0), clock.now_ns())
    await control.tick()

    assert control.mode is TeleopMode.ACTIVE
    assert backend.gripper_commands == pytest.approx([1.0])


@pytest.mark.asyncio
async def test_explicit_commissioning_gripper_command_requires_active_authority_and_writes_once() -> None:
    unarmed, _latest, unarmed_backend, _clock = make_control()
    await unarmed.connect()

    with pytest.raises(RuntimeError, match="^gripper_command_requires_armed$"):
        await unarmed.command_gripper(0.0)
    assert unarmed_backend.gripper_commands == []

    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True, trigger=0.0), clock.now_ns())
    await control.tick()
    await control.command_gripper(0.0)

    assert backend.gripper_commands == pytest.approx([0.0])


@pytest.mark.asyncio
async def test_initialize_observed_gripper_rejects_a_running_control_loop() -> None:
    control, _latest, _backend, _clock = make_control()
    await control.connect()
    control._running = True

    with pytest.raises(RuntimeError, match="^gripper_initialize_requires_stopped$"):
        await control.initialize_observed_gripper()


@pytest.mark.asyncio
async def test_latched_fault_blocks_new_gripper_commands_after_stop_completion() -> None:
    control, latest, backend, clock = make_control()
    await enter_published_recoverable_fault(control, "workspace_violation")
    assert control.mode is TeleopMode.DISARMED
    assert control._fault == "workspace_violation"

    latest.publish(
        frame(8, False, trigger=0.75, session_id="fault-session"),
        clock.now_ns(),
    )
    await control.tick()

    assert backend.gripper_commands == []
    assert backend.gripper == 0.0


@pytest.mark.asyncio
async def test_failed_control_loop_blocks_new_gripper_commands() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, trigger=0.20), clock.now_ns())
    await control.tick()
    clock.advance_ms(100)
    control._loop_failed = True

    latest.publish(frame(2, False, trigger=0.80), clock.now_ns())
    await control.tick()

    assert backend.gripper_commands == []
    assert backend.gripper == pytest.approx(0.0)


@pytest.mark.asyncio
async def test_shutdown_started_blocks_new_gripper_commands() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, trigger=0.20), clock.now_ns())
    await control.tick()
    await control.stop()
    clock.advance_ms(100)

    latest.publish(frame(2, False, trigger=0.80), clock.now_ns())
    await control.tick()

    assert backend.gripper_commands == []
    assert backend.gripper == pytest.approx(0.0)


@pytest.mark.asyncio
async def test_gripper_is_never_sent_from_an_unqualified_frame() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False, trigger=0.8, tracking_valid=False), clock.now_ns())

    await control.tick()

    assert backend.gripper_commands == []


async def run_with_fake_sleep(
    monkeypatch: pytest.MonkeyPatch,
    control: RobotControl,
    clock: FakeClock,
    tick: Callable[[], Awaitable[None]],
) -> list[float]:
    sleeps: list[float] = []

    async def fake_sleep(delay: float) -> None:
        sleeps.append(delay)
        clock.advance_ms(delay * 1_000)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", fake_sleep)
    control.tick = tick  # type: ignore[method-assign]
    control._running = True
    await control.run()
    return sleeps


@pytest.mark.asyncio
async def test_run_uses_50hz_absolute_deadlines(monkeypatch: pytest.MonkeyPatch) -> None:
    control, _latest, _backend, clock = make_control()
    tick_count = 0

    async def five_ms_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(5)
        if tick_count == 3:
            control._running = False

    sleeps = await run_with_fake_sleep(monkeypatch, control, clock, five_ms_tick)

    assert tick_count == 3
    assert sleeps == pytest.approx([0.015, 0.015, 0.015])


@pytest.mark.asyncio
async def test_raw_execution_over_40ms_does_not_fault_when_deadline_lateness_is_not_over_40ms(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    latest.publish(frame(1, False), clock.now_ns())
    tick_count = 0

    async def fifty_ms_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(50)
        if tick_count == 2:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, fifty_ms_tick)

    assert control.mode != TeleopMode.FAULT
    assert StopReason.FAULT not in backend.stops


@pytest.mark.asyncio
async def test_two_consecutive_absolute_deadline_lateness_over_40ms_fault_and_stop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    control_tick = control.tick
    tick_count = 0

    async def seventy_ms_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(70)
        latest.publish(frame(tick_count, False), clock.now_ns())
        await control_tick()
        if tick_count == 3:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, seventy_ms_tick)

    assert backend.stops[-1] == StopReason.FAULT
    assert control.mode == TeleopMode.FAULT
    fault_state = await control.state_message()
    assert fault_state.mode == TeleopMode.FAULT
    assert control.mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_disconnected_idle_wakeup_delay_does_not_fault_or_repeat_stop(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, _latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    await control.on_disconnect()
    assert control.mode is TeleopMode.DISCONNECTED
    assert backend.stops == [StopReason.DISCONNECT]
    tick_count = 0

    async def idle_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 4:
            control._running = False

    async def delayed_wakeup(_delay: float) -> None:
        clock.advance_ms(70)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", delayed_wakeup)
    control.tick = idle_tick  # type: ignore[method-assign]
    control._running = True
    await control.run()

    assert [event for event in recorder.events if event["kind"] == "robot_fault"] == []
    assert backend.stops == [StopReason.DISCONNECT]
    assert control.mode is TeleopMode.DISCONNECTED
    await control.connect()
    assert control.mode is TeleopMode.READY


@pytest.mark.asyncio
async def test_disarmed_with_stale_frame_ignores_idle_wakeup_delay(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.disarm()
    assert control.mode is TeleopMode.DISARMED
    assert backend.stops == [StopReason.GRIP_RELEASED]
    clock.advance_ms(1_000)
    tick_count = 0

    async def idle_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 4:
            control._running = False

    async def delayed_wakeup(_delay: float) -> None:
        clock.advance_ms(70)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", delayed_wakeup)
    control.tick = idle_tick  # type: ignore[method-assign]
    control._running = True
    await control.run()

    assert [event for event in recorder.events if event["kind"] == "robot_fault"] == []
    assert backend.stops == [StopReason.GRIP_RELEASED]
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_disconnected_with_stale_frame_ignores_idle_wakeup_delay(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.on_disconnect()
    assert control.mode is TeleopMode.DISCONNECTED
    assert backend.stops == [StopReason.DISCONNECT]
    clock.advance_ms(1_000)
    tick_count = 0

    async def idle_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 4:
            control._running = False

    async def delayed_wakeup(_delay: float) -> None:
        clock.advance_ms(70)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", delayed_wakeup)
    control.tick = idle_tick  # type: ignore[method-assign]
    control._running = True
    await control.run()

    assert [event for event in recorder.events if event["kind"] == "robot_fault"] == []
    assert backend.stops == [StopReason.DISCONNECT]
    assert control.mode is TeleopMode.DISCONNECTED


@pytest.mark.asyncio
async def test_ready_without_client_frame_ignores_idle_wakeup_delay(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    assert control.mode is TeleopMode.READY
    assert latest.snapshot() is None
    tick_count = 0

    async def idle_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 4:
            control._running = False

    async def delayed_wakeup(_delay: float) -> None:
        clock.advance_ms(70)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", delayed_wakeup)
    control.tick = idle_tick  # type: ignore[method-assign]
    control._running = True
    await control.run()

    assert [event for event in recorder.events if event["kind"] == "robot_fault"] == []
    assert backend.stops == []
    assert control.mode is TeleopMode.READY


@pytest.mark.asyncio
async def test_armed_without_new_frame_still_faults_on_repeated_overrun(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    await control.arm()
    assert control.mode is TeleopMode.ARMED
    tick_count = 0

    async def idle_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 3:
            control._running = False

    async def delayed_wakeup(_delay: float) -> None:
        clock.advance_ms(70)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", delayed_wakeup)
    control.tick = idle_tick  # type: ignore[method-assign]
    control._running = True
    await control.run()

    assert backend.stops == [StopReason.FAULT]
    assert control.mode is TeleopMode.FAULT


@pytest.mark.asyncio
async def test_overrun_fault_records_idle_deadline_and_previous_tick_timing(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    tick_count = 0

    async def slow_idle_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(70)
        if tick_count == 3:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, slow_idle_tick)

    faults = [event for event in recorder.events if event["kind"] == "robot_fault"]
    assert len(faults) == 1
    assert faults[0]["payload"] == {
        "reason": "control_overrun",
        "mode_before_fault": "READY",
        "deadline_lateness_ns": 100_000_000,
        "previous_tick_duration_ns": 70_000_000,
        "wakeup_lateness_ns": 0,
        "frame_age_ms": 140.0,
    }
    assert backend.stops == [StopReason.FAULT]


@pytest.mark.asyncio
async def test_overrun_fault_distinguishes_scheduler_wakeup_from_tick_duration(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    tick_count = 0

    async def instant_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 3:
            control._running = False

    async def delayed_wakeup(_delay: float) -> None:
        clock.advance_ms(70)

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", delayed_wakeup)
    control.tick = instant_tick  # type: ignore[method-assign]
    control._running = True
    await control.run()

    faults = [event for event in recorder.events if event["kind"] == "robot_fault"]
    assert len(faults) == 1
    assert faults[0]["payload"] == {
        "reason": "control_overrun",
        "mode_before_fault": "READY",
        "deadline_lateness_ns": 100_000_000,
        "previous_tick_duration_ns": 0,
        "wakeup_lateness_ns": 70_000_000,
        "frame_age_ms": 140.0,
    }
    assert backend.stops == [StopReason.FAULT]


@pytest.mark.parametrize(
    "reason",
    [
        StopReason.GRIP_RELEASED,
        StopReason.STALE,
        StopReason.FAULT,
        StopReason.HOME,
    ],
)
@pytest.mark.asyncio
async def test_completed_slow_safety_stop_rebases_the_next_control_deadline(
    monkeypatch: pytest.MonkeyPatch,
    reason: StopReason,
) -> None:
    control, _latest, backend, clock = make_control()
    tick_count = 0

    async def slow_stop(stop_reason: StopReason) -> None:
        backend.stops.append(stop_reason)
        clock.advance_ms(300)

    async def tick_with_one_slow_stop() -> None:
        nonlocal tick_count
        tick_count += 1
        if tick_count == 1:
            await control._stop_backend(reason)
        if tick_count == 3:
            control._running = False

    backend.stop = slow_stop  # type: ignore[method-assign]
    await run_with_fake_sleep(monkeypatch, control, clock, tick_with_one_slow_stop)

    assert backend.stops == [reason]
    assert control._consecutive_overruns == 0


@pytest.mark.asyncio
async def test_slow_hold_resume_does_not_count_one_preflight_as_two_active_overruns(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, False), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.HOLD

    async def slow_resume_preflight() -> BackendPreflight:
        clock.advance_ms(90)
        return backend.preflight_result

    backend.preflight = slow_resume_preflight  # type: ignore[method-assign]
    real_tick = control.tick
    tick_count = 0

    async def resume_then_regular_ticks() -> None:
        nonlocal tick_count
        tick_count += 1
        latest.publish(frame(3 + tick_count, True), clock.now_ns())
        await real_tick()
        if tick_count == 4:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, resume_then_regular_ticks)

    assert control.mode is TeleopMode.ACTIVE
    assert StopReason.FAULT not in backend.stops
    assert not [event for event in recorder.events if event["kind"] == "robot_fault"]


@pytest.mark.asyncio
async def test_successful_slow_gripper_write_while_armed_does_not_fault_on_catch_up_ticks(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)

    async def slow_successful_gripper(value: float) -> None:
        backend.gripper_commands.append(value)
        clock.advance_ms(180)

    backend.set_gripper = slow_successful_gripper  # type: ignore[method-assign]
    real_tick = control.tick
    tick_count = 0

    async def trigger_then_regular_ticks() -> None:
        nonlocal tick_count
        tick_count += 1
        latest.publish(
            frame(tick_count + 1, False, trigger=0.0 if tick_count == 1 else 0.5),
            clock.now_ns(),
        )
        await real_tick()
        if tick_count == 5:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, trigger_then_regular_ticks)

    assert backend.gripper_commands == pytest.approx([0.5])
    assert control.mode is TeleopMode.ARMED
    assert StopReason.FAULT not in backend.stops


@pytest.mark.asyncio
async def test_fast_gripper_write_does_not_hide_slow_active_state_read(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    real_get_state = backend.get_state

    async def slow_state_read() -> RobotStateMessage:
        clock.advance_ms(90)
        return await real_get_state()

    backend.get_state = slow_state_read  # type: ignore[method-assign]
    real_tick = control.tick
    tick_count = 0

    async def trigger_and_grip_then_regular_ticks() -> None:
        nonlocal tick_count
        tick_count += 1
        latest.publish(
            frame(
                tick_count + 1,
                tick_count >= 2,
                trigger=0.0 if tick_count == 1 else 0.5,
            ),
            clock.now_ns(),
        )
        await real_tick()
        if tick_count == 5:
            control._running = False

    await run_with_fake_sleep(
        monkeypatch, control, clock, trigger_and_grip_then_regular_ticks
    )

    assert backend.gripper_commands == pytest.approx([0.5])
    assert control.mode is TeleopMode.FAULT
    assert backend.stops[-1] is StopReason.FAULT


@pytest.mark.asyncio
async def test_failed_gripper_write_while_armed_still_latches_fault() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)

    async def timed_out_gripper(_value: float) -> None:
        clock.advance_ms(200)
        raise BackendCommandError("sdk_timeout:set_claw")

    backend.set_gripper = timed_out_gripper  # type: ignore[method-assign]
    latest.publish(frame(2, False, trigger=0.0), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, False, trigger=0.5), clock.now_ns())
    await control.tick()

    assert control.mode is TeleopMode.FAULT
    assert backend.stops[-1] is StopReason.FAULT
    assert control._fault == "sdk_timeout:set_claw"


@pytest.mark.asyncio
async def test_slow_gripper_write_during_active_motion_does_not_rebase_overrun(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True, trigger=0.0), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE

    async def slow_successful_gripper(value: float) -> None:
        backend.gripper_commands.append(value)
        clock.advance_ms(180)

    backend.set_gripper = slow_successful_gripper  # type: ignore[method-assign]
    real_tick = control.tick
    tick_count = 0

    async def trigger_during_active_motion() -> None:
        nonlocal tick_count
        tick_count += 1
        latest.publish(frame(tick_count + 2, True, trigger=0.5), clock.now_ns())
        await real_tick()
        if tick_count == 4:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, trigger_during_active_motion)

    assert backend.gripper_commands == pytest.approx([0.5])
    assert control.mode is TeleopMode.FAULT
    assert backend.stops[-1] is StopReason.FAULT


@pytest.mark.asyncio
async def test_active_control_still_faults_on_repeated_deadline_lateness(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    real_tick = control.tick
    tick_count = 0

    async def repeated_slow_active_ticks() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(70)
        latest.publish(frame(2 + tick_count, True), clock.now_ns())
        await real_tick()
        if tick_count == 3:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, repeated_slow_active_ticks)

    assert control.mode is TeleopMode.FAULT
    assert backend.stops[-1] is StopReason.FAULT


@pytest.mark.asyncio
async def test_two_long_executions_do_not_pretend_both_started_late(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, _latest, backend, clock = make_control()
    tick_count = 0

    async def seventy_ms_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(70)
        if tick_count == 2:
            control._running = False

    await run_with_fake_sleep(monkeypatch, control, clock, seventy_ms_tick)

    assert control.mode != TeleopMode.FAULT
    assert StopReason.FAULT not in backend.stops


@pytest.mark.asyncio
async def test_unexpected_control_loop_exception_fails_stop_without_masking_or_restart() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    assert control.mode == TeleopMode.ACTIVE

    recorder = NoopRecorder()
    recorder.write_vr_frame = AsyncMock(  # type: ignore[method-assign]
        side_effect=RuntimeError("secret recorder detail")
    )
    control.recorder = recorder
    latest.publish(frame(3, True, p=(0, 1.2, -0.31)), clock.now_ns())

    async def stop_then_fail(reason: StopReason) -> None:
        backend.stops.append(reason)
        raise RuntimeError("secondary stop detail")

    backend.stop = stop_then_fail  # type: ignore[method-assign]
    await control.start()
    failed_task = control._task
    assert failed_task is not None

    with pytest.raises(RuntimeError, match="secret recorder detail"):
        await failed_task

    assert control._running is False
    assert control.mode == TeleopMode.FAULT
    assert backend.stops[-1] == StopReason.FAULT
    assert control.last_target is None
    fault_state = await control.state_message()
    assert fault_state.mode == TeleopMode.FAULT
    assert fault_state.fault == "stop_unverified"
    assert "secret recorder detail" not in (fault_state.fault or "")
    with pytest.raises(RuntimeError, match="control_faulted"):
        await control.start()
    assert control._task is failed_task


@pytest.mark.asyncio
async def test_control_loop_task_cancellation_remains_cancellation_without_fault_stop() -> None:
    control, _latest, backend, _clock = make_control()
    tick_started = asyncio.Event()

    async def blocked_tick() -> None:
        tick_started.set()
        await asyncio.Future()

    control.tick = blocked_tick  # type: ignore[method-assign]
    await control.start()
    task = control._task
    assert task is not None
    await tick_started.wait()

    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task

    assert control._running is False
    assert control.mode != TeleopMode.FAULT
    assert StopReason.FAULT not in backend.stops


@pytest.mark.asyncio
async def test_backend_command_error_faults_and_is_observable() -> None:
    control, latest, backend, clock = make_control()
    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("backend_command_failed")
    )
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, True, p=(0, 1.2, -0.31)), clock.now_ns())

    await control.tick()

    assert backend.stops[-1] == StopReason.FAULT
    assert control.mode == TeleopMode.FAULT
    assert (await control.state_message()).mode == TeleopMode.FAULT
    assert control.mode == TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_self_collision_command_error_is_a_soft_constraint() -> None:
    control, latest, backend, clock = make_control()
    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("self_collision")
    )
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(
        frame(3, True, p=(0.0, 1.2, -0.31)),
        clock.now_ns(),
    )

    await control.tick()

    state = await control.state_message()
    assert control.mode is TeleopMode.ACTIVE
    assert control._fault is None
    assert state.constraint == "self_collision"


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("reason", "expected_constraint"),
    [
        ("ik_joint_jump", "motion_continuity_boundary"),
        ("ik_joint_limit", "joint_boundary"),
        ("joint_speed_limit", "motion_continuity_boundary"),
    ],
)
async def test_pvat_continuity_rejection_is_a_soft_constraint(
    reason: str,
    expected_constraint: str,
) -> None:
    control, latest, backend, clock = make_control()
    backend.command_tcp = AsyncMock(side_effect=BackendCommandError(reason))
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(frame(3, True, p=(0.0, 1.2, -0.31)), clock.now_ns())

    await control.tick()

    state = await control.state_message()
    assert control.mode is TeleopMode.ACTIVE
    assert control._fault is None
    assert state.constraint == expected_constraint


@pytest.mark.asyncio
async def test_tracking_lag_is_public_motion_continuity_boundary() -> None:
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    safe_target = control.last_target

    backend.command_tcp = AsyncMock(
        side_effect=BackendCommandError("ik_tracking_lag")
    )
    latest.publish(
        frame(3, True, p=(0.0, 1.2, -0.31)),
        clock.now_ns(),
    )
    await control.tick()

    state = await control.state_message()
    assert state.constraint == "motion_continuity_boundary"
    assert control.mode is TeleopMode.ACTIVE
    assert control._fault is None
    assert control.last_target == safe_target


@pytest.mark.asyncio
async def test_stop_is_idempotent_and_uses_shutdown_reason() -> None:
    control, _latest, backend, _clock = make_control()

    await control.stop()
    await control.stop()

    assert backend.stops == [StopReason.SHUTDOWN]


@pytest.mark.asyncio
async def test_start_is_permanently_rejected_after_stop() -> None:
    control, _latest, backend, _clock = make_control()
    await control.stop()

    with pytest.raises(RuntimeError, match="control_shutdown"):
        await control.start()

    assert control._task is None
    assert backend.stops == [StopReason.SHUTDOWN]


@pytest.mark.asyncio
async def test_concurrent_stop_waits_for_old_run_and_prevents_restart() -> None:
    control, _latest, backend, _clock = make_control()
    cancellation_seen = asyncio.Event()
    allow_run_exit = asyncio.Event()
    second_stop_started = asyncio.Event()
    restart_started = asyncio.Event()

    async def blocked_run() -> None:
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            cancellation_seen.set()
            await allow_run_exit.wait()

    control.run = blocked_run  # type: ignore[method-assign]
    await control.start()
    old_task = control._task
    assert old_task is not None
    first_stop = asyncio.create_task(control.stop())
    await cancellation_seen.wait()

    async def marked_stop() -> None:
        second_stop_started.set()
        await control.stop()

    async def marked_restart() -> None:
        restart_started.set()
        await control.start()

    second_stop = asyncio.create_task(marked_stop())
    restart = asyncio.create_task(marked_restart())
    await second_stop_started.wait()
    await restart_started.wait()

    assert not second_stop.done()
    assert not restart.done()
    assert control._task is old_task
    allow_run_exit.set()
    await asyncio.gather(first_stop, second_stop)
    with pytest.raises(RuntimeError, match="control_shutdown"):
        await restart
    assert control._task is None
    assert old_task.done()
    assert backend.stops == [StopReason.SHUTDOWN]


@pytest.mark.asyncio
async def test_arm_rejects_failed_backend_preflight_without_motion() -> None:
    control, latest, backend, clock = make_control()
    await control.connect()
    latest.publish(frame(1, grip=False), clock.now_ns())
    await control.tick()
    backend.preflight_result = BackendPreflight(
        ready=False,
        reason="tcp_mismatch",
        robot_state=BackendState.IDLE,
        actual_tcp=backend.actual_tcp,
        actual_q=backend.actual_q,
        tcp_matches=False,
        capabilities=("pvat",),
    )

    with pytest.raises(
        RuntimeError,
        match="^arm_blocked_by_preflight:tcp_mismatch$",
    ):
        await control.arm()

    assert control.mode is TeleopMode.READY
    assert backend.targets == []


@pytest.mark.asyncio
async def test_stop_records_request_before_backend_and_confirmation_after() -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await activate(control, latest, clock)
    latest.publish(frame(10, grip=False), clock.now_ns())

    await control.tick()

    kinds = [str(event["kind"]) for event in recorder.events]
    assert kinds.index("stop_requested") < kinds.index("stop_confirmed")
    assert backend.stops[-1] is StopReason.GRIP_RELEASED


@pytest.mark.asyncio
async def test_failed_grip_stop_records_original_error_without_confirmation() -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await activate(control, latest, clock)
    backend.stop = AsyncMock(side_effect=BackendCommandError("stop_incomplete"))
    latest.publish(frame(10, grip=False), clock.now_ns())
    await control.tick()
    failure = next(e for e in recorder.events if e["kind"] == "stop_failed")
    assert failure["payload"]["reason"] == "grip_released"
    assert failure["payload"]["error"] == "stop_incomplete"
    assert not any(e["kind"] == "stop_confirmed" for e in recorder.events)
    assert control.mode is TeleopMode.FAULT


@pytest.mark.asyncio
async def test_disarm_ignores_a_queued_grip_held_frame_after_revoking_motion() -> None:
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)
    targets_before = list(backend.targets)
    stop_started = asyncio.Event()
    allow_stop = asyncio.Event()

    async def slow_stop(reason: StopReason) -> None:
        backend.stops.append(reason)
        stop_started.set()
        await allow_stop.wait()

    backend.stop = slow_stop  # type: ignore[method-assign]
    latest.publish(frame(3, True, p=(0.01, 1.2, -0.3)), clock.now_ns())
    disarm = asyncio.create_task(control.disarm())
    await stop_started.wait()
    try:
        await control.tick()
    finally:
        allow_stop.set()
        await disarm

    assert backend.targets == targets_before
    assert control.mode is TeleopMode.DISARMED
    assert control._fault is None
    assert control._loop_failed is False
    assert control.mapper._hand_anchor is None
    assert control.last_target is None


@pytest.mark.asyncio
async def test_critical_recorder_failure_never_prevents_physical_stop() -> None:
    recorder = RecordingRecorder()
    control, latest, backend, clock = make_control(recorder=recorder)
    await activate(control, latest, clock)
    recorder.fail_critical_with = RecorderUnavailable("critical_log_queue_full")

    await control.disarm()

    assert backend.stops[-1] is StopReason.GRIP_RELEASED
    assert control.mode is TeleopMode.FAULT
    assert control._fault == "recording_unavailable"


@pytest.mark.asyncio
async def test_disarm_stop_error_revokes_active_authority_in_finally() -> None:
    control, latest, backend, clock = make_control()
    await activate(control, latest, clock)

    async def fail_stop(reason: StopReason) -> None:
        backend.stops.append(reason)
        raise BackendCommandError("sdk_timeout:stop_move")

    backend.stop = fail_stop  # type: ignore[method-assign]

    with pytest.raises(BackendCommandError, match="^sdk_timeout:stop_move$"):
        await control.disarm()

    assert control.mode is TeleopMode.FAULT
    assert control._fault == "stop_unverified"
    assert control.mapper._hand_anchor is None
    assert control.last_target is None
    assert control._pending_stop_completion is True


@pytest.mark.asyncio
async def test_fault_backend_state_never_completes_an_unverified_stop() -> None:
    control, _latest, backend, _clock = make_control()
    await control.connect()
    control.machine.fault()
    control._fault = "stop_unverified"
    control._pending_stop_completion = True
    backend.robot_state = BackendState.FAULT

    state = await control.state_message()

    assert state.mode is TeleopMode.FAULT
    assert state.fault == "stop_unverified"
    assert control.mode is TeleopMode.FAULT
    assert control._pending_stop_completion is True


@pytest.mark.asyncio
async def test_out_of_real_envelope_holds_target_and_retreat_resumes() -> None:
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.005,
        max_rotation_from_anchor_rad=np.deg2rad(30),
    )
    control, latest, backend, clock = make_control(limiter=limiter)
    await activate(control, latest, clock)
    safe_target = control.last_target

    latest.publish(frame(3, True, p=(0.02, 1.2, -0.3)), clock.now_ns())
    await control.tick()

    assert backend.targets == []
    assert control.last_target == safe_target
    assert (await control.state_message()).constraint == "workspace_boundary"
    assert control.mode is TeleopMode.ACTIVE

    latest.publish(frame(4, True, p=(0.001, 1.2, -0.3)), clock.now_ns())
    await control.tick()

    assert [command_id for command_id, _target in backend.targets] == [4]
    assert control.mode is TeleopMode.ACTIVE


@pytest.mark.asyncio
async def test_axis_clamp_keeps_other_axes_moving_and_reverse_clears_constraint() -> None:
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.005,
        max_rotation_from_anchor_rad=np.deg2rad(30),
        workspace_boundary_mode="axis_clamp",
        max_linear_speed=10.0,
        max_linear_accel=100.0,
    )
    control, latest, backend, clock = make_control(limiter=limiter)
    await activate(control, latest, clock)
    hand_anchor = control.mapper._hand_anchor.model_copy(deep=True)
    tcp_anchor = control.mapper._tcp_anchor.model_copy(deep=True)

    latest.publish(frame(3, True, p=(0.02, 1.22, -0.3)), clock.now_ns())
    await control.tick()
    first = backend.targets[-1][1]

    latest.publish(frame(4, True, p=(0.02, 1.195, -0.3)), clock.now_ns())
    await control.tick()
    second = backend.targets[-1][1]

    assert [command_id for command_id, _target in backend.targets] == [3, 4]
    assert first.p[0] <= 0.305 + 1e-12
    assert second.p[0] <= 0.305 + 1e-12
    assert second.p[1] < first.p[1]
    assert (await control.state_message()).constraint == "workspace_boundary"
    assert control.mode is TeleopMode.ACTIVE

    latest.publish(frame(5, True, p=(0.004, 1.2, -0.3)), clock.now_ns())
    await control.tick()
    clock.advance_ms(101)
    latest.publish(frame(6, True, p=(0.004, 1.2, -0.3)), clock.now_ns())
    await control.tick()

    assert (await control.state_message()).constraint is None
    assert control.mapper._hand_anchor == hand_anchor
    assert control.mapper._tcp_anchor == tcp_anchor
