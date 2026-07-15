import asyncio
from collections.abc import Awaitable, Callable
from unittest.mock import AsyncMock

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError, StopReason
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

    async def get_state(self) -> RobotStateMessage:
        return RobotStateMessage(
            server_mono_ns=1,
            mode=TeleopMode.READY,
            robot_state=self.robot_state,
            actual_tcp=Pose(p=(0.3, 0.0, 0.3), q=(0, 0, 0, 1)),
            actual_q=(0, 0, 0, 0, 0, 0),
            gripper=self.gripper,
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


def make_control() -> tuple[RobotControl, LatestVRFrame, FakeBackend, FakeClock]:
    latest = LatestVRFrame()
    backend = FakeBackend()
    clock = FakeClock()
    control = RobotControl(backend=backend, latest=latest, clock=clock, recorder=NoopRecorder())
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
    backend.command_tcp = AsyncMock(side_effect=BackendCommandError("ik_unreachable"))
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
    backend.command_tcp = AsyncMock(side_effect=BackendCommandError("ik_unreachable"))
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
    await control.connect()

    latest.publish(frame(1, False, trigger=0.70), clock.now_ns())
    await control.tick()
    clock.advance_ms(50)
    latest.publish(frame(2, False, trigger=0.90), clock.now_ns())
    await control.tick()
    clock.advance_ms(49)
    latest.publish(frame(3, False, trigger=0.80), clock.now_ns())
    await control.tick()
    clock.advance_ms(1)
    latest.publish(frame(4, False, trigger=0.75), clock.now_ns())
    await control.tick()

    assert backend.gripper_commands == pytest.approx([0.70, 0.75])

    clock.advance_ms(100)
    latest.publish(frame(5, False, trigger=0.77), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.70, 0.75])

    clock.advance_ms(100)
    latest.publish(frame(6, False, trigger=0.771), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == pytest.approx([0.70, 0.75, 0.771])


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
    latest.publish(frame(1, False), clock.now_ns())
    control_tick = control.tick
    tick_count = 0

    async def seventy_ms_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(70)
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
    assert fault_state.fault == "control_loop_error"
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
    backend.command_tcp = AsyncMock(side_effect=BackendCommandError("ik_unreachable"))
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
