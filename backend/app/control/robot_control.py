from __future__ import annotations

import asyncio
import contextlib
import math
from dataclasses import dataclass

from app.control.coordinate_mapper import CoordinateMapper
from app.control.filters import PoseFilter
from app.control.safety import SafetyLimiter, SafetyViolation
from app.control.state_machine import TeleopStateMachine
from app.recording.base import RecorderSink
from app.robots.base import BackendCommandError, RobotBackend, StopReason
from app.schemas.messages import BackendState, Pose, RobotStateMessage, TeleopMode, VRFrame
from app.timebase import MonotonicClock

CONTROL_PERIOD_NS = 20_000_000
OVERRUN_NS = 40_000_000
GRIPPER_PERIOD_NS = 100_000_000
GRIPPER_MIN_DELTA = 0.02


@dataclass(frozen=True)
class ReceivedFrame:
    frame: VRFrame
    received_ns: int


class LatestVRFrame:
    def __init__(self) -> None:
        self._value: ReceivedFrame | None = None
        self._retired_sessions: set[str] = set()

    def publish(self, frame: VRFrame, received_ns: int) -> None:
        if self._value is None:
            self._value = ReceivedFrame(frame, received_ns)
            return
        current_session = self._value.frame.session_id
        if frame.session_id == current_session:
            if frame.seq > self._value.frame.seq:
                self._value = ReceivedFrame(frame, received_ns)
            return
        if frame.session_id not in self._retired_sessions:
            self._retired_sessions.add(current_session)
            self._value = ReceivedFrame(frame, received_ns)

    def snapshot(self) -> ReceivedFrame | None:
        return self._value

    @property
    def depth(self) -> int:
        """Return the number of frames retained by the newest-only store."""
        return int(self._value is not None)


class RobotControl:
    def __init__(
        self,
        *,
        backend: RobotBackend,
        latest: LatestVRFrame,
        clock: MonotonicClock,
        recorder: RecorderSink,
    ) -> None:
        self.backend = backend
        self.latest = latest
        self.clock = clock
        self.recorder = recorder
        self.mapper = CoordinateMapper()
        self.limiter = SafetyLimiter()
        self.machine = TeleopStateMachine()
        self.filter = PoseFilter()
        self.last_seq: int | None = None
        self._last_frame_id: tuple[str, int] | None = None
        self.last_target: Pose | None = None
        self._last_sample_age_ms: float | None = None
        self._last_gripper_sent: float | None = None
        self._last_gripper_sent_ns: int | None = None
        self._consecutive_overruns = 0
        self._pending_stop_completion = False
        self._hard_stop_completion = False
        self._fault: str | None = None
        self._running = False
        self._loop_failed = False
        self._task: asyncio.Task[None] | None = None
        self._lifecycle_lock = asyncio.Lock()
        self._shutdown_started = False
        self._shutdown_stopped = False

    @property
    def mode(self) -> TeleopMode:
        return self.machine.mode

    async def connect(self) -> None:
        self.machine.connect()
        self._clear_stop_episode()

    async def start(self) -> None:
        async with self._lifecycle_lock:
            if self._shutdown_started:
                raise RuntimeError("control_shutdown")
            if self._loop_failed:
                raise RuntimeError("control_faulted")
            if self._running:
                return
            if self.machine.mode == TeleopMode.DISCONNECTED:
                self.machine.connect()
            self._running = True
            self._task = asyncio.create_task(self.run())

    async def arm(self) -> None:
        self.machine.arm()

    async def disarm(self) -> None:
        if self.machine.mode in {TeleopMode.STALE, TeleopMode.FAULT}:
            return
        if self.machine.mode == TeleopMode.ACTIVE:
            self.mapper.clear()
        await self.backend.stop(StopReason.GRIP_RELEASED)
        self.machine.disarm()

    async def on_disconnect(self) -> None:
        await self.backend.stop(StopReason.DISCONNECT)
        self.mapper.clear()
        self._clear_stop_episode()
        self.machine.disconnect()

    async def stop(self) -> None:
        async with self._lifecycle_lock:
            if self._shutdown_stopped:
                return
            self._shutdown_started = True
            self._running = False
            task = self._task
            if task is not None and task is not asyncio.current_task():
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await task
            self._task = None
            await self.backend.stop(StopReason.SHUTDOWN)
            self._shutdown_stopped = True

    async def run(self) -> None:
        next_deadline_ns = self.clock.now_ns()
        try:
            while self._running:
                now_ns = self.clock.now_ns()
                lateness_ns = now_ns - next_deadline_ns
                if lateness_ns > OVERRUN_NS:
                    self._consecutive_overruns += 1
                else:
                    self._consecutive_overruns = 0
                if self._consecutive_overruns >= 2:
                    await self._enter_fault("control_overrun")
                await self.tick()
                next_deadline_ns += CONTROL_PERIOD_NS
                delay_seconds = max(
                    0.0,
                    (next_deadline_ns - self.clock.now_ns()) / 1_000_000_000,
                )
                await asyncio.sleep(delay_seconds)
        except asyncio.CancelledError:
            self._running = False
            raise
        except Exception:
            self._running = False
            self._loop_failed = True
            self.mapper.clear()
            self.last_target = None
            self.machine.fault()
            self._fault = "control_loop_error"
            self._pending_stop_completion = True
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self.backend.stop(StopReason.FAULT)
            raise

    async def tick(self) -> None:
        received = self.latest.snapshot()
        if received is None:
            return
        now_ns = self.clock.now_ns()
        age_ms = max(0.0, (now_ns - received.received_ns) / 1_000_000)
        self._last_sample_age_ms = age_ms
        if age_ms >= 250:
            await self._safe_stop(StopReason.STALE)
            self._hard_stop_completion = True
            return
        if (
            age_ms >= 100
            or not received.frame.tracking_valid
            or received.frame.visibility != "visible"
        ):
            await self._safe_stop(StopReason.STALE)
            return

        try:
            frame_id = (received.frame.session_id, received.frame.seq)
            if (
                self._last_frame_id is not None
                and received.frame.session_id != self._last_frame_id[0]
            ):
                await self.backend.stop(StopReason.DISCONNECT)
                self.mapper.clear()
                self.last_target = None
                if self.machine.mode not in {TeleopMode.FAULT, TeleopMode.STALE}:
                    self._clear_stop_episode()
                    self.machine.disarm()
            is_new_frame = frame_id != self._last_frame_id
            if is_new_frame:
                await self.recorder.write_vr_frame(received.frame, received.received_ns)
            await self._send_latest_gripper(received.frame.right.trigger, now_ns)
            previous_mode = self.machine.mode
            self.machine.observe_grip(received.frame.right.grip)
            if (
                previous_mode in {TeleopMode.ARMED, TeleopMode.HOLD}
                and self.machine.mode == TeleopMode.ACTIVE
            ):
                state = await self.backend.get_state()
                self.mapper.capture(
                    Pose(p=received.frame.right.p, q=received.frame.right.q),
                    state.actual_tcp,
                )
                self.limiter.set_anchor(state.actual_tcp.p)
                self.filter.reset(state.actual_tcp)
                self.last_target = state.actual_tcp
            elif previous_mode == TeleopMode.ACTIVE and self.machine.mode == TeleopMode.HOLD:
                self.mapper.clear()
                await self.backend.stop(StopReason.GRIP_RELEASED)
            elif self.machine.mode == TeleopMode.ACTIVE and is_new_frame:
                if self.last_target is None:
                    raise RuntimeError("active_without_target")
                raw_requested = self.mapper.target(
                    Pose(p=received.frame.right.p, q=received.frame.right.q)
                )
                requested = self.filter.update(raw_requested, 0.02)
                target = self.limiter.limit(self.last_target, requested, 0.02)
                await self.backend.command_tcp(target, received.frame.seq)
                self.last_target = target
            self.last_seq = received.frame.seq
            self._last_frame_id = frame_id
        except (BackendCommandError, SafetyViolation) as error:
            await self._enter_fault(str(error))

    async def state_message(self) -> RobotStateMessage:
        state = await self.backend.get_state()
        server_mono_ns = self.clock.now_ns()
        received = self.latest.snapshot()
        sample_age_ms = (
            None
            if received is None
            else max(0.0, (server_mono_ns - received.received_ns) / 1_000_000)
        )
        message = state.model_copy(
            update={
                "server_mono_ns": server_mono_ns,
                "ack_seq": self.last_seq,
                "mode": self.machine.mode,
                "sample_age_ms": sample_age_ms,
                "fault": self._fault,
            }
        )
        await self.recorder.write_robot_state(message, server_mono_ns)
        if self._pending_stop_completion and self.machine.mode in {
            TeleopMode.STALE,
            TeleopMode.FAULT,
        } and (
            self._hard_stop_completion or state.robot_state != BackendState.MOVING
        ):
            self._pending_stop_completion = False
            self._hard_stop_completion = False
            self.machine.stop_complete()
        return message

    async def _send_latest_gripper(self, value: float, now_ns: int) -> None:
        if self._last_gripper_sent is None:
            await self.backend.set_gripper(value)
            self._last_gripper_sent = value
            self._last_gripper_sent_ns = now_ns
            return
        if self._last_gripper_sent_ns is None:
            return
        if now_ns - self._last_gripper_sent_ns < GRIPPER_PERIOD_NS:
            return
        delta = abs(value - self._last_gripper_sent)
        if delta < GRIPPER_MIN_DELTA or math.isclose(
            delta,
            GRIPPER_MIN_DELTA,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            return
        await self.backend.set_gripper(value)
        self._last_gripper_sent = value
        self._last_gripper_sent_ns = now_ns

    async def _safe_stop(self, reason: StopReason) -> None:
        if reason == StopReason.STALE:
            if self.machine.mode == TeleopMode.FAULT:
                self._pending_stop_completion = True
                return
            if self.machine.mode != TeleopMode.STALE:
                self.machine.stale()
                await self.backend.stop(reason)
            self._pending_stop_completion = True
            return
        await self.backend.stop(reason)

    async def _enter_fault(self, fault: str) -> None:
        if self.machine.mode != TeleopMode.FAULT:
            self.machine.fault()
            self._fault = fault
            await self.backend.stop(StopReason.FAULT)
        self._pending_stop_completion = True

    def _clear_stop_episode(self) -> None:
        self._pending_stop_completion = False
        self._hard_stop_completion = False
        self._fault = None
