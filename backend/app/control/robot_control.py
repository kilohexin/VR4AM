from __future__ import annotations

import asyncio
import contextlib
import math
from dataclasses import dataclass
from typing import Literal

from app.control.coordinate_mapper import CoordinateMapper
from app.control.filters import PoseFilter
from app.control.safety import SafetyLimiter, SafetyViolation
from app.control.state_machine import TeleopStateMachine
from app.recording.base import RecorderSink
from app.recording.commissioning import RecorderUnavailable
from app.robots.base import (
    BackendCommandError,
    HomeOptions,
    HomePhase,
    RobotBackend,
    StopReason,
)
from app.schemas.messages import (
    BackendState,
    ConstraintKind,
    Pose,
    RecoveryPhase,
    RobotStateMessage,
    TeleopMode,
    VRFrame,
)
from app.timebase import MonotonicClock

CONTROL_PERIOD_NS = 20_000_000
OVERRUN_NS = 40_000_000
GRIPPER_PERIOD_NS = 100_000_000
GRIPPER_MIN_DELTA = 0.02
VR_FRAME_STALE_MS = 100.0
RECOVERABLE_FAULTS = frozenset(
    {
        "workspace_violation",
        "ik_unreachable",
        "ik_singular",
        "joint_safety_window",
        "backend_command_failed",
    }
)
HomeRejectReason = Literal[
    "fault_present",
    "grip_pressed",
    "not_stopped",
    "control_loop_unavailable",
    "home_failed",
]


@dataclass(frozen=True)
class FaultResetResult:
    accepted: bool
    reason: str | None = None
    message: str | None = None


@dataclass(frozen=True)
class HomeResult:
    accepted: bool
    reason: HomeRejectReason | None = None
    message: str | None = None


@dataclass(frozen=True)
class RecoveryEpisode:
    control_generation: int
    session_id: str | None


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
        mapper: CoordinateMapper | None = None,
        limiter: SafetyLimiter | None = None,
        constraint_clear_ms: int = 100,
        home_options: HomeOptions | None = None,
    ) -> None:
        self.backend = backend
        self.latest = latest
        self.clock = clock
        self.recorder = recorder
        self.mapper = mapper if mapper is not None else CoordinateMapper()
        self.limiter = limiter if limiter is not None else SafetyLimiter()
        self.constraint_clear_ns = constraint_clear_ms * 1_000_000
        self.home_options = home_options or HomeOptions(
            max_speed_radps=0.25,
            timeout_s=15.0,
            position_tolerance_rad=math.radians(1.0),
            velocity_tolerance_radps=0.02,
            stable_seconds=0.3,
        )
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
        self._constraint: ConstraintKind | None = None
        self._constraint_valid_since_ns: int | None = None
        self._recovery_phase: RecoveryPhase | None = None
        self._running = False
        self._loop_failed = False
        self._task: asyncio.Task[None] | None = None
        self._lifecycle_lock = asyncio.Lock()
        self._recovery_lock = asyncio.Lock()
        self._shutdown_started = False
        self._shutdown_stopped = False
        self._control_generation = 0
        self._disconnect_stop_pending_frame = False

    @property
    def mode(self) -> TeleopMode:
        return self.machine.mode

    @property
    def control_generation(self) -> int:
        return self._control_generation

    async def connect(self) -> None:
        self.machine.connect()
        if self._fault is not None:
            if self._pending_stop_completion:
                self.machine.fault()
            else:
                self.machine.disarm()
        self._advance_control_generation()
        if self._fault is None:
            self._clear_stop_episode()

    async def initialize_observed_gripper(self) -> float:
        if (
            self._running
            or self.machine.mode not in {TeleopMode.READY, TeleopMode.DISARMED}
        ):
            raise RuntimeError("gripper_initialize_requires_stopped")
        if self._fault is not None or self._shutdown_started:
            raise RuntimeError("gripper_initialize_unavailable")
        state = await self.backend.get_state()
        self._last_gripper_sent = state.gripper
        self._last_gripper_sent_ns = self.clock.now_ns()
        return state.gripper

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
                self._advance_control_generation()
            self._running = True
            self._task = asyncio.create_task(self.run())

    async def arm(self) -> None:
        if self._shutdown_started:
            raise RuntimeError("control_shutdown")
        if self._loop_failed:
            raise RuntimeError("control_faulted")
        if self._fault is not None:
            raise RuntimeError("arm_blocked_by_fault")
        if self._recovery_lock.locked():
            raise RuntimeError("arm_blocked_by_recovery")
        preflight = await self.backend.preflight()
        recorded = await self._record_critical(
            "preflight_result",
            {
                "ready": preflight.ready,
                "reason": preflight.reason,
                "robot_state": preflight.robot_state.value,
                "tcp_matches": preflight.tcp_matches,
                "capabilities": list(preflight.capabilities),
            },
        )
        if not recorded:
            self._latch_recording_fault()
            raise RuntimeError("arm_blocked_by_preflight:recording_unavailable")
        if not preflight.ready:
            raise RuntimeError(f"arm_blocked_by_preflight:{preflight.reason}")
        recorded = await self._record_critical(
            "state_transition",
            {"from": self.machine.mode.value, "to": TeleopMode.ARMED.value},
        )
        if not recorded:
            self._latch_recording_fault()
            raise RuntimeError("arm_blocked_by_preflight:recording_unavailable")
        self.machine.arm()

    async def disarm(self) -> None:
        if self.machine.mode in {TeleopMode.STALE, TeleopMode.FAULT}:
            return
        if self.machine.mode == TeleopMode.ACTIVE:
            self.mapper.clear()
        stopped_with_recording = await self._stop_backend(StopReason.GRIP_RELEASED)
        self._clear_constraint()
        if not stopped_with_recording:
            return
        recorded = await self._record_critical(
            "state_transition",
            {"from": self.machine.mode.value, "to": TeleopMode.DISARMED.value},
        )
        if not recorded:
            self._latch_recording_fault()
            return
        self.machine.disarm()

    async def home(self) -> HomeResult:
        async with self._recovery_lock:
            return await self._home_locked()

    async def _home_locked(self) -> HomeResult:
        rejection = self._home_rejection()
        if rejection is not None:
            return rejection
        home_episode = self._recovery_episode()

        self._recovery_phase = "stopping"
        home_motion_started = False
        try:
            try:
                await self._stop_backend(StopReason.HOME)
                idle_rejection = await self._wait_for_home_idle()
                if idle_rejection is not None:
                    self._lock_after_home_failure()
                    return idle_rejection

                interrupted = self._home_snapshot_rejection(home_episode)
                if interrupted is not None:
                    self._lock_after_home_failure()
                    return interrupted

                def on_home_phase(phase: HomePhase) -> None:
                    self._recovery_phase = phase

                home_motion_started = True
                await self.backend.home(self.home_options, on_home_phase)
            except asyncio.CancelledError:
                if home_motion_started:
                    await self._stop_failed_home_motion()
                self._lock_after_home_failure()
                raise
            except Exception:
                if home_motion_started:
                    await self._stop_failed_home_motion()
                self._lock_after_home_failure()
                return HomeResult(
                    False,
                    "home_failed",
                    "仿真无法返回初始姿态，请稍后重试。",
                )

            interrupted = self._home_snapshot_rejection(home_episode)
            if interrupted is not None:
                await self._stop_failed_home_motion()
                self._lock_after_home_failure()
                return interrupted

            release_cutoff = self.latest.snapshot()
            if release_cutoff is not None:
                self._last_frame_id = (
                    release_cutoff.frame.session_id,
                    release_cutoff.frame.seq,
                )
            self.mapper.clear()
            self.filter.clear()
            self.limiter.clear()
            self.last_target = None
            self._clear_constraint()
            self.machine.disarm()
            return HomeResult(True)
        finally:
            self._recovery_phase = None

    async def reset_fault(self) -> FaultResetResult:
        async with self._recovery_lock:
            return await self._reset_fault_locked()

    async def _reset_fault_locked(self) -> FaultResetResult:
        rejection = self._fault_reset_rejection()
        if rejection is not None:
            return rejection
        reset_episode = self._recovery_episode()
        reset_fault = self._fault
        try:
            state = await self.backend.get_state()
        except Exception:
            return FaultResetResult(
                False,
                "control_loop_unavailable",
                "控制循环不可用，请重启后端并重新检查。",
            )
        rejection = self._fault_reset_snapshot_rejection(
            reset_episode,
            reset_fault,
        )
        if rejection is not None:
            return rejection
        backend_rejection = self._backend_reset_rejection(state.robot_state)
        if backend_rejection is not None:
            return backend_rejection
        self._recovery_phase = "stopping"
        try:
            await self._stop_backend(StopReason.FAULT)
        except Exception:
            self._recovery_phase = None
            return FaultResetResult(
                False,
                "stop_incomplete",
                "无法确认仿真已停止，故障保持锁定。",
            )
        rejection = self._fault_reset_snapshot_rejection(
            reset_episode,
            reset_fault,
        )
        if rejection is not None:
            self._recovery_phase = None
            return rejection

        def on_home_phase(phase: HomePhase) -> None:
            self._recovery_phase = phase

        try:
            await self.backend.home(self.home_options, on_home_phase)
        except asyncio.CancelledError:
            await self._stop_failed_home_motion()
            raise
        except Exception:
            await self._stop_failed_home_motion()
            return FaultResetResult(
                False,
                "stop_incomplete",
                "仿真无法返回初始姿态，故障保持锁定。",
            )
        finally:
            self._recovery_phase = None

        rejection = self._fault_reset_snapshot_rejection(
            reset_episode,
            reset_fault,
        )
        if rejection is not None:
            await self._stop_failed_home_motion()
            return rejection
        release_cutoff = self.latest.snapshot()
        if release_cutoff is not None:
            self._last_frame_id = (
                release_cutoff.frame.session_id,
                release_cutoff.frame.seq,
            )
        self.mapper.clear()
        self.filter.clear()
        self.limiter.clear()
        self.last_target = None
        self._clear_constraint()
        self._clear_stop_episode(clear_fault=True)
        self.machine.disarm()
        return FaultResetResult(True)

    async def on_disconnect(self) -> None:
        self._disconnect_stop_pending_frame = False
        self._recovery_phase = None
        self._advance_control_generation()
        await self._stop_backend(StopReason.DISCONNECT)
        self._disconnect_stop_pending_frame = True
        self.mapper.clear()
        self._clear_constraint()
        if self._fault is None:
            self._clear_stop_episode()
        self.machine.disconnect()

    async def stop(self) -> None:
        async with self._lifecycle_lock:
            if self._shutdown_stopped:
                return
            self._shutdown_started = True
            self._advance_control_generation()
            self._running = False
            task = self._task
            if task is not None and task is not asyncio.current_task():
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await task
            self._task = None
            await self._stop_backend(StopReason.SHUTDOWN)
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
            self._clear_constraint()
            self._recovery_phase = None
            self.machine.fault()
            self._fault = "control_loop_error"
            self._pending_stop_completion = True
            self._advance_control_generation()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self._stop_backend(StopReason.FAULT)
            raise

    async def tick(self) -> None:
        if self._recovery_lock.locked():
            return
        received = self.latest.snapshot()
        if received is None:
            return
        disconnect_stop_completed = self._disconnect_stop_pending_frame
        self._disconnect_stop_pending_frame = False
        now_ns = self.clock.now_ns()
        age_ms = max(0.0, (now_ns - received.received_ns) / 1_000_000)
        self._last_sample_age_ms = age_ms
        if age_ms >= 250:
            await self._safe_stop(StopReason.STALE)
            if not self._hard_stop_completion:
                self._hard_stop_completion = True
                self._advance_control_generation()
            return
        if (
            age_ms >= VR_FRAME_STALE_MS
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
                self._advance_control_generation()
                if not disconnect_stop_completed:
                    await self._stop_backend(StopReason.DISCONNECT)
                self.mapper.clear()
                self.last_target = None
                self._clear_constraint()
                if self.machine.mode not in {TeleopMode.FAULT, TeleopMode.STALE}:
                    self._clear_stop_episode()
                    self.machine.disarm()
            is_new_frame = frame_id != self._last_frame_id
            if is_new_frame:
                await self.recorder.write_vr_frame(received.frame, received.received_ns)
            if (
                self._fault is None
                and not self._loop_failed
                and not self._shutdown_started
            ):
                await self._send_latest_gripper(received.frame.right.trigger, now_ns)
            previous_mode = self.machine.mode
            if (
                is_new_frame
                and self._fault is None
                and not self._loop_failed
                and not self._shutdown_started
            ):
                mode_before_grip = self.machine.mode
                self.machine.observe_grip(received.frame.right.grip)
                if self.machine.mode != mode_before_grip:
                    recorded = await self._record_critical(
                        "state_transition",
                        {
                            "from": mode_before_grip.value,
                            "to": self.machine.mode.value,
                        },
                    )
                    if not recorded:
                        await self._handle_recording_unavailable()
                        return
            if (
                previous_mode in {TeleopMode.ARMED, TeleopMode.HOLD}
                and self.machine.mode == TeleopMode.ACTIVE
            ):
                state = await self.backend.get_state()
                try:
                    self.mapper.capture(
                        Pose(p=received.frame.right.p, q=received.frame.right.q),
                        state.actual_tcp,
                        received.frame.head_q,
                    )
                except RuntimeError as error:
                    if str(error) != "invalid_control_basis":
                        raise
                    raise SafetyViolation("invalid_numeric") from error
                self.limiter.set_pose_anchor(state.actual_tcp)
                self.filter.reset(state.actual_tcp)
                self.last_target = state.actual_tcp
            elif previous_mode == TeleopMode.ACTIVE and self.machine.mode == TeleopMode.HOLD:
                self.mapper.clear()
                self._clear_constraint()
                await self._stop_backend(StopReason.GRIP_RELEASED)
            elif self.machine.mode == TeleopMode.ACTIVE and is_new_frame:
                if self.last_target is None:
                    raise RuntimeError("active_without_target")
                raw_requested = self.mapper.target(
                    Pose(p=received.frame.right.p, q=received.frame.right.q)
                )
                workspace = self.limiter.project_workspace(raw_requested)
                if workspace.constrained and workspace.hold:
                    self.filter.reset(self.last_target)
                    self.limiter.reset_motion()
                    self._set_constraint("workspace_boundary")
                    self.last_seq = received.frame.seq
                    self._last_frame_id = frame_id
                    return
                requested = self.filter.update(workspace.pose, 0.02)
                target = self.limiter.limit_motion(self.last_target, requested, 0.02)
                try:
                    await self.backend.command_tcp(target, received.frame.seq)
                except BackendCommandError as error:
                    code = str(error)
                    if code in {"ik_unreachable", "ik_singular"}:
                        self._set_constraint("ik_boundary")
                    elif code == "joint_safety_window":
                        self._set_constraint("joint_boundary")
                    elif code == "self_collision":
                        self._set_constraint("self_collision")
                    else:
                        raise
                    self.filter.reset(self.last_target)
                    self.limiter.reset_motion()
                else:
                    self.last_target = target
                    if workspace.constrained:
                        self._set_constraint("workspace_boundary")
                    else:
                        self._observe_valid_constraint(now_ns)
            self.last_seq = received.frame.seq
            self._last_frame_id = frame_id
        except RecorderUnavailable:
            await self._handle_recording_unavailable()
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
                "constraint": self._constraint,
                "recovery_phase": self._recovery_phase,
            }
        )
        try:
            await self.recorder.write_robot_state(message, server_mono_ns)
        except RecorderUnavailable:
            await self._handle_recording_unavailable()
        if self._pending_stop_completion and self.machine.mode in {
            TeleopMode.STALE,
            TeleopMode.FAULT,
        } and (
            self._hard_stop_completion or state.robot_state != BackendState.MOVING
        ):
            self._pending_stop_completion = False
            self._hard_stop_completion = False
            self.machine.stop_complete()
            self._advance_control_generation()
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

    async def _record_critical(self, kind: str, payload: object) -> bool:
        try:
            await self.recorder.write_critical_event(
                kind,
                payload,
                self.clock.now_ns(),
            )
        except RecorderUnavailable:
            self._fault = "recording_unavailable"
            return False
        return True

    async def _stop_backend(self, reason: StopReason) -> bool:
        recorded = await self._record_critical(
            "stop_requested",
            {"reason": reason.value},
        )
        try:
            await self.backend.stop(reason)
        except BaseException:
            if not recorded:
                self._latch_recording_fault()
            raise
        confirmed = await self._record_critical(
            "stop_confirmed",
            {"reason": reason.value},
        )
        if not recorded or not confirmed:
            self._latch_recording_fault()
            return False
        return True

    async def _handle_recording_unavailable(self) -> None:
        self._fault = "recording_unavailable"
        if self.machine.mode in {
            TeleopMode.ARMED,
            TeleopMode.ACTIVE,
            TeleopMode.HOLD,
            TeleopMode.STALE,
            TeleopMode.FAULT,
        }:
            await self._stop_backend(StopReason.FAULT)
        self._latch_recording_fault()

    def _latch_recording_fault(self) -> None:
        changed = self._fault != "recording_unavailable"
        self._fault = "recording_unavailable"
        if self.machine.mode != TeleopMode.FAULT:
            self.machine.fault()
            changed = True
        if not self._pending_stop_completion:
            self._pending_stop_completion = True
            changed = True
        if changed:
            self._advance_control_generation()

    async def _safe_stop(self, reason: StopReason) -> None:
        if reason == StopReason.STALE:
            if self.machine.mode == TeleopMode.FAULT:
                if not self._pending_stop_completion:
                    self._pending_stop_completion = True
                    self._advance_control_generation()
                return
            if self.machine.mode != TeleopMode.STALE:
                self.machine.stale()
                self._advance_control_generation()
                await self._stop_backend(reason)
            if not self._pending_stop_completion:
                self._pending_stop_completion = True
                self._advance_control_generation()
            return
        await self._stop_backend(reason)

    async def _enter_fault(self, fault: str) -> None:
        self._clear_constraint()
        self._recovery_phase = None
        if self.machine.mode != TeleopMode.FAULT:
            self.machine.fault()
            self._fault = fault
            self._advance_control_generation()
            await self._record_critical(
                "robot_fault",
                {"reason": fault},
            )
            await self._stop_backend(StopReason.FAULT)
        if not self._pending_stop_completion:
            self._pending_stop_completion = True
            self._advance_control_generation()

    def _clear_stop_episode(self, *, clear_fault: bool = False) -> None:
        changed = (
            self._pending_stop_completion
            or self._hard_stop_completion
            or (clear_fault and self._fault is not None)
        )
        self._pending_stop_completion = False
        self._hard_stop_completion = False
        if clear_fault:
            self._fault = None
        if changed:
            self._advance_control_generation()

    def _set_constraint(self, constraint: ConstraintKind) -> None:
        self._constraint = constraint
        self._constraint_valid_since_ns = None

    def _observe_valid_constraint(self, now_ns: int) -> None:
        if self._constraint is None:
            return
        if self._constraint_valid_since_ns is None:
            self._constraint_valid_since_ns = now_ns
            return
        if now_ns - self._constraint_valid_since_ns >= self.constraint_clear_ns:
            self._clear_constraint()

    def _clear_constraint(self) -> None:
        self._constraint = None
        self._constraint_valid_since_ns = None

    def _advance_control_generation(self) -> None:
        self._control_generation += 1

    def _home_rejection(self) -> HomeResult | None:
        if self._fault is not None:
            return HomeResult(
                False,
                "fault_present",
                "存在未清除故障，请先完成故障复位。",
            )
        if self._loop_failed or self._shutdown_started:
            return HomeResult(
                False,
                "control_loop_unavailable",
                "控制循环不可用，请重启后端并重新检查。",
            )
        if self.machine.mode not in {TeleopMode.READY, TeleopMode.DISARMED}:
            return HomeResult(
                False,
                "not_stopped",
                "仅可在停止状态下执行 Home。",
            )
        received = self.latest.snapshot()
        if received is None:
            return HomeResult(
                False,
                "grip_pressed",
                "请先松开手柄抓握键，再请求 Home。",
            )
        sample_age_ms = max(
            0.0,
            (self.clock.now_ns() - received.received_ns) / 1_000_000,
        )
        if (
            sample_age_ms >= VR_FRAME_STALE_MS
            or not received.frame.tracking_valid
            or received.frame.visibility != "visible"
            or received.frame.right.grip
        ):
            return HomeResult(
                False,
                "grip_pressed",
                "请先松开手柄抓握键，再请求 Home。",
            )
        return None

    def _recovery_episode(self) -> RecoveryEpisode:
        received = self.latest.snapshot()
        return RecoveryEpisode(
            control_generation=self._control_generation,
            session_id=None if received is None else received.frame.session_id,
        )

    def _recovery_episode_changed(self, episode: RecoveryEpisode) -> bool:
        received = self.latest.snapshot()
        session_id = None if received is None else received.frame.session_id
        return (
            self._control_generation != episode.control_generation
            or session_id != episode.session_id
        )

    def _home_snapshot_rejection(
        self,
        home_episode: RecoveryEpisode,
    ) -> HomeResult | None:
        if self._fault is not None:
            return HomeResult(
                False,
                "fault_present",
                "存在未清除故障，请先完成故障复位。",
            )
        if self._loop_failed or self._shutdown_started:
            return HomeResult(
                False,
                "control_loop_unavailable",
                "控制循环不可用，请重启后端并重新检查。",
            )
        if (
            self._recovery_episode_changed(home_episode)
            or self.machine.mode not in {TeleopMode.READY, TeleopMode.DISARMED}
        ):
            return HomeResult(
                False,
                "not_stopped",
                "控制状态已变化，请确认停止后重试。",
            )
        return None

    async def _wait_for_home_idle(self) -> HomeResult | None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + 1.0
        while True:
            remaining = deadline - loop.time()
            if remaining <= 0:
                return HomeResult(False, "home_failed", "等待仿真停止超时。")
            try:
                state = await asyncio.wait_for(
                    self.backend.get_state(),
                    timeout=remaining,
                )
            except TimeoutError:
                return HomeResult(False, "home_failed", "等待仿真停止超时。")
            if loop.time() >= deadline:
                return HomeResult(False, "home_failed", "等待仿真停止超时。")
            if state.robot_state is BackendState.IDLE:
                return None
            if state.robot_state is not BackendState.MOVING:
                return HomeResult(
                    False,
                    "home_failed",
                    "仿真后端无法进入停止状态。",
                )
            await asyncio.sleep(min(0.02, max(0.0, deadline - loop.time())))

    async def _stop_failed_home_motion(self) -> None:
        with contextlib.suppress(asyncio.CancelledError, Exception):
            await self._stop_backend(StopReason.HOME)

    def _lock_after_home_failure(self) -> None:
        if self.machine.mode in {TeleopMode.READY, TeleopMode.DISARMED}:
            self.machine.disarm()

    def _fault_reset_rejection(self) -> FaultResetResult | None:
        if self._fault is None:
            return FaultResetResult(False, "no_fault", "当前没有可复位故障。")
        if self._fault not in RECOVERABLE_FAULTS:
            return FaultResetResult(
                False,
                "unrecoverable_fault",
                "该故障无法在线复位，请重启后端并重新检查。",
            )
        if self._loop_failed or self._shutdown_started:
            return FaultResetResult(
                False,
                "control_loop_unavailable",
                "控制循环不可用，请重启后端并重新检查。",
            )
        if self._pending_stop_completion or self.machine.mode != TeleopMode.DISARMED:
            return FaultResetResult(
                False,
                "stop_incomplete",
                "停止尚未完成，请稍后重试。",
            )
        return None

    def _fault_reset_snapshot_rejection(
        self,
        reset_episode: RecoveryEpisode,
        reset_fault: str | None,
    ) -> FaultResetResult | None:
        rejection = self._fault_reset_rejection()
        if rejection is not None:
            return rejection
        if self._recovery_episode_changed(reset_episode) or self._fault != reset_fault:
            return FaultResetResult(
                False,
                "stop_incomplete",
                "停止尚未完成，请稍后重试。",
            )
        return None

    @staticmethod
    def _backend_reset_rejection(
        robot_state: BackendState,
    ) -> FaultResetResult | None:
        if robot_state == BackendState.IDLE:
            return None
        if robot_state == BackendState.MOVING:
            return FaultResetResult(
                False,
                "backend_moving",
                "仿真仍在运动，请稍后重试。",
            )
        if robot_state == BackendState.DISCONNECTED:
            return FaultResetResult(
                False,
                "control_loop_unavailable",
                "控制循环不可用，请重启后端并重新检查。",
            )
        if robot_state == BackendState.FAULT:
            return FaultResetResult(
                False,
                "stop_incomplete",
                "仿真后端仍处于故障状态，故障保持锁定。",
            )
        return FaultResetResult(
            False,
            "stop_incomplete",
            "仿真尚未完全停止，请稍后重试。",
        )
