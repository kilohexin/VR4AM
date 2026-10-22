from __future__ import annotations

import asyncio
import contextlib
import math
from dataclasses import dataclass
from typing import Callable, Literal

from app.control.coordinate_mapper import CoordinateMapper
from app.control.filters import PoseFilter
from app.control.safety import SafetyLimiter, SafetyViolation, WorkspaceProjection
from app.control.state_machine import TeleopStateMachine
from app.control.transactional_settings import PursuitSettings, SegmentedSettings
from app.control.target_strategy import (
    DefaultTargetStrategy, TargetContext, TargetLifecycle, TargetStrategy, TargetUpdate,
)
from app.recording.base import RecorderSink
from app.recording.commissioning import RecorderUnavailable
from app.robots.base import (
    BackendCommandError,
    HomeOptions,
    GripperCommandRevoked,
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
CONTROL_DT_S = CONTROL_PERIOD_NS / 1_000_000_000
TARGET_DT_CAP_NS = 2 * CONTROL_PERIOD_NS
OVERRUN_NS = 40_000_000
GRIPPER_PERIOD_NS = 100_000_000
GRIPPER_MIN_DELTA = 0.02
GRIPPER_ACTIVATION_DELTA = 0.1
VR_FRAME_STALE_MS = 100.0
TARGET_SAMPLE_PERIOD_NS = 100_000_000
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
SimulationScaleRejectReason = Literal[
    "not_simulation",
    "not_stopped",
    "invalid_scale",
    "automation_active",
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
class SimulationScaleResult:
    accepted: bool
    translation_scale: float
    reason: SimulationScaleRejectReason | None = None


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
        self._grip_release_count = 0

    def publish(self, frame: VRFrame, received_ns: int) -> None:
        if self._value is not None:
            current = self._value.frame
            if frame.session_id == current.session_id:
                if frame.seq <= current.seq:
                    return
                if current.right.grip and not frame.right.grip:
                    self._grip_release_count += 1
            else:
                if frame.session_id in self._retired_sessions:
                    return
                self._retired_sessions.add(current.session_id)
        self._value = ReceivedFrame(frame, received_ns)

    @property
    def grip_release_count(self) -> int:
        """Accepted clutch edges survive coalescing without queuing motion frames."""
        return self._grip_release_count

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
        target_strategy: TargetStrategy | None = None,
        target_lifecycle: TargetLifecycle | None = None,
        bounded_target_timing: bool = False,
        transactional_settings: PursuitSettings | None = None,
        segmented_settings: SegmentedSettings | None = None,
        prepare_on_arm: bool = False,
        explicit_preparation: bool = False,
        prepare_options: HomeOptions | None = None,
        finite_position_session=None,
    ) -> None:
        self.backend = backend
        self.latest = latest
        self.clock = clock
        self.recorder = recorder
        self.finite_position_session=finite_position_session
        self._finite_position_enabled=getattr(backend,'position_candidate',False) is True
        if type(prepare_on_arm) is not bool:
            raise ValueError('invalid_prepare_on_arm')
        if prepare_on_arm and getattr(backend, 'supports_prepare_on_arm', False) is not True:
            raise ValueError('prepare_on_arm_not_supported')
        self._prepare_on_arm = prepare_on_arm
        if type(explicit_preparation) is not bool or explicit_preparation and not prepare_on_arm:
            raise ValueError('explicit_preparation_requires_prepare_capability')
        self._explicit_preparation = explicit_preparation
        self._preparation_unlock = None
        self.prepare_options = prepare_options or HomeOptions(.05,60.,.001,.02,.3)
        self._preparation_episode: RecoveryEpisode | None = None
        self._explicit_gripper_current: Callable[[], bool] | None = None
        self.mapper = mapper if mapper is not None else CoordinateMapper()
        self.limiter = limiter if limiter is not None else SafetyLimiter()
        if type(bounded_target_timing) is not bool:
            raise ValueError('bounded_target_timing_must_be_bool')
        self._bounded_target_timing = bounded_target_timing
        self.target_strategy = target_strategy if target_strategy is not None else DefaultTargetStrategy()
        self.target_lifecycle = target_lifecycle
        if segmented_settings is not None:
            if (not isinstance(segmented_settings, SegmentedSettings)
                    or transactional_settings is not None
                    or target_strategy is not None or target_lifecycle is not None):
                raise ValueError('conflicting_target_strategy')
            from app.control.segmented_strategy import SegmentedTargetStrategy
            strategy = SegmentedTargetStrategy(backend, self.limiter, context=self._target_context)
            self.target_strategy = self.target_lifecycle = strategy
        if transactional_settings is not None:
            if target_strategy is not None or target_lifecycle is not None:
                raise ValueError('conflicting_target_strategy')
            from app.control.transactional_strategy import TransactionalTargetStrategy
            strategy = TransactionalTargetStrategy(backend, self.limiter,
                transactional_settings, context=self._target_context)
            self.target_strategy = strategy
            self.target_lifecycle = strategy
        self._target_lifecycle_failed = False
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
        self._last_target_sample_ns: int | None = None
        self._target_sample_rate_limited_frames = 0
        self._last_sample_age_ms: float | None = None
        self._last_gripper_sent: float | None = None
        self._last_gripper_sent_ns: int | None = None
        self._armed_trigger_baseline: float | None = None
        self._trigger_changed_since_arm = False
        self._observed_grip_release_count = latest.grip_release_count
        self._grip_stop_in_progress = False
        self._target_hold_release_count: int | None = None
        self._consecutive_overruns = 0
        self._deadline_rebase_requested = False
        self._deadline_compensation_ns = 0
        self._last_target_input_ns: int | None = None
        self._last_target_step_ns: int | None = None
        self._pending_stop_completion = False
        self._hard_stop_completion = False
        self._completed_stale_stop = False
        self._stop_unverified = False
        self._stop_unverified_from_cancelled_stop = False
        self._fault: str | None = None
        self._constraint: ConstraintKind | None = None
        self._constraint_valid_since_ns: int | None = None
        self._recovery_phase: RecoveryPhase | None = None
        self._running = False
        self._loop_failed = False
        self._task: asyncio.Task[None] | None = None
        self._backend_stop_owners: set[asyncio.Task] = set()
        self._lifecycle_lock = asyncio.Lock()
        self._recovery_lock = asyncio.Lock()
        self._shutdown_started = False
        self._shutdown_stopped = False
        self._control_generation = 0
        self._disconnect_stop_pending_frame = False
        self._disarm_in_progress = False
        self._disconnecting = 0
        bind_motion_authority = getattr(backend, "bind_motion_authority", None)
        if callable(bind_motion_authority):
            bind_motion_authority(self._has_motion_dispatch_authority)
        bind_gripper_authority = getattr(backend, "bind_gripper_authority", None)
        if callable(bind_gripper_authority):
            bind_gripper_authority(self._has_gripper_dispatch_authority)

    def _has_motion_dispatch_authority(self) -> bool:
        context = self._target_context()
        return context.active and context.authorized

    def _has_gripper_dispatch_authority(self) -> bool:
        received = self.latest.snapshot()
        return (
            self._explicit_gripper_request_is_current()
            and
            received is not None
            and self._gripper_authority_is_current(self.control_generation)
            and self._frame_is_current(received, self.control_generation)
            and received.received_ns <= self.clock.now_ns()
            and received.frame.tracking_valid
            and received.frame.visibility == "visible"
            and (self._last_frame_id is None
                 or received.frame.session_id == self._last_frame_id[0])
            and (self.mode in {TeleopMode.ARMED,TeleopMode.HOLD} or received.frame.right.grip)
        )

    @property
    def mode(self) -> TeleopMode:
        return self.machine.mode

    @property
    def control_generation(self) -> int:
        return self._control_generation

    async def connect(self) -> None:
        state = await self.backend.get_state()
        self.machine.connect()
        self._completed_stale_stop = False
        self._last_gripper_sent = state.gripper
        self._last_gripper_sent_ns = self.clock.now_ns()
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

    async def set_simulation_scale(
        self,
        value: float,
        *,
        automation_active: bool,
    ) -> SimulationScaleResult:
        current = self.mapper.translation_scale
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            return SimulationScaleResult(False, current, "invalid_scale")
        scaled = round(float(value) * 10)
        if (
            not math.isclose(float(value) * 10, scaled, abs_tol=1e-9)
            or not 5 <= scaled <= 100
        ):
            return SimulationScaleResult(False, current, "invalid_scale")
        if automation_active:
            return SimulationScaleResult(False, current, "automation_active")
        state = await self.backend.get_state()
        if state.backend != "LEBAI_FAKE":
            return SimulationScaleResult(False, current, "not_simulation")
        if (
            self.machine.mode
            not in {
                TeleopMode.READY,
                TeleopMode.HOLD,
                TeleopMode.DISARMED,
                TeleopMode.ARMED,
            }
            or state.robot_state is not BackendState.IDLE
            or self.mapper.has_anchor
        ):
            return SimulationScaleResult(False, current, "not_stopped")
        requested = scaled / 10
        if not math.isclose(current, requested, abs_tol=1e-12):
            recorded = await self._record_critical(
                "simulation_scale_changed",
                {"previous": current, "current": requested},
            )
            if not recorded:
                self._latch_recording_fault()
                return SimulationScaleResult(False, current, "not_stopped")
            self.mapper.set_translation_scale(requested)
        return SimulationScaleResult(True, self.mapper.translation_scale)

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
        if self._explicit_preparation:
            episode=self._recovery_episode()
            if (not self.machine.can_arm or not self._preparation_is_current(episode)
                    or self._pending_stop_completion or self._stop_unverified):
                raise RuntimeError('unlock_requires_fresh_grip_release')
            if not await self._record_critical('preparation_unlocked',{'mode':self.machine.mode.value}):
                self._latch_recording_fault()
                raise RuntimeError('unlock_recording_unavailable')
            if not self._preparation_is_current(episode):raise RuntimeError('arm_authority_changed')
            self._preparation_unlock=(episode,self.latest.grip_release_count)
            return
        if self._prepare_on_arm:
            if self._recovery_lock.locked():
                raise RuntimeError('arm_blocked_by_recovery')
            async with self._recovery_lock:
                await self._prepare_and_arm()
            return
        await self._arm_without_prepare()

    def _preparation_unlocked(self) -> bool:
        grant=self._preparation_unlock
        return bool(grant is not None and grant[1]==self.latest.grip_release_count
            and self._preparation_is_current(grant[0])
            and not self._pending_stop_completion and not self._stop_unverified)

    async def prepare(self) -> None:
        if (not self._explicit_preparation or not self._preparation_unlocked()
                or self._recovery_lock.locked()):
            self._preparation_unlock=None
            raise RuntimeError('prepare_requires_unlock')
        self._preparation_unlock=None  # A failed/cancelled request cannot replay.
        async with self._recovery_lock:
            await self._prepare_and_arm()

    async def _arm_without_prepare(self, *, ensure_prepared: Callable[[], None] | None = None) -> None:
        if self._shutdown_started:
            raise RuntimeError("control_shutdown")
        if self._loop_failed:
            raise RuntimeError("control_faulted")
        if self._fault is not None:
            raise RuntimeError("arm_blocked_by_fault")
        if self._recovery_lock.locked() and ensure_prepared is None:
            raise RuntimeError("arm_blocked_by_recovery")
        generation, mode = self._control_generation, self.machine.mode

        def ensure_current() -> None:
            if ensure_prepared is not None:
                ensure_prepared()
                return
            if (
                self.machine.mode is not mode
                or not self._control_generation_is_current(generation)
            ):
                raise RuntimeError("arm_authority_changed")

        preflight = await self.backend.preflight()
        ensure_current()
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
        ensure_current()
        if not preflight.ready:
            raise RuntimeError(f"arm_blocked_by_preflight:{preflight.reason}")
        recorded = await self._record_critical(
            "state_transition",
            {"from": self.machine.mode.value, "to": TeleopMode.ARMED.value},
        )
        if not recorded:
            self._latch_recording_fault()
            raise RuntimeError("arm_blocked_by_preflight:recording_unavailable")
        ensure_current()
        self.machine.arm()
        self._completed_stale_stop = False
        # The physical gripper may not match the released Trigger position.
        # Observe the first qualified frame before treating Trigger as a command.
        self._armed_trigger_baseline = None
        self._trigger_changed_since_arm = False
        # Connecting only observes the gripper; it does not issue a command.
        # Make the first authorized delta eligible immediately after Arm.
        self._last_gripper_sent_ns = None

    def _preparation_is_current(self, episode: RecoveryEpisode) -> bool:
        received = self.latest.snapshot()
        return (
            not self._recovery_episode_changed(episode)
            and self.machine.mode is TeleopMode.READY
            and self._fault is None and not self._loop_failed
            and not self._shutdown_started and not self._disconnecting
            and not self._disarm_in_progress and not self._grip_stop_in_progress
            and not self._target_lifecycle_failed
            and self.latest.grip_release_count == self._observed_grip_release_count
            and received is not None
            and 0 <= self.clock.now_ns() - received.received_ns < VR_FRAME_STALE_MS * 1_000_000
            and received.frame.tracking_valid and received.frame.visibility == 'visible'
            and not received.frame.right.grip
        )

    async def _prepare_and_arm(self) -> None:
        # Only an explicit operator request enters this path. Connection and reconnect
        # never prepare. The original recovery owns joint writes and stopping.
        episode = self._recovery_episode()
        if not self.machine.can_arm or not self._preparation_is_current(episode):
            raise RuntimeError('arm_prepare_requires_fresh_grip_release')
        self._recovery_phase = 'stopping'
        motion_started = False
        try:
            await self._stop_backend(StopReason.HOME)
            expected = RecoveryEpisode(episode.control_generation + 1, episode.session_id)
            if not self._preparation_is_current(expected):
                raise RuntimeError('arm_authority_changed')
            episode = expected
            self._preparation_episode = episode

            def ensure_current() -> None:
                if not self._preparation_is_current(episode):
                    raise RuntimeError('arm_authority_changed')

            idle_rejection = await self._wait_for_home_idle()
            ensure_current()
            if idle_rejection is not None:
                raise RuntimeError('arm_prepare_failed:not_stopped')
            preflight = await self.backend.preflight()
            ensure_current()
            recorded = await self._record_critical('preflight_result', {
                'ready': preflight.ready, 'reason': preflight.reason,
                'robot_state': preflight.robot_state.value,
                'tcp_matches': preflight.tcp_matches,
                'capabilities': list(preflight.capabilities),
            })
            if not recorded:
                self._latch_recording_fault()
                raise RuntimeError('arm_prepare_failed:recording_unavailable')
            ensure_current()
            if not preflight.ready and preflight.reason != 'singular_configuration':
                raise RuntimeError(f'arm_prepare_failed:{preflight.reason}')

            def on_phase(phase: HomePhase) -> None:
                # Original Real.prepare invokes homing synchronously after
                # capturing backend generations and before taking the SDK lock.
                # This prevents a control revoke during its snapshot read from
                # being adopted as a new recovery generation.
                ensure_current()
                self._recovery_phase = phase

            motion_started = True
            await self.backend.prepare(self.prepare_options if self._explicit_preparation else self.home_options, on_phase, authorize=ensure_current)
            ensure_current()
            self.mapper.clear()
            self.filter.clear()
            self.limiter.clear()
            self.last_target = None
            self._clear_constraint()
            received = self.latest.snapshot()
            assert received is not None
            self._last_frame_id = (received.frame.session_id, received.frame.seq)
            # Recheck ordinary teleoperation preflight after arrival; the
            # recovery's singularity exception grants no joystick authority.
            await self._arm_without_prepare(ensure_prepared=ensure_current)
            if self._finite_position_enabled:self._install_position_session()
        except asyncio.CancelledError:
            if motion_started:
                await self._stop_failed_home_motion()
            self._lock_after_home_failure()
            raise
        except Exception as error:
            if motion_started:
                await self._stop_failed_home_motion()
            self._lock_after_home_failure()
            if isinstance(error, RuntimeError) and str(error).startswith('arm_'):
                raise
            raise RuntimeError('arm_prepare_failed') from error
        finally:
            self._preparation_episode = None
            self._recovery_phase = None

    async def disarm(self) -> None:
        if self.machine.mode in {TeleopMode.STALE, TeleopMode.FAULT}:
            return
        self._disarm_in_progress = True
        self.last_target = None
        try:
            if self.finite_position_session is not None:
                await self.finite_position_session.abort(StopReason.GRIP_RELEASED)
                self.finite_position_session=None
            if self.machine.mode == TeleopMode.ACTIVE:
                self.mapper.clear()
            try:
                stopped_with_recording = await self._stop_backend(
                    StopReason.GRIP_RELEASED
                )
            finally:
                self.mapper.clear()
                self.last_target = None
                self._clear_constraint()
                if self._stop_unverified:
                    self.machine.fault()
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
        finally:
            self._disarm_in_progress = False

    async def home(self) -> HomeResult:
        if self.finite_position_session is not None and self.finite_position_session.busy:
            raise RuntimeError('position_busy')
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
                if (
                    self._control_generation
                    != home_episode.control_generation + 1
                    or self._recovery_episode().session_id
                    != home_episode.session_id
                ):
                    self._lock_after_home_failure()
                    return HomeResult(
                        False,
                        "not_stopped",
                        "Control state changed while stopping; retry Home.",
                    )
                home_episode = self._recovery_episode()
                idle_rejection = await self._wait_for_home_idle()
                if idle_rejection is not None:
                    self._lock_after_home_failure()
                    return idle_rejection

                interrupted = self._home_snapshot_rejection(home_episode)
                if interrupted is not None:
                    self._lock_after_home_failure()
                    return interrupted

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
                    return HomeResult(
                        False,
                        "home_failed",
                        "Home preflight could not be recorded safely.",
                    )
                if (
                    not preflight.ready
                    and preflight.reason != "singular_configuration"
                ):
                    self._lock_after_home_failure()
                    return HomeResult(
                        False,
                        "home_failed",
                        f"Home preflight failed: {preflight.reason}.",
                    )

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
            expected_stop_episode = RecoveryEpisode(
                control_generation=reset_episode.control_generation + 1,
                session_id=reset_episode.session_id,
            )
            await self._stop_backend(StopReason.FAULT)
            reset_episode = expected_stop_episode
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
        # The session's old input remains stored until cleanup completes.
        # Withdraw its authority before any stop/logging await can let run()
        # collect a revoked pursuit or prepare another target.
        self._disconnecting += 1
        try:
            if self.finite_position_session is not None:
                await self.finite_position_session.abort(StopReason.DISCONNECT)
                self.finite_position_session=None
            await self._disconnect_session()
        finally:
            self._disconnecting -= 1

    async def _disconnect_session(self) -> None:
        self._disconnect_stop_pending_frame = False
        self._recovery_phase = None
        self._advance_control_generation()
        stopped_with_recording = False
        try:
            stopped_with_recording = await self._stop_backend(
                StopReason.DISCONNECT
            )
            if stopped_with_recording:
                self._resolve_cancelled_stop_with_verified_disconnect()
        finally:
            self.mapper.clear()
            self.filter.clear()
            self.limiter.clear()
            self.last_target = None
            self._clear_constraint()
            if self._stop_unverified:
                self.machine.fault()
        if not stopped_with_recording:
            return
        self._disconnect_stop_pending_frame = True
        if self._fault is None:
            self._clear_stop_episode()
        self.machine.disconnect()

    async def stop(self) -> None:
        if self.finite_position_session is not None:
            self._shutdown_started=True
            await self.finite_position_session.abort(StopReason.SHUTDOWN)
        async with self._lifecycle_lock:
            if self._shutdown_stopped:
                return
            self._shutdown_started = True
            self._advance_control_generation()
            self._running = False
            task = self._task
            if task is not None and task is not asyncio.current_task():
                if task in self._backend_stop_owners:
                    # The loop is already stopping the robot. Give that bounded
                    # operation a chance to finish instead of manufacturing a
                    # cancelled-stop fault during ordinary application shutdown.
                    await asyncio.wait({task}, timeout=1.0)
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError, Exception):
                    await task
            self._task = None
            await self._stop_backend(StopReason.SHUTDOWN)
            self._shutdown_stopped = True

    async def run(self) -> None:
        next_deadline_ns = self.clock.now_ns()
        next_wakeup_ns: int | None = None
        previous_tick_duration_ns: int | None = None
        self._deadline_compensation_ns = 0
        try:
            while self._running:
                now_ns = self.clock.now_ns()
                if self._deadline_rebase_requested:
                    next_deadline_ns = now_ns
                    self._consecutive_overruns = 0
                    self._deadline_rebase_requested = False
                    self._deadline_compensation_ns = 0
                lateness_ns = now_ns - next_deadline_ns
                if (self._finite_position_enabled and self.finite_position_session is not None
                        and not self._recovery_lock.locked() and lateness_ns>OVERRUN_NS):
                    # This offline position candidate has no PVAT stream to
                    # catch up. Its independent input/entry/completion deadlines
                    # remain active in Session/Adapter; never replay missed ticks.
                    self.backend._schedule_sdk_diagnostic(dict(kind='finite_position_scheduler_late',
                        lateness_ns=lateness_ns,stream_catchup=False,input_deadline_ms=100))
                    next_deadline_ns=now_ns;self._consecutive_overruns=0;lateness_ns=0
                latest = self.latest.snapshot()
                no_client_frame = self._position_idle_deadline_rebase() or (
                    self.machine.mode in {TeleopMode.DISCONNECTED, TeleopMode.READY}
                    and latest is None
                )
                stopped_with_stale_frame = (
                    self.machine.mode in {TeleopMode.DISCONNECTED, TeleopMode.DISARMED}
                    and latest is not None
                    and now_ns - latest.received_ns >= VR_FRAME_STALE_MS * 1_000_000
                )
                if (
                    lateness_ns > OVERRUN_NS
                    and (no_client_frame or stopped_with_stale_frame)
                    and self._fault is None
                    and not self._pending_stop_completion
                    and not self._stop_unverified
                ):
                    # No active control or unresolved stop owns this deadline.
                    # A retained stale frame must not turn an idle delay into a fault.
                    next_deadline_ns = now_ns
                    lateness_ns = 0
                    self._consecutive_overruns = 0
                if lateness_ns > OVERRUN_NS:
                    self._consecutive_overruns += 1
                else:
                    self._consecutive_overruns = 0
                if self._consecutive_overruns >= 2:
                    latest = self.latest.snapshot()
                    await self._enter_fault(
                        "control_overrun",
                        diagnostics={
                            "mode_before_fault": self.machine.mode.value,
                            "deadline_lateness_ns": lateness_ns,
                            "previous_tick_duration_ns": previous_tick_duration_ns,
                            "wakeup_lateness_ns": (
                                max(0, now_ns - next_wakeup_ns)
                                if next_wakeup_ns is not None
                                else None
                            ),
                            "frame_age_ms": (
                                max(0.0, (now_ns - latest.received_ns) / 1_000_000)
                                if latest is not None and latest.received_ns <= now_ns
                                else None
                            ),
                        },
                    )
                tick_started_ns = self.clock.now_ns()
                await self.tick()
                previous_tick_duration_ns = self.clock.now_ns() - tick_started_ns
                if self._deadline_rebase_requested:
                    next_deadline_ns = self.clock.now_ns()
                    self._consecutive_overruns = 0
                    self._deadline_rebase_requested = False
                    self._deadline_compensation_ns = 0
                else:
                    # A successful claw RPC can occupy an ARMED tick without
                    # arm motion; account for only that wait, not other work.
                    next_deadline_ns += self._deadline_compensation_ns
                    self._deadline_compensation_ns = 0
                next_deadline_ns += CONTROL_PERIOD_NS
                before_sleep_ns = self.clock.now_ns()
                delay_ns = max(0, next_deadline_ns - before_sleep_ns)
                next_wakeup_ns = before_sleep_ns + delay_ns
                await asyncio.sleep(delay_ns / 1_000_000_000)
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
        if self._disconnecting or self._disarm_in_progress or self._grip_stop_in_progress:
            return
        episode = self._preparation_episode
        if episode is not None and not self._preparation_is_current(episode):
            # Recovery normally suppresses control ticks. Preparation still
            # needs the original 100-ms input lease and immediate stop revoke.
            self._preparation_episode = None
            await self.disarm()
            return
        if (self._disconnecting or self._disarm_in_progress
                or self._recovery_lock.locked() or self._grip_stop_in_progress):
            return
        if self._finite_position_enabled and self.finite_position_session is not None:
            session=self.finite_position_session
            await session.advance()
            phase=session.view().phase
            if phase=='executing':self.machine.observe_grip(True)
            elif phase in ('ready','draining','hold'):self.machine.observe_grip(False)
            if phase=='fault' and self._fault is None:
                await self._enter_fault(session.view().reason or 'position_fault')
            return
        if await self._consume_grip_release():
            return
        if self.target_lifecycle is None:
            await self._tick_frame()
            return
        try:
            if await self._apply_target_update():
                return
            await self._tick_frame()
            await self._apply_target_update()
        except (BackendCommandError, SafetyViolation) as error:
            await self._enter_fault(str(error))

    async def _consume_grip_release(self) -> bool:
        count = self.latest.grip_release_count
        if count == self._observed_grip_release_count:
            return False
        self._observed_grip_release_count = count
        received = self.latest.snapshot()
        if (
            self.machine.mode is not TeleopMode.ACTIVE
            or self._fault is not None or self._loop_failed or self._shutdown_started
            or received is None
            or not received.frame.tracking_valid or received.frame.visibility != 'visible'
            or not 0 <= self.clock.now_ns() - received.received_ns < VR_FRAME_STALE_MS * 1_000_000
            or (self._last_frame_id is not None and received.frame.session_id != self._last_frame_id[0])
        ):
            return False

        self._grip_stop_in_progress = True
        generation = self._control_generation
        self.machine.observe_grip(False)
        self.mapper.clear()
        self._clear_constraint()
        self._armed_trigger_baseline = None
        self._trigger_changed_since_arm = False
        try:
            frame_id = (received.frame.session_id, received.frame.seq)
            consume_frame = not received.frame.right.grip and frame_id != self._last_frame_id
            if consume_frame:
                await self.recorder.write_vr_frame(received.frame, received.received_ns)
            recorded = await self._record_critical(
                'state_transition', {'from': 'ACTIVE', 'to': 'HOLD'},
            )
            if not recorded:
                await self._handle_recording_unavailable()
            elif (
                self.machine.mode is TeleopMode.HOLD
                and generation == self._control_generation
                and self._fault is None and not self._shutdown_started
                and not self._disarm_in_progress and not self._disconnecting
                and not self._recovery_lock.locked()
            ):
                await self._stop_backend(StopReason.GRIP_RELEASED)
                if self.machine.mode is TeleopMode.HOLD and self._fault is None and consume_frame:
                    # The release was accepted before stop's bounded wait. Do
                    # not reinterpret that same frame as stale after the wait.
                    self.last_seq = received.frame.seq
                    self._last_frame_id = frame_id
        except asyncio.CancelledError:
            if self.machine.mode is TeleopMode.HOLD and not self._stop_unverified:
                self._latch_stop_unverified(from_cancelled_stop=True)
            raise
        except RecorderUnavailable:
            await self._handle_recording_unavailable()
        except (BackendCommandError, SafetyViolation) as error:
            await self._enter_fault(str(error))
        finally:
            self._grip_stop_in_progress = False
        return True

    def _target_context(self) -> TargetContext:
        received = self.latest.snapshot()
        authorized = (
            received is not None
            and self._target_frame_is_current(received, self.control_generation)
            and received.received_ns <= self.clock.now_ns()
            and received.frame.right.grip and received.frame.tracking_valid
            and received.frame.visibility == 'visible'
            and (self._last_frame_id is None
                 or received.frame.session_id == self._last_frame_id[0])
        )
        return TargetContext(self.control_generation, self.clock.now_ns(),
                             self.mode is TeleopMode.ACTIVE, authorized)

    async def _apply_target_update(self) -> bool:
        if self._target_lifecycle_failed:
            if self._fault is None:
                await self._enter_fault('target_strategy_failed')
            return True
        context = self._target_context()
        try:
            update = self.target_lifecycle.collect(context)
            if not isinstance(update, TargetUpdate):
                raise TypeError('invalid_target_update')
        except (Exception, asyncio.CancelledError):
            self._target_lifecycle_failed = True
            await self._enter_fault('target_strategy_failed')
            return True
        if update.generation != context.generation or context.generation != self.control_generation:
            return False
        if update.fault is not None:
            if self._fault is None:
                await self._enter_fault(update.fault)
                return True
            return False
        if not context.authorized or not self._target_context().authorized:
            return False
        if update.stop:
            await self.disarm()
            return True
        if update.hold:
            self._target_hold_release_count=self.latest.grip_release_count
            self.machine.observe_grip(False)
            self.mapper.clear()
            self.filter.clear()
            self.limiter.clear()
            self.last_target=None
            self._clear_constraint()
            self._advance_control_generation()
            try:
                recorded=await self._record_critical('target_hold',{'reason':'segment_unsent_expired','requires_release':True})
                await self._stop_backend(StopReason.STALE)
                if not recorded:self._latch_recording_fault()
            except asyncio.CancelledError:
                self._latch_stop_unverified(from_cancelled_stop=True)
                raise
            return True
        if update.committed is not None:
            self.last_target = update.committed.model_copy(deep=True)
        return False

    async def _tick_frame(self) -> None:
        if self._recovery_lock.locked() or self._disarm_in_progress:
            return
        received = self.latest.snapshot()
        if received is None:
            return
        disconnect_stop_completed = self._disconnect_stop_pending_frame
        self._disconnect_stop_pending_frame = False
        now_ns = self.clock.now_ns()
        if received.received_ns > now_ns:
            # A different clock epoch or backwards clock cannot grant freshness.
            self._last_sample_age_ms = None
            self.mapper.clear()
            self.filter.clear()
            self.limiter.clear()
            self.last_target = None
            if not (self._completed_stale_stop and self.mode is TeleopMode.DISARMED):
                await self._safe_stop(StopReason.STALE)
            return
        age_ms = max(0.0, (now_ns - received.received_ns) / 1_000_000)
        self._last_sample_age_ms = age_ms
        if self._completed_stale_stop and self.machine.mode == TeleopMode.DISARMED and (
            age_ms >= VR_FRAME_STALE_MS
            or not received.frame.tracking_valid
            or received.frame.visibility != "visible"
        ):
            return
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
            frame_generation = self._control_generation
            if is_new_frame:
                await self.recorder.write_vr_frame(received.frame, received.received_ns)
                if not self._frame_is_current(received, frame_generation):
                    return
            if (
                self._fault is None
                and not self._loop_failed
                and not self._shutdown_started
                and self.machine.mode in {TeleopMode.ARMED, TeleopMode.ACTIVE, TeleopMode.HOLD}
                and not (self.machine.mode is TeleopMode.ACTIVE and not received.frame.right.grip)
            ):
                gripper_generation = self._control_generation
                gripper_wait_ns = await self._send_latest_gripper(
                    received.frame.right.trigger, now_ns
                )
                if gripper_wait_ns is not None and self.machine.mode == TeleopMode.ARMED:
                    self._deadline_compensation_ns += gripper_wait_ns
                # A claw RPC can yield before Grip transitions or anchor
                # capture. Revalidate the input and authority after it returns.
                if (
                    self.machine.mode not in {TeleopMode.ARMED, TeleopMode.ACTIVE, TeleopMode.HOLD}
                    or not self._frame_is_current(received, gripper_generation)
                ):
                    return
            previous_mode = self.machine.mode
            if (
                is_new_frame
                and self._fault is None
                and not self._loop_failed
                and not self._shutdown_started
            ):
                mode_before_grip = self.machine.mode
                if mode_before_grip in {TeleopMode.ARMED,TeleopMode.HOLD} and self._target_hold_release_count is not None:
                    # This is a fresh, validated frame after the HOLD decision.
                    # A new session's released input has no inherited edge count.
                    if not received.frame.right.grip:
                        self._target_hold_release_count=None
                    else:
                        self.last_seq=received.frame.seq
                        self._last_frame_id=frame_id
                        return
                if (
                    mode_before_grip == TeleopMode.HOLD
                    and received.frame.right.grip
                ):
                    preflight = await self.backend.preflight()
                    if (
                        self.machine.mode is not mode_before_grip
                        or not self._frame_is_current(received, frame_generation)
                    ):
                        return
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
                        await self._handle_recording_unavailable()
                        return
                    if (
                        self.machine.mode is not mode_before_grip
                        or not self._frame_is_current(received, frame_generation)
                    ):
                        return
                    if not preflight.ready:
                        reason = preflight.reason or "unknown"
                        raise BackendCommandError(
                            f"preflight_not_ready:{reason}"
                        )
                activating = (
                    mode_before_grip in {TeleopMode.ARMED, TeleopMode.HOLD}
                    and received.frame.right.grip
                )
                if activating:
                    # Keep the stopped/armed mode while obtaining the anchor.
                    # No await may separate a validated capture from activation.
                    state = await self.backend.get_state()
                    if (
                        self.machine.mode is not mode_before_grip
                        or not self._frame_is_current(received, frame_generation)
                    ):
                        return
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
                    self._last_target_input_ns = None
                    self._last_target_step_ns = None
                self.machine.observe_grip(received.frame.right.grip)
                if activating and self.target_lifecycle is not None:
                    try:
                        self.target_lifecycle.on_anchor(
                            state.actual_tcp.model_copy(deep=True), self._target_context(),
                        )
                    except (Exception, asyncio.CancelledError) as error:
                        self._target_lifecycle_failed = True
                        raise BackendCommandError('target_strategy_failed') from error
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
                    if activating and not self._target_frame_is_current(received, frame_generation):
                        return
            if (
                previous_mode in {TeleopMode.ARMED, TeleopMode.HOLD}
                and self.machine.mode == TeleopMode.ACTIVE
            ):
                if previous_mode == TeleopMode.HOLD:
                    self._deadline_rebase_requested = True
            elif previous_mode == TeleopMode.ACTIVE and self.machine.mode == TeleopMode.HOLD:
                self.mapper.clear()
                self._clear_constraint()
                self._armed_trigger_baseline = None
                self._trigger_changed_since_arm = False
                await self._stop_backend(StopReason.GRIP_RELEASED)
            elif self.machine.mode == TeleopMode.ACTIVE and is_new_frame:
                if self.last_target is None:
                    raise RuntimeError("active_without_target")
                generation = self._control_generation
                previous_target = self.last_target
                control_dt_s, input_interval_ns, processing_interval_ns = self._target_step_timing(received)
                raw_requested = self.mapper.target(
                    Pose(p=received.frame.right.p, q=received.frame.right.q)
                )
                workspace = self.limiter.project_workspace(raw_requested)
                requested = None
                target = None
                plan = None
                if not (workspace.constrained and workspace.hold):
                    requested = self.filter.update(workspace.pose, control_dt_s)
                    plan = self.target_strategy.prepare(
                        previous_target, requested, control_dt_s, limiter=self.limiter,
                    )
                    target = plan.pose
                await self._record_target_sample(
                    received=received,
                    generation=generation,
                    previous_target=previous_target,
                    mapped=raw_requested,
                    workspace=workspace,
                    filtered=requested,
                    limited=target,
                    dispatch_kind=plan.kind if plan is not None else 'workspace_hold',
                    control_dt_s=control_dt_s,
                    input_interval_ns=input_interval_ns,
                    processing_interval_ns=processing_interval_ns,
                )
                # Recording implementations can yield. Do not submit an old
                # target after a stop, a newer input, or an expired VR frame.
                if not self._target_frame_is_current(received, generation):
                    if (
                        generation == self._control_generation
                        and self.last_target is not None
                    ):
                        self.filter.reset(self.last_target)
                        self.limiter.reset_motion()
                        self._last_target_step_ns = self.clock.now_ns()
                    return
                if workspace.constrained and workspace.hold:
                    self.filter.reset(self.last_target)
                    self.limiter.reset_motion()
                    self._set_constraint("workspace_boundary")
                    self.last_seq = received.frame.seq
                    self._last_frame_id = frame_id
                    self._last_target_step_ns = self.clock.now_ns()
                    return
                assert plan is not None
                try:
                    committed = None
                    if plan.kind != 'wait':
                        committed = await self.target_strategy.dispatch(
                            plan, received.frame.seq, backend=self.backend,
                        )
                except BackendCommandError as error:
                    code = str(error)
                    if code in {"ik_unreachable", "ik_singular"}:
                        constraint = "ik_boundary"
                    elif code in {
                        "joint_safety_window",
                        "ik_joint_limit",
                    }:
                        constraint = "joint_boundary"
                    elif code == "ik_tracking_lag":
                        constraint = "tracking_lag"
                    elif code in {
                        "ik_joint_jump",
                        "joint_speed_limit",
                    }:
                        constraint = "motion_continuity_boundary"
                    elif code == "self_collision":
                        constraint = "self_collision"
                    else:
                        raise
                    if not self._target_frame_is_current(received, generation):
                        if generation == self._control_generation:
                            self._last_target_step_ns = self.clock.now_ns()
                        return
                    self._set_constraint(constraint)
                    self.filter.reset(self.last_target)
                    self.limiter.reset_motion()
                else:
                    # Dispatch may wait. Never restore a stopped anchor or
                    # consume an old frame after a newer input takes ownership.
                    if not self._target_frame_is_current(received, generation):
                        if generation == self._control_generation:
                            self._last_target_step_ns = self.clock.now_ns()
                        return
                    if committed is not None:
                        self.last_target = committed.model_copy(deep=True)
                        if workspace.constrained:
                            self._set_constraint("workspace_boundary")
                        else:
                            self._observe_valid_constraint(now_ns)
                self._last_target_step_ns = self.clock.now_ns()
            self.last_seq = received.frame.seq
            self._last_frame_id = frame_id
        except RecorderUnavailable:
            await self._handle_recording_unavailable()
        except (BackendCommandError, SafetyViolation) as error:
            await self._enter_fault(str(error))

    def _target_step_timing(self, received: ReceivedFrame) -> tuple[float, int | None, int | None]:
        # Consume each evaluated new input once, even when its target is later
        # held/rejected/abandoned. Never accumulate a backlog of motion time.
        # Only server arrival timestamps are comparable; client clocks are not.
        previous = self._last_target_input_ns
        now_ns = self.clock.now_ns()
        processing_interval = (None if self._last_target_step_ns is None
            else now_ns - self._last_target_step_ns)
        self._last_target_step_ns = now_ns
        if previous is None or received.received_ns > previous:
            self._last_target_input_ns = received.received_ns
        interval = None if previous is None else received.received_ns - previous
        # A delayed previous recorder/dispatch must not credit overlapping time.
        # Retain the nominal 50 Hz floor; only sparse-input steps can grow.
        elapsed = (CONTROL_PERIOD_NS if interval is None or processing_interval is None
            else min(interval, processing_interval))
        step_ns = min(TARGET_DT_CAP_NS, max(CONTROL_PERIOD_NS, elapsed))
        if not self._bounded_target_timing:
            step_ns = CONTROL_PERIOD_NS
        return step_ns / 1_000_000_000, interval, processing_interval

    async def _record_target_sample(
        self,
        *,
        received: ReceivedFrame,
        generation: int,
        previous_target: Pose,
        mapped: Pose,
        workspace: WorkspaceProjection,
        filtered: Pose | None,
        limited: Pose | None,
        dispatch_kind: str = 'candidate',
        control_dt_s: float = CONTROL_DT_S,
        input_interval_ns: int | None = None,
        processing_interval_ns: int | None = None,
    ) -> None:
        now_ns = self.clock.now_ns()
        if (
            self._last_target_sample_ns is not None
            and now_ns - self._last_target_sample_ns < TARGET_SAMPLE_PERIOD_NS
        ):
            self._target_sample_rate_limited_frames += 1
            return
        # This is a pre-submission sample, not evidence of IK acceptance or
        # robot execution. Use the recorder's bounded normal-event queue.
        await self.recorder.write_event(
            {
                "kind": "control_target_sample",
                "session_id": received.frame.session_id,
                "command_id": received.frame.seq,
                "control_generation": generation,
                "frame_received_ns": received.received_ns,
                "control_dt_s": control_dt_s,
                "target_input_interval_ns": input_interval_ns,
                "target_processing_interval_ns": processing_interval_ns,
                "control_dt_cap_s": (TARGET_DT_CAP_NS / 1_000_000_000
                    if self._bounded_target_timing else CONTROL_DT_S),
                "target_timing_policy": ('bounded_server_arrival' if self._bounded_target_timing else 'nominal20'),
                "dispatch_kind": dispatch_kind,
                "sample_period_ns": TARGET_SAMPLE_PERIOD_NS,
                "rate_limited_frames": self._target_sample_rate_limited_frames,
                "translation_scale": self.mapper.translation_scale,
                "hand_pose": Pose(
                    p=received.frame.right.p, q=received.frame.right.q,
                ).model_dump(mode="json"),
                "previous_target_tcp": previous_target.model_dump(mode="json"),
                "mapped_tcp": mapped.model_dump(mode="json"),
                "workspace_tcp": workspace.pose.model_dump(mode="json"),
                "workspace_constrained": workspace.constrained,
                "workspace_hold": workspace.hold,
                "filtered_tcp": (
                    None if filtered is None else filtered.model_dump(mode="json")
                ),
                "limited_tcp": (
                    None if limited is None else limited.model_dump(mode="json")
                ),
            },
            now_ns,
        )
        self._last_target_sample_ns = now_ns
        self._target_sample_rate_limited_frames = 0

    def _target_frame_is_current(
        self, received: ReceivedFrame, generation: int,
    ) -> bool:
        return (
            self.machine.mode is TeleopMode.ACTIVE
            and self._frame_is_current(received, generation)
        )

    def _frame_is_current(
        self, received: ReceivedFrame, generation: int,
    ) -> bool:
        return (
            self._control_generation_is_current(generation)
            and self.latest.snapshot() is received
            and 0 <= self.clock.now_ns() - received.received_ns
            < VR_FRAME_STALE_MS * 1_000_000
        )

    def _control_generation_is_current(self, generation: int) -> bool:
        return (
            generation == self._control_generation
            and self.latest.grip_release_count == self._observed_grip_release_count
            and not self._grip_stop_in_progress
            and self._fault is None
            and not self._loop_failed
            and not self._shutdown_started
            and not self._disarm_in_progress
            and not self._disconnecting
            and not self._recovery_lock.locked()
            and not self._target_lifecycle_failed
        )

    async def state_message(self) -> RobotStateMessage:
        """Observe, compose, record, then advance pending stop state in that order."""
        state = await self._read_observed_state()
        message = self._compose_state_message(state)
        await self._record_state_message(message)
        # Recheck live control flags after recording yields. The returned message
        # deliberately describes the state before this completion transition.
        self._complete_pending_stop_from_state(state)
        return message

    async def _read_observed_state(self) -> RobotStateMessage:
        """Read the backend and apply its fault observation to control authority."""
        state = await self.backend.get_state()
        if state.fault is not None and self._fault is None:
            self._latch_backend_fault(state.fault)
        return state

    def _position_idle_deadline_rebase(self):
        return bool(self._finite_position_enabled and self.machine.mode in {TeleopMode.READY,TeleopMode.DISARMED}
            and not self._fault and not self._stop_unverified and not self._pending_stop_completion
            and not self._recovery_lock.locked() and not self._recovery_phase
            and self.finite_position_session is None and not self.backend.position_busy
            and not self.backend._motion_accepted and not self.backend._motion_write_stop_required
            and not self.backend._latched_fault and not self.backend._motion_calls.unknown_methods
            and not self.backend._snapshot_calls.unknown_methods)

    def _install_position_session(self):
        from app.control.finite_position_session import FinitePositionSession
        binding=self.backend.bind_position_generation(self._control_generation)
        def authorized():
            if (self._control_generation!=binding.generation or self._fault or self._shutdown_started
                    or self._disconnecting or self._disarm_in_progress or self._recovery_lock.locked()
                    or self.machine.mode not in {TeleopMode.ARMED,TeleopMode.ACTIVE,TeleopMode.HOLD}):
                raise ValueError('position_authority_revoked')
            received=self.latest.snapshot()
            if received is not None and (not received.frame.tracking_valid or received.frame.visibility!='visible'):
                raise ValueError('position_input_unavailable')
        self.finite_position_session=FinitePositionSession(self.backend,binding,clock=self.clock.now_ns,
            recorder=self.recorder,authorize=authorized)

    def accept_position_input(self,message,received_ns):
        from app.control.finite_position_types import PositionIntent
        session=self.finite_position_session
        if not self._finite_position_enabled or session is None:
            raise ValueError('position_not_prepared')
        session.offer(PositionIntent(message.session_id,message.generation,message.seq,received_ns,
            message.kind,message.vector))

    def _compose_state_message(self, state: RobotStateMessage) -> RobotStateMessage:
        """Overlay current control metadata without I/O or control transitions.

        Composition does not observe faults or confirm a stop. This legacy wire
        message also has no robot-measurement age; it is not a cached-display API.
        """
        server_mono_ns = self.clock.now_ns()
        received = self.latest.snapshot()
        sample_age_ms = (
            # The legacy wire field measures VR input age, not robot telemetry age.
            None
            if received is None or received.received_ns > server_mono_ns
            else max(0.0, (server_mono_ns - received.received_ns) / 1_000_000)
        )
        status_message = getattr(self.target_strategy, 'status_message', None)
        return state.model_copy(
            update={
                "server_mono_ns": server_mono_ns,
                "ack_seq": self.last_seq,
                "mode": self.machine.mode,
                "sample_age_ms": sample_age_ms,
                "fault": self._fault or state.fault,
                "constraint": self._constraint,
                "recovery_phase": self._recovery_phase,
                "translation_scale": self.mapper.translation_scale,
                "target_control": status_message() if callable(status_message) else None,
                "manual_input_ready": self.machine.mode in {TeleopMode.ARMED,TeleopMode.ACTIVE,TeleopMode.HOLD}
                    and self._control_generation_is_current(self._control_generation)
                    and self._target_hold_release_count is None
                    and not self._pending_stop_completion and not self._stop_unverified,
                "preparation_required": self._explicit_preparation,
                "preparation_unlocked": self._preparation_unlocked(),
                "finite_position": self._position_status_message(),
            }
        )

    def _position_status_message(self):
        if not self._finite_position_enabled:return None
        from dataclasses import asdict
        from app.control.finite_position_types import PositionView
        from app.schemas.messages import FinitePositionStatus
        view=self.finite_position_session.view() if self.finite_position_session is not None else PositionView(generation=self._control_generation)
        return FinitePositionStatus(**asdict(view),policy=getattr(self.backend,'position_policy','finite_position_offline'))

    async def _record_state_message(self, message: RobotStateMessage) -> None:
        """Record the composed message and retain the existing failure response."""
        try:
            await self.recorder.write_robot_state(message, message.server_mono_ns)
        except RecorderUnavailable:
            await self._handle_recording_unavailable()

    def _complete_pending_stop_from_state(self, state: RobotStateMessage) -> None:
        """Apply existing completion rules to this call's authoritative read.

        Must run after recording; a display cache must never call this helper.
        """
        if self._pending_stop_completion and self.machine.mode in {
            TeleopMode.STALE,
            TeleopMode.FAULT,
        } and (
            not self._stop_unverified
            and (
                self._hard_stop_completion
                or state.robot_state is BackendState.IDLE
            )
        ):
            if self.machine.mode == TeleopMode.STALE:
                self._completed_stale_stop = True
            self._pending_stop_completion = False
            self._hard_stop_completion = False
            self.machine.stop_complete()
            self._advance_control_generation()

    async def command_gripper(self, value: float, *, is_current: Callable[[], bool] | None = None) -> None:
        if self.finite_position_session is not None and self.finite_position_session.busy:
            raise RuntimeError('position_busy')
        if self._explicit_gripper_current is not None:
            raise RuntimeError('gripper_command_busy')
        self._explicit_gripper_current = is_current or (lambda: True)
        try:
            await self._command_gripper(value)
        finally:
            self._explicit_gripper_current = None

    def _explicit_gripper_request_is_current(self) -> bool:
        callback = self._explicit_gripper_current
        if callback is None:
            return True
        try:
            return callback() is True
        except Exception:
            return False

    async def _command_gripper(self, value: float) -> None:
        generation = self._control_generation
        if not self._gripper_authority_is_current(generation):
            raise RuntimeError("gripper_command_requires_armed")
        preflight = await self.backend.preflight()
        if not self._gripper_authority_is_current(generation):
            raise RuntimeError("gripper_command_authority_changed")
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
            raise RuntimeError("gripper_command_preflight_failed:recording_unavailable")
        if not self._gripper_authority_is_current(generation):
            raise RuntimeError("gripper_command_authority_changed")
        if not preflight.ready:
            raise RuntimeError(
                f"gripper_command_preflight_failed:{preflight.reason}"
            )
        if not await self._try_set_gripper(value):
            raise RuntimeError("gripper_command_authority_changed")
        if not self._gripper_authority_is_current(generation):
            raise RuntimeError("gripper_command_authority_changed")
        self._last_gripper_sent = value
        self._last_gripper_sent_ns = self.clock.now_ns()

    async def _try_set_gripper(self, value: float) -> bool:
        try:
            await self.backend.set_gripper(value)
        except GripperCommandRevoked:
            return False
        return True

    def _gripper_authority_is_current(self, generation: int) -> bool:
        return (
            self._explicit_gripper_request_is_current()
            and
            self.machine.mode in {TeleopMode.ARMED, TeleopMode.ACTIVE, TeleopMode.HOLD}
            and self._control_generation_is_current(generation)
        )

    async def _send_latest_gripper(self, value: float, now_ns: int) -> int | None:
        had_intent = self._trigger_changed_since_arm
        if self._armed_trigger_baseline is None:
            self._armed_trigger_baseline = value
            return None
        if not self._trigger_changed_since_arm:
            # Require a deliberate press before mapping Trigger to the claw.
            # Small analog jitter or releasing a held Trigger is not intent.
            self._armed_trigger_baseline = min(self._armed_trigger_baseline, value)
            if value - self._armed_trigger_baseline < GRIPPER_ACTIVATION_DELTA:
                return None
            self._trigger_changed_since_arm = True
        if self._last_gripper_sent is None:
            started_ns = self.clock.now_ns()
            if not await self._try_set_gripper(value):
                if not had_intent:
                    self._trigger_changed_since_arm = False
                return None
            wait_ns = max(0, self.clock.now_ns() - started_ns)
            self._last_gripper_sent = value
            self._last_gripper_sent_ns = now_ns
            return wait_ns
        delta = abs(value - self._last_gripper_sent)
        if delta < GRIPPER_MIN_DELTA or math.isclose(
            delta,
            GRIPPER_MIN_DELTA,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            return None
        if (
            self._last_gripper_sent_ns is not None
            and now_ns - self._last_gripper_sent_ns < GRIPPER_PERIOD_NS
        ):
            return None
        started_ns = self.clock.now_ns()
        if not await self._try_set_gripper(value):
            if not had_intent:
                self._trigger_changed_since_arm = False
            return None
        wait_ns = max(0, self.clock.now_ns() - started_ns)
        self._last_gripper_sent = value
        self._last_gripper_sent_ns = now_ns
        return wait_ns

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
        self._advance_control_generation()
        recorded = await self._record_critical(
            "stop_requested",
            {"reason": reason.value},
        )
        try:
            await self._await_backend_stop(reason)
        except BaseException as error:
            fault = (
                "stop_incomplete"
                if isinstance(error, BackendCommandError)
                and str(error) == "stop_incomplete"
                else "stop_unverified"
            )
            self._latch_stop_unverified(
                fault,
                from_cancelled_stop=isinstance(error, asyncio.CancelledError),
            )
            await self._record_critical(
                "stop_failed",
                {
                    "reason": reason.value,
                    "error": str(error),
                    "error_type": type(error).__name__,
                    "fault": fault,
                },
            )
            if not recorded:
                self._latch_recording_fault()
            raise
        confirmed = await self._record_critical(
            "stop_confirmed",
            {"reason": reason.value},
        )
        self._deadline_rebase_requested = True
        if not recorded or not confirmed:
            self._latch_recording_fault()
            return False
        return True

    async def _await_backend_stop(self, reason: StopReason) -> None:
        owner = asyncio.current_task()
        assert owner is not None
        self._backend_stop_owners.add(owner)
        try:
            await self.backend.stop(reason)
        finally:
            self._backend_stop_owners.discard(owner)

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

    def _latch_stop_unverified(
        self,
        fault: str = "stop_unverified",
        *,
        from_cancelled_stop: bool = False,
    ) -> None:
        changed = not self._stop_unverified or self._fault != fault
        self._stop_unverified = True
        self._stop_unverified_from_cancelled_stop = from_cancelled_stop
        self._fault = fault
        self.mapper.clear()
        self.filter.clear()
        self.limiter.clear()
        self.last_target = None
        self._clear_constraint()
        self._recovery_phase = None
        if self.machine.mode is not TeleopMode.FAULT:
            self.machine.fault()
            changed = True
        if not self._pending_stop_completion:
            self._pending_stop_completion = True
            changed = True
        self._hard_stop_completion = False
        if changed:
            self._advance_control_generation()

    def _resolve_cancelled_stop_with_verified_disconnect(self) -> None:
        if not self._stop_unverified_from_cancelled_stop:
            return
        if self._fault != "stop_unverified":
            return
        self._stop_unverified = False
        self._stop_unverified_from_cancelled_stop = False
        self._fault = None
        self._pending_stop_completion = False
        self._hard_stop_completion = False
        self._advance_control_generation()

    def _latch_backend_fault(self, fault: str) -> None:
        changed = self._fault != fault or self.machine.mode is not TeleopMode.FAULT
        self._fault = fault
        self._stop_unverified = True
        self.mapper.clear()
        self.filter.clear()
        self.limiter.clear()
        self.last_target = None
        self._clear_constraint()
        self._recovery_phase = None
        self.machine.fault()
        if not self._pending_stop_completion:
            self._pending_stop_completion = True
            changed = True
        self._hard_stop_completion = False
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

    async def _enter_fault(
        self,
        fault: str,
        *,
        diagnostics: dict[str, object] | None = None,
    ) -> None:
        self._clear_constraint()
        self._recovery_phase = None
        if self.machine.mode != TeleopMode.FAULT:
            self.machine.fault()
            self._fault = fault
            self._advance_control_generation()
            payload: dict[str, object] = {"reason": fault}
            if diagnostics is not None:
                payload.update(diagnostics)
            await self._record_critical(
                "robot_fault",
                payload,
            )
            try:
                await self._stop_backend(StopReason.FAULT)
            except asyncio.CancelledError:
                raise
            except Exception:
                pass
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
            self._stop_unverified = False
            self._stop_unverified_from_cancelled_stop = False
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
        if self.finite_position_session is not None:
            self.backend.position_aborted=True
        self._preparation_unlock=None
        self._last_target_input_ns = None
        self._last_target_step_ns = None
        if self.target_lifecycle is not None:
            try:
                self.target_lifecycle.on_revoke(self._control_generation)
            except (Exception, asyncio.CancelledError):
                # Never let a strategy callback interrupt permission revocation
                # or prevent the stop that follows it. Fail closed on next tick.
                # A synchronous hook cannot receive task cancellation at an
                # await: its own CancelledError is a callback failure. Actual
                # cancellation in the owner's awaited stop still propagates.
                self._target_lifecycle_failed = True

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
        sample_age_ns = self.clock.now_ns() - received.received_ns
        if (
            not 0 <= sample_age_ns < VR_FRAME_STALE_MS * 1_000_000
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
