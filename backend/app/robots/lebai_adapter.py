from __future__ import annotations

import asyncio
import copy
import math
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass, replace
from typing import Any, Literal

from scipy.spatial.transform import Rotation

from app.recording.clock_anchor import sample_performance_ns
from app.robots.lebai_observer_window import ObserverWindow
from app.robots.lebai_sdk_lock import SdkLock
from app.robots.lebai_motion_calls import OwnedMotionCalls
from app.robots.lebai_snapshot_calls import OwnedSnapshotCalls

from app.config import TELEOP_SINGULARITY_GUARD_RAD, LebaiSettings
from app.robots.base import (
    BackendCommandError,
    BackendPreflight,
    GripperCommandRevoked,
    HomeOptions,
    HomePhase,
    StopReason,
)
from app.robots.lebai_codec import (
    estop_fault,
    joint_vector,
    map_robot_state,
    pose_from_lebai,
    pose_to_lebai,
)
from app.robots.lebai_ik_policy import (
    IkCandidateMetrics,
    evaluate_ik_candidate,
    proportional_recovery_fraction,
)
from app.robots.lebai_sdk_bridge import (
    LebaiClientProtocol,
    SdkCapabilities,
    connect_real_client,
    detect_capabilities,
)
from app.robots.lebai_pump import PvatPump, PvatRequest
from app.robots.command_admission import CommandAdmission, CommandFeedback, CommandRef
from app.robots.motion_view import AcceptedTarget, JointObservation, MotionView
from app.robots.lebai_pvat import PvatLimits, PvatPoint, build_pvat_point
from app.robots.lebai_stop_transaction import StopTransaction
from app.robots.lebai_stop_diagnostics import StopDiagnosticSampler
from app.robots.lebai_recovery import interpolate_pose, recovery_candidates
from app.schemas.messages import (
    BackendState,
    JointVector,
    Pose,
    RobotStateMessage,
    TeleopMode,
)

ClientFactory = Callable[[str], Awaitable[LebaiClientProtocol]]
EventCallback = Callable[[dict[str, object], int], Awaitable[None]]
PvatMode = Literal["advance", "interpolated_advance", "catch_up"]
RECOVERABLE_IK_ERRORS = {
    "ik_unreachable",
    "ik_invalid",
    "ik_joint_limit",
    "ik_joint_jump",
    "joint_speed_limit",
}
_SNAPSHOT_READ_TIMEOUT_NS = 300_000_000
# These raw controller states can be stationary; generic mapped HOLD cannot.
# PAUSED/STOP still fail motion preflight even after a verified stop.
_STOP_SETTLED_STATES = frozenset({"IDLE", "PAUSED", "STOP", 5, 6, 12})
_STOP_JOINT_DRIFT_RAD = 0.001
_STOP_TCP_DRIFT_M = 0.0005


@dataclass(frozen=True)
class LebaiSnapshot:
    captured_ns: int
    read_requested_ns: int
    read_completed_ns: int
    read_purpose: str
    robot_state: BackendState
    estop: str | None
    actual_q: JointVector
    actual_qd: JointVector
    actual_qdd: JointVector
    target_q: JointVector
    target_qd: JointVector
    target_qdd: JointVector
    actual_tcp: Pose
    target_tcp: Pose
    actual_flange: Pose
    tcp_setting: Pose
    gripper: float
    running_motion: object | None
    sdk_latencies_ms: dict[str, float]
    raw_robot_state: str | int | None = None
    sdk_lock_wait_ms: float = 0.0
    raw_kinematics: dict[str, Any] | None = None
    raw_running_motion: object | None = None
    motion_state: str | None = None
    diagnostic_timing: dict[str, object] | None = None


@dataclass(frozen=True)
class _SolvedCandidate:
    target: Pose
    metrics: IkCandidateMetrics
    point: PvatPoint
    recovery_fraction: float
    pvat_mode: PvatMode
    advances_target: bool


@dataclass
class _SnapshotRead:
    client: LebaiClientProtocol | None
    generation: int
    sequence: int = 0
    snapshot: LebaiSnapshot | None = None
    owner: OwnedSnapshotCalls | None = None


class _MotionSnapshotFault(BackendCommandError):
    """A known read fault; process stopping after releasing the SDK lock."""


class _RecoveryRevoked(BackendCommandError):
    """Recovery authority changed; never stop a replacement connection."""


def _pvat_point_inputs(snapshot: LebaiSnapshot, point: PvatPoint) -> dict[str, object]:
    # Saved input evidence only: no new SDK reads, clocks, or planning choices.
    # Each event gets separate lists so recorder mutation cannot cross records.
    return {
        "snapshot_read_requested_ns": snapshot.read_requested_ns,
        "snapshot_read_started_ns": snapshot.captured_ns,
        "snapshot_read_completed_ns": snapshot.read_completed_ns,
        "actual_q": list(snapshot.actual_q),
        "actual_qd": list(snapshot.actual_qd),
        "actual_qdd": list(snapshot.actual_qdd),
        "controller_target_q": list(snapshot.target_q),
        "controller_target_qd": list(snapshot.target_qd),
        "controller_target_qdd": list(snapshot.target_qdd),
        "velocity_reference_source": point.reference_source,
        "velocity_reference_qd": list(point.reference_qd),
    }


class _CandidateRejected(BackendCommandError):
    def __init__(self, reason: str, metrics: IkCandidateMetrics) -> None:
        super().__init__(reason)
        self.metrics = metrics


class RealLebaiAdapter:
    # Internal commissioning subclass hook, never configurable by VR/API input.
    _motion_stop_rpc_method = "stop_move"

    def __init__(
        self,
        settings: LebaiSettings,
        *,
        client_factory: ClientFactory = connect_real_client,
        clock: Callable[[], int] = time.monotonic_ns,
        pvat_clock: Callable[[], int] = time.perf_counter_ns,
        sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
        event_callback: EventCallback | None = None,
        backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI",
        pump_clock: Callable[[], float] | None = None,
        pump_sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
        stop_diagnostic_sampling: bool = False,
        observer_wait_s: float = 0.0,
    ) -> None:
        self.settings = settings
        self._client_factory = client_factory
        self._stop_diagnostic_sampling = stop_diagnostic_sampling
        self._stop_diagnostic_sampler: StopDiagnosticSampler | None = None
        self._clock = clock
        self._pvat_clock = pvat_clock
        self._sleep = sleep
        self._event_callback = event_callback
        self._backend_label = backend_label
        self._client: LebaiClientProtocol | None = None
        self._sdk_lock = SdkLock()
        self._motion_calls = OwnedMotionCalls()
        self._snapshot_calls = OwnedSnapshotCalls()
        self._motion_write_stop_required = False
        self._observer_window = ObserverWindow(observer_wait_s)
        self._stop_lock = asyncio.Lock()
        self._disconnecting = 0
        self._stop_transaction: StopTransaction | None = None
        self._stop_episode_id = 0
        self._gripper_revocation_generation = 0
        self._sdk_request_id = 0
        self._pvat_request_id_in_flight: int | None = None
        self._sdk_diagnostic_tasks: set[asyncio.Task[None]] = set()
        self._sdk_diagnostics_dropped = 0
        self._snapshot: LebaiSnapshot | None = None
        self._snapshot_read_sequence = 0
        self._latest_snapshot_read: _SnapshotRead | None = None
        self._snapshot_read_fault: tuple[_SnapshotRead, str] | None = None
        self._motion_observation: JointObservation | None = None
        self._capabilities = SdkCapabilities(names=())
        self._command_id: int | None = None
        self._motion_accepted = False
        margin = settings.joint_limit_margin_rad
        self._pvat_limits = PvatLimits(
            horizon_s=settings.control.pvat_horizon_s,
            max_joint_speed_radps=settings.control.max_joint_speed_radps,
            max_joint_acceleration_radps2=(
                settings.control.max_joint_acceleration_radps2
            ),
            max_joint_tracking_error_rad=(
                settings.control.max_joint_tracking_error_rad
            ),
            soft_joint_min_rad=tuple(
                value + margin for value in settings.soft_joint_min_rad
            ),
            soft_joint_max_rad=tuple(
                value - margin for value in settings.soft_joint_max_rad
            ),
        )
        self._pump = PvatPump(
            self._send_target,
            period_s=1 / settings.control.pvat_send_hz,
            clock=pump_clock,
            sleep=pump_sleep,
        )
        self._previous_sent_qd: JointVector | None = None
        self._last_pvat_started_ns: int | None = None
        self._last_pvat_clock_started_ns: int | None = None
        self._last_sent_tcp: Pose | None = None
        self._last_accepted_solution_q: JointVector | None = None
        self._accepted_target_command_id: int | None = None
        self._accepted_target_ref: CommandRef | None = None
        self._constraint: str | None = None
        self._constraint_error: str | None = None
        self._constraint_feedback: CommandFeedback | None = None
        self._consecutive_ik_failures = 0
        self._latched_fault: str | None = None
        self._latched_fault_from_cancelled_stop = False
        self._preflight_ready = False
        self._motion_authority: Callable[[], bool] | None = None
        self._gripper_authority: Callable[[], bool] | None = None

    def bind_gripper_authority(self, check: Callable[[], bool]) -> None:
        """Bind a synchronous gate for claw requests that have not entered SDK."""
        if not callable(check):
            raise TypeError("gripper_authority_must_be_callable")
        if self._gripper_authority is not None and self._gripper_authority != check:
            raise RuntimeError("gripper_authority_already_bound")
        self._gripper_authority = check

    def _require_gripper_authority(self) -> None:
        if self._gripper_authority is None:
            return
        try:
            permitted = self._gripper_authority() is True
        except Exception:
            permitted = False
        if not permitted:
            raise GripperCommandRevoked("gripper_command_authority_changed")

    def bind_motion_authority(self, check: Callable[[], bool]) -> None:
        """Gate pending teleop PVAT sends; never stop an RPC already sent.

        The owner still runs its original stop transaction. Standalone callers
        retain the existing adapter contract when no teleop owner is bound.
        """
        if not callable(check):
            raise TypeError("motion_authority_must_be_callable")
        if self._motion_authority is not None and self._motion_authority != check:
            raise RuntimeError("motion_authority_already_bound")
        self._motion_authority = check

    def _motion_dispatch_is_current(
        self, request: PvatRequest, *,
        sdk_outcome: Literal["not_started", "returned"] = "not_started",
        sdk_request_id: int | None = None,
    ) -> bool:
        if not self._pump.is_current(request.generation):
            return False
        if self._motion_authority is None:
            return True
        try:
            permitted = self._motion_authority() is True
        except Exception:
            permitted = False
        if not permitted and request.completion is not None:
            request.completion._invalidate()
            request.completion._resolve(
                "invalidated", sdk_outcome=sdk_outcome,
                sdk_request_id=sdk_request_id,
                error="teleop_motion_authority_revoked",
            )
        return permitted

    @property
    def constraint(self) -> str | None:
        return self._constraint

    @property
    def pump_fault(self) -> BackendCommandError | None:
        return self._pump.fault

    def read_motion_view(self) -> MotionView:
        """Copy cached facts synchronously on the owning event loop; no I/O.

        A missing/stale sample stays missing/stale. No SDK reads, tasks, logging
        or fallback. Old view objects do not track later revocation or faults.
        """
        accepted = None
        if (self._accepted_target_ref is not None and self._last_sent_tcp is not None
                and self._last_accepted_solution_q is not None):
            accepted = AcceptedTarget(
                self._accepted_target_ref, tuple(self._last_sent_tcp.p),
                tuple(self._last_sent_tcp.q), tuple(self._last_accepted_solution_q),
            )
        return MotionView(
            observed_ns=self._clock(), generation=self._pump.generation,
            client_attached=self._client is not None, pump_running=self._pump.running,
            pump_fault=str(self._pump.fault) if self._pump.fault is not None else None,
            latched_fault=self._latched_fault, preflight_ready=self._preflight_ready,
            stop_in_progress=self._stop_lock.locked(), accepted=accepted,
            sample=self._motion_observation,
        )

    async def connect(self) -> None:
        if self._snapshot_calls.unknown_methods:
            raise BackendCommandError("snapshot_rpc_result_unknown")
        if self._motion_calls.unknown_methods:
            raise BackendCommandError("motion_rpc_result_unknown")
        if self._disconnecting:
            raise BackendCommandError("disconnect_in_progress")
        if self._client is not None:
            return
        if (self._snapshot_calls.active
                or any(not task.done() for task in self._snapshot_calls.tasks)):
            raise BackendCommandError("snapshot_read_pending")
        if self._stop_diagnostic_sampler is not None and self._stop_diagnostic_sampler.unresolved:
            raise BackendCommandError("stop_diagnostic_read_pending")
        if self._stop_transaction is not None:
            if any(not r.task.done() for r in self._stop_transaction.requests.values()):
                raise BackendCommandError("stop_request_pending")
            # A new connection must never reuse the previous client's requests.
            # Faults remain latched; connecting cannot grant motion permission.
            self._stop_transaction = None
        if self._snapshot_calls.closed:
            self._snapshot_calls = OwnedSnapshotCalls()
        owner = self._snapshot_calls
        owner.active += 1  # Include the whole connection pipeline and recorder awaits.
        def revoked():
            return owner.closed or owner is not self._snapshot_calls or bool(self._disconnecting)
        def require_current():
            if revoked():
                raise BackendCommandError("snapshot_read_client_changed")
        try:
            try:
                client = await owner.call('connect', lambda: self._client_factory(self.settings.ip),
                    3., revoked=revoked)
            except TimeoutError:
                raise BackendCommandError("sdk_timeout:connect") from None
            require_current()
            try:
                entry, unknown = self._snapshot_native_evidence(client, 'is_connected')
                connected = await owner.call('initial_is_connected', client.is_connected,
                    .2, revoked=revoked, entered=entry, native_result_unknown=unknown)
            except TimeoutError:
                raise BackendCommandError("sdk_timeout:is_connected") from None
            require_current()
            if not connected:
                raise BackendCommandError("robot_disconnected")
            self._client = client
            self._capabilities = detect_capabilities(client)
            snapshot = await self._read_snapshot(read_purpose="connection")
            require_current()
            preflight = self._preflight_from_snapshot(snapshot)
            self._preflight_ready = preflight.ready
            if self.settings.mode == "control" and preflight.ready:
                await self._pump.start()
        except BaseException:
            owner.close()
            self._client = None
            self._snapshot = None
            self._latest_snapshot_read = None
            self._snapshot_read_fault = None
            self._motion_observation = None
            self._preflight_ready = False
            raise
        finally:
            owner.active -= 1

    async def disconnect(self) -> None:
        self._observer_window.release_all()
        # Revoke pending gripper calls even when no motion needs a stop RPC.
        self._gripper_revocation_generation += 1
        self._disconnecting += 1
        try:
            async with self._stop_lock:
                try:
                    if (
                        self.settings.mode == "control"
                        and (self._motion_accepted or self._motion_write_stop_required)
                        and self._client is not None
                    ):
                        self._ensure_stop_transaction()
                        await self._stop_in_transaction(StopReason.SHUTDOWN)
                finally:
                    await self._disconnect_local_resources()
        finally:
            self._disconnecting -= 1

    async def _disconnect_local_resources(self) -> None:
        await self._pump.stop()
        self._snapshot_calls.close()
        self._motion_calls.close()
        if self._stop_diagnostic_sampler is not None:
            await self._stop_diagnostic_sampler.close()
        if self._stop_transaction is not None:
            await self._stop_transaction.close()
        if self._sdk_diagnostic_tasks:
            _, pending = await asyncio.wait(
                tuple(self._sdk_diagnostic_tasks), timeout=0.05
            )
            for task in pending:
                task.cancel()
            await asyncio.sleep(0)
        self._reset_pvat_history()
        self._snapshot = None
        self._latest_snapshot_read = None
        self._snapshot_read_fault = None
        self._motion_observation = None
        self._client = None
        self._motion_accepted = False
        self._motion_write_stop_required = False  # This obligation belongs to the released client.
        self._preflight_ready = False

    async def command_tcp(self, target: Pose, command_id: int) -> None:
        # Preserve the legacy contract used by RobotControl: enqueue first,
        # then return the previous asynchronous constraint as an exception.
        await self.submit_tcp(target, command_id)
        if self._constraint_error is not None:
            raise BackendCommandError(self._constraint_error)

    async def submit_tcp(self, target: Pose, command_id: int) -> CommandAdmission:
        """Opt-in admission receipt; previous feedback does not reject this queue write.

        No await separates permission checks, enqueue and receipt creation.
        Hard failures retain the original exceptions and enqueue nothing.
        """
        self._check_submission_ready()
        admission = self._pump.submit(target, command_id)
        return replace(admission, previous_feedback=self._constraint_feedback)

    async def submit_catch_up(
        self, accepted_command: CommandRef, command_id: int,
    ) -> CommandAdmission:
        """Request one PVAT toward the same accepted solution, without new IK.

        Opt-in only: the owner must supervise VR freshness and stop/reanchor.
        This does not indicate convergence or restore candidate velocity history.
        The reference is scoped to this adapter and checked again at dispatch.
        """
        self._check_submission_ready()
        if (accepted_command is None or accepted_command != self._accepted_target_ref
                or not self._pump.is_current(accepted_command.generation)
                or self._last_sent_tcp is None or self._last_accepted_solution_q is None):
            raise BackendCommandError("catch_up_target_changed")
        admission = self._pump.submit(
            self._last_sent_tcp, command_id, catch_up_from=accepted_command,
        )
        return replace(admission, previous_feedback=self._constraint_feedback)

    def _check_submission_ready(self) -> None:
        self._require_control()
        snapshot = self._snapshot
        if snapshot is None:
            raise BackendCommandError("robot_state_stale")
        max_age_ns = self._command_snapshot_max_age_ns()
        if self._clock() - snapshot.captured_ns > max_age_ns:
            raise BackendCommandError("robot_state_stale")
        if self._pump.fault is not None:
            raise self._pump.fault
        if not self._preflight_ready:
            raise BackendCommandError("preflight_not_ready")
        if not self._pump.running:
            raise BackendCommandError("preflight_not_ready")

    async def set_gripper(self, value: float) -> None:
        self._require_control()
        gripper_generation = self._gripper_revocation_generation
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        if not self._preflight_ready:
            raise BackendCommandError("preflight_not_ready")
        generation = self._pump.generation
        read_floor = self._snapshot_read_sequence
        self._require_gripper_authority()
        snapshot = await self._read_snapshot(read_purpose="gripper")
        self._require_gripper_authority()
        boundary_fault = self._motion_boundary_fault(snapshot)
        if boundary_fault is not None:
            self._preflight_ready = False
            raise BackendCommandError(
                f"preflight_not_ready:{boundary_fault}"
            )
        try:
            normalized = min(1.0, max(0.0, float(value)))
        except (TypeError, ValueError):
            raise BackendCommandError("invalid_gripper_command") from None
        if not math.isfinite(normalized):
            raise BackendCommandError("invalid_gripper_command")
        gripper = self.settings.gripper
        amplitude = round(
            gripper.open_amplitude_percent
            + normalized
            * (
                gripper.closed_amplitude_percent
                - gripper.open_amplitude_percent
            )
        )
        def require_dispatch() -> None:
            # Check both after lock acquisition and inside the scheduled SDK task.
            self._require_control()
            self._require_gripper_authority()
            if (
                self._stop_lock.locked()
                or not self._preflight_ready
                or self._latched_fault is not None
                or gripper_generation != self._gripper_revocation_generation
            ):
                raise BackendCommandError("preflight_not_ready")
            try:
                if not self._pump.is_current(generation):
                    raise BackendCommandError("preflight_not_ready")
                self._check_known_snapshot_fault(client, generation)
                # New health cannot erase a known fault or extend this
                # request's original read lifetime while it waits for the lock.
                self._ensure_command_snapshot_fresh(snapshot)
                current = self._dispatch_snapshot(client, generation, read_floor)
                boundary_fault = self._motion_boundary_fault(current)
                if boundary_fault is not None:
                    raise _MotionSnapshotFault(boundary_fault)
            except _MotionSnapshotFault as error:
                self._preflight_ready = False
                raise BackendCommandError(f"preflight_not_ready:{error}") from None
            except BackendCommandError:
                self._preflight_ready = False
                raise

        async with self._sdk_lock:
            require_dispatch()

            async def send_if_authorized() -> None:
                # Recheck inside the task scheduled by wait_for, before SDK entry.
                require_dispatch()
                try:
                    await client.set_claw(gripper.max_force_percent, amplitude)
                except TimeoutError:
                    raise
                except Exception:
                    raise BackendCommandError("sdk_call_failed:set_claw") from None

            try:
                await asyncio.wait_for(send_if_authorized(), timeout=0.20)
            except TimeoutError:
                raise BackendCommandError("sdk_timeout:set_claw") from None

    async def stop(self, reason: StopReason) -> None:
        self._observer_window.release_all()
        if self.settings.mode == "readonly":
            return
        async with self._stop_lock:
            self._ensure_stop_transaction()
            await self._stop_in_transaction(reason)

    def _ensure_stop_transaction(self) -> None:
        if self._stop_transaction is None:
            self._stop_episode_id += 1
            self._stop_transaction = StopTransaction(
                self._stop_episode_id, self._clock, self._schedule_sdk_diagnostic,
                on_request_started=self._start_stop_diagnostic_sampling,
            )

    def _start_stop_diagnostic_sampling(self, method: str) -> None:
        if (not self._stop_diagnostic_sampling or self._event_callback is None
                or method != self._motion_stop_rpc_method or self._client is None):
            return
        old = self._stop_diagnostic_sampler
        if old is not None:
            if old.episode_id == self._stop_episode_id:
                return
            if old.unresolved:
                self._schedule_sdk_diagnostic({
                    "kind": "stop_diagnostic_reader_skipped",
                    "episode_id": self._stop_episode_id,
                    "reason": "previous_read_pending",
                })
                return
        self._stop_diagnostic_sampler = StopDiagnosticSampler(
            self._client, self._stop_episode_id, self._clock, self._schedule_sdk_diagnostic,
        )
        self._stop_diagnostic_sampler.start()

    async def _stop_in_transaction(self, reason: StopReason) -> None:
        assert self._stop_transaction is not None
        self._gripper_revocation_generation += 1
        diagnostics: dict[str, Any] = {
            "kind": "stop_diagnostics",
            "stop_policy": self._motion_stop_rpc_method + "_only",
            "episode_id": self._stop_transaction.episode_id,
            "reason": reason.value,
            "initial_fault": self._latched_fault,
            "started_ns": self._clock(),
            "stop_entry": {
                "pvat_request_id_in_flight": self._pvat_request_id_in_flight,
                "pvat_pending": self._pump.has_pending,
                "sdk_lock_held": self._sdk_lock.locked(),
            },
            "rpc_calls": [],
            "samples": [],
            "outcome": "failed",
        }
        try:
            await self._stop_verified(reason, diagnostics)
            diagnostics["outcome"] = "confirmed"
        except BaseException as error:
            diagnostics["error"] = str(error)
            diagnostics["error_type"] = type(error).__name__
            raise
        finally:
            diagnostics["completed_ns"] = self._clock()
            diagnostics["latched_fault"] = self._latched_fault
            if self._stop_diagnostic_sampler is not None:
                await self._stop_diagnostic_sampler.close()
            # Never delay the stop on the recorder. Flush only after the
            # safety work, outside the SDK lock, with a bounded wait.
            try:
                await asyncio.wait_for(
                    self._emit_diagnostic_event(diagnostics), timeout=0.05,
                )
            except TimeoutError:
                pass

    async def _call_stop_rpc(
        self, client: LebaiClientProtocol, method: str,
        diagnostics: dict[str, Any],
    ) -> object:
        call: dict[str, Any] = {
            "method": method, "started_ns": self._clock(), "timeout_ms": 200,
        }
        diagnostics["rpc_calls"].append(call)
        try:
            if method in {"stop_move", "stop_sys", "pause_move"}:
                assert self._stop_transaction is not None
                async def send_stop():
                    if method == self._motion_stop_rpc_method:
                        self._motion_write_stop_required = False
                    return await getattr(client, method)()
                result = await self._stop_transaction.call(method, send_stop, call)
            else:
                result = await asyncio.wait_for(getattr(client, method)(), timeout=0.20)
            call["outcome"] = "returned"
            if method == "is_connected":
                call["connected"] = bool(result)
            return result
        except BaseException as error:
            call["outcome"] = (
                "timeout" if isinstance(error, TimeoutError)
                else "cancelled" if isinstance(error, asyncio.CancelledError)
                else "error"
            )
            call["error_type"] = type(error).__name__
            raise
        finally:
            call["completed_ns"] = self._clock()

    async def _stop_verified(
        self, reason: StopReason, diagnostics: dict[str, Any],
    ) -> None:
        method = self._motion_stop_rpc_method
        self._pump.invalidate()
        self._reset_pvat_history()
        self._preflight_ready = False
        client = self._client
        if client is None:
            self._latch_unverified_stop()
            raise BackendCommandError("robot_disconnected")
        try:
            lock_requested_ns = self._clock()
            lock_timing = diagnostics[method + "_lock"] = {
                "requested_ns": lock_requested_ns,
                "acquired_ns": None,
                "wait_ns": None,
                "diagnostic_timing": {
                    "clock": "time.perf_counter_ns", "requested_ns": sample_performance_ns(),
                    "acquired_ns": None,
                },
            }
            try:
                async with self._sdk_lock.stop_priority():
                    lock_timing["diagnostic_timing"]["acquired_ns"] = sample_performance_ns()
                    lock_acquired_ns = self._clock()
                    lock_timing["acquired_ns"] = lock_acquired_ns
                    lock_timing["wait_ns"] = lock_acquired_ns - lock_requested_ns
                    await self._call_stop_rpc(client, method, diagnostics)
            finally:
                lock_timing["diagnostic_timing"]["completed_ns"] = sample_performance_ns()
                if lock_timing["acquired_ns"] is None:
                    lock_timing["wait_ns"] = self._clock() - lock_requested_ns
        except asyncio.CancelledError:
            await asyncio.shield(
                self._fail_unverified_stop(
                    diagnostics=diagnostics,
                    from_cancelled_stop=True,
                )
            )
            raise
        except TimeoutError:
            await self._fail_unverified_stop(diagnostics=diagnostics)
            raise BackendCommandError("sdk_timeout:" + method) from None
        except Exception:
            await self._fail_unverified_stop(diagnostics=diagnostics)
            raise BackendCommandError("sdk_call_failed:" + method) from None
        transaction = self._stop_transaction
        assert transaction is not None
        if transaction.verification_started_ns is None:
            transaction.verification_started_ns = self._clock()
        started_ns = transaction.verification_started_ns
        stable_since_ns = transaction.stable_since_ns
        stable_anchor: LebaiSnapshot | None = transaction.stable_anchor
        try:
            while True:
                snapshot = await self._read_snapshot(
                    emit_kinematics=False, read_purpose="stop_verification",
                )
                now_ns = self._clock()
                stationary = (
                    max(abs(value) for value in snapshot.actual_qd) <= 0.02
                    and (
                        snapshot.raw_robot_state.strip().upper()
                        if isinstance(snapshot.raw_robot_state, str)
                        else snapshot.raw_robot_state
                    ) in _STOP_SETTLED_STATES
                    and snapshot.estop is None
                    and snapshot.running_motion is None
                )
                joint_drift = 0.0
                tcp_drift = 0.0
                if stable_anchor is not None:
                    joint_drift = max(abs(a - b) for a, b in zip(
                        snapshot.actual_q, stable_anchor.actual_q,
                    ))
                    tcp_drift = math.dist(
                        snapshot.actual_tcp.p, stable_anchor.actual_tcp.p,
                    )
                    stationary = (
                        stationary
                        and joint_drift <= _STOP_JOINT_DRIFT_RAD
                        and tcp_drift <= _STOP_TCP_DRIFT_M
                    )
                diagnostics["samples"].append({
                    "captured_ns": snapshot.captured_ns,
                    "read_requested_ns": snapshot.read_requested_ns,
                    "read_completed_ns": snapshot.read_completed_ns,
                    "read_purpose": snapshot.read_purpose,
                    "observed_ns": now_ns,
                    "raw_robot_state": snapshot.raw_robot_state,
                    "estop": snapshot.estop,
                    "running_motion": snapshot.running_motion,
                    "raw_running_motion": snapshot.raw_running_motion,
                    "motion_state": snapshot.motion_state,
                    "actual_q": list(snapshot.actual_q),
                    "actual_qd": list(snapshot.actual_qd),
                    "actual_tcp": snapshot.actual_tcp.model_dump(mode="json"),
                    "stationary": stationary,
                    "joint_drift_rad": joint_drift,
                    "tcp_drift_m": tcp_drift,
                })
                if stationary:
                    if stable_since_ns is None:
                        stable_since_ns = now_ns
                        stable_anchor = snapshot
                        transaction.stable_since_ns = stable_since_ns
                        transaction.stable_anchor = stable_anchor
                    elif (now_ns - stable_since_ns >= 300_000_000
                          and (transaction.confirmed or now_ns - started_ns < 500_000_000)):
                        if 'move_pvat' in self._motion_calls.unknown_methods:
                            await self._fail_unverified_stop(diagnostics=diagnostics)
                            raise BackendCommandError("stop_unverified")
                        self._motion_accepted = False
                        self._motion_write_stop_required = False
                        transaction.confirmed = True
                        return
                else:
                    stable_since_ns = None
                    stable_anchor = None
                    transaction.stable_since_ns = None
                    transaction.stable_anchor = None
                    transaction.confirmed = False
                if now_ns - started_ns >= 500_000_000:
                    self._latched_fault = "stop_incomplete"
                    await self._fail_unverified_stop(
                        diagnostics=diagnostics,
                        preserve_fault=True,
                    )
                    raise BackendCommandError("stop_incomplete")
                await self._sleep(0.02)
        except asyncio.CancelledError:
            await asyncio.shield(
                self._fail_unverified_stop(
                    diagnostics=diagnostics,
                    from_cancelled_stop=True,
                )
            )
            raise
        except BackendCommandError as error:
            if str(error) == "stop_incomplete":
                raise
            await self._fail_unverified_stop(diagnostics=diagnostics)
            if str(error) == "robot_disconnected":
                raise BackendCommandError(
                    "stop_unverified_disconnected"
                ) from None
            raise
        except Exception:
            await self._fail_unverified_stop(diagnostics=diagnostics)
            raise BackendCommandError("stop_unverified") from None

    def _latch_unverified_stop(
        self,
        *,
        from_cancelled_stop: bool = False,
    ) -> None:
        self._observer_window.release_all()
        if self._latched_fault not in {"stop_unverified", "stop_incomplete"}:
            self._latched_fault = "stop_unverified"
        self._latched_fault_from_cancelled_stop = from_cancelled_stop
        self._motion_accepted = False
        self._preflight_ready = False
        self._pump.invalidate()
        self._reset_pvat_history()

    async def _fail_unverified_stop(
        self,
        *,
        diagnostics: dict[str, Any],
        preserve_fault: bool = False,
        from_cancelled_stop: bool = False,
    ) -> None:
        self._observer_window.release_all()
        if not preserve_fault:
            self._latch_unverified_stop(
                from_cancelled_stop=from_cancelled_stop,
            )
        else:
            self._motion_accepted = False
            self._preflight_ready = False
            self._pump.invalidate()
            self._reset_pvat_history()

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        self._require_control()
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        snapshot = await self._read_snapshot(read_purpose="home")
        preflight = self._preflight_from_snapshot(
            snapshot,
            allow_singular=True,
        )
        if not preflight.ready:
            raise BackendCommandError(
                f"preflight_not_ready:{preflight.reason}"
            )
        await self._move_to_joint_pose(
            self.settings.home_q,
            options,
            on_phase,
            failure="home_failed",
        )

    async def prepare(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        self._require_control()
        if self._client is None:
            raise BackendCommandError("robot_disconnected")
        snapshot = await self._read_snapshot(read_purpose="prepare")
        preflight = self._preflight_from_snapshot(
            snapshot,
            allow_singular=True,
        )
        if not preflight.ready:
            raise BackendCommandError(
                f"preflight_not_ready:{preflight.reason}"
            )
        await self._move_to_joint_pose(
            self.settings.teleop_ready_q,
            options,
            on_phase,
            failure="prepare_failed",
        )

    async def _move_to_joint_pose(
        self,
        target_q: JointVector,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
        *,
        failure: str,
    ) -> None:
        client = self._client
        owner = self._snapshot_calls
        if client is None:
            raise BackendCommandError("robot_disconnected")
        if self._pump.has_pending:
            raise BackendCommandError("home_command_pending")
        sdk_entered = False
        prior_motion_needs_stop = self._motion_accepted or self._motion_write_stop_required
        self._preflight_ready = False
        self._pump.invalidate()
        generation = self._pump.generation
        stop_generation = self._gripper_revocation_generation
        self._reset_pvat_history()
        on_phase("homing")
        lifecycle: dict[str, object] = {}
        def require_current():
            if (client is not self._client or owner is not self._snapshot_calls or owner.closed
                    or self._disconnecting or self._stop_lock.locked()
                    or generation != self._pump.generation
                    or stop_generation != self._gripper_revocation_generation):
                raise _RecoveryRevoked('recovery_request_revoked')
        async def stop_owned():
            # Check after taking the stop lock: a disconnect can replace the
            # client while this request waits for an existing stop transaction.
            async with self._stop_lock:
                if (client is not self._client or owner is not self._snapshot_calls
                        or owner.closed):
                    return
                self._observer_window.release_all()
                self._ensure_stop_transaction()
                await self._stop_in_transaction(StopReason.HOME)
        try:
            async with self._sdk_lock:
                require_current()
                self._begin_motion_epoch()
                speed = min(options.max_speed_radps, self.settings.control.max_joint_speed_radps)
                acceleration = self.settings.control.max_joint_acceleration_radps2
                self._sdk_request_id += 1
                lifecycle.update({
                    'kind': 'sdk_request_lifecycle', 'method': 'movej',
                    'request_id': self._sdk_request_id, 'failure_code': failure,
                    'started_ns': self._clock(), 'rpc_outcome': 'started',
                    'p': list(target_q), 'a': acceleration, 'v': speed, 't': 0., 'r': 0.,
                    'evidence_scope': 'local_sdk_call_not_remote_execution',
                })
                try:
                    async def send_recovery_if_current():
                        nonlocal sdk_entered
                        require_current()
                        self._begin_motion_epoch()
                        sdk_entered = True
                        self._motion_accepted = True
                        return await client.movej(list(target_q), acceleration, speed, 0., 0.)
                    motion_id = await asyncio.wait_for(
                        send_recovery_if_current(),
                        timeout=0.20,
                    )
                    lifecycle.update(rpc_outcome='returned', motion_id=motion_id)
                except _RecoveryRevoked:
                    lifecycle.update(rpc_outcome='not_started', revocation_reason='recovery_request_revoked')
                    raise
                except asyncio.CancelledError:
                    lifecycle['rpc_outcome'] = 'cancelled'
                    raise
                except TimeoutError:
                    lifecycle['rpc_outcome'] = 'timeout'
                    raise
                except Exception as error:
                    lifecycle.update(rpc_outcome='error', error_type=type(error).__name__)
                    raise
                finally:
                    lifecycle['completed_ns'] = self._clock()
        except asyncio.CancelledError:
            if sdk_entered or prior_motion_needs_stop:
                await asyncio.shield(stop_owned())
            raise
        except _RecoveryRevoked:
            raise
        except TimeoutError:
            await stop_owned()
            raise BackendCommandError("sdk_timeout:movej") from None
        except Exception:
            await stop_owned()
            raise BackendCommandError("sdk_call_failed:movej") from None
        finally:
            # Existing bounded best-effort recording. No logging await before
            # the error's original stop path; RPC timings exclude cleanup time.
            if lifecycle:
                self._schedule_sdk_diagnostic(lifecycle)
        started_ns = self._clock()
        stable_since_ns: int | None = None
        stabilizing = False
        while True:
            require_current()
            snapshot = await self._read_snapshot(read_purpose="joint_move")
            require_current()
            async with self._sdk_lock:
                require_current()
                try:
                    async def read_recovery_state_if_current():
                        require_current()
                        return await client.get_motion_state(motion_id)
                    motion_state = await asyncio.wait_for(
                        read_recovery_state_if_current(),
                        timeout=0.20,
                    )
                    require_current()
                except _RecoveryRevoked:
                    raise
                except TimeoutError:
                    raise BackendCommandError(
                        "sdk_timeout:get_motion_state"
                    ) from None
                except Exception:
                    raise BackendCommandError(
                        "sdk_call_failed:get_motion_state"
                    ) from None
            if str(motion_state).upper() in {"ERROR", "FAILED", "CANCELLED"}:
                raise BackendCommandError(failure)
            now_ns = self._clock()
            position_error = max(
                abs(actual - target)
                for actual, target in zip(
                    snapshot.actual_q,
                    target_q,
                    strict=True,
                )
            )
            velocity = max(abs(value) for value in snapshot.actual_qd)
            within_tolerance = (
                position_error <= options.position_tolerance_rad
                and velocity <= options.velocity_tolerance_radps
            )
            if within_tolerance:
                if stable_since_ns is None:
                    stable_since_ns = now_ns
                    stabilizing = True
                    on_phase("stabilizing")
                elif (
                    now_ns - stable_since_ns
                    >= int(options.stable_seconds * 1_000_000_000)
                ):
                    self._motion_accepted = False
                    return
            else:
                stable_since_ns = None
                if stabilizing:
                    stabilizing = False
                    on_phase("homing")
            if now_ns - started_ns >= int(options.timeout_s * 1_000_000_000):
                await stop_owned()
                raise BackendCommandError("home_timeout")
            await self._sleep(0.02)

    async def get_state(self) -> RobotStateMessage:
        if self._pump.fault is not None and self._latched_fault is None:
            raise self._pump.fault
        observer_timing = None
        if (self._observer_window.max_wait_s > 0 and self._latched_fault is None
                and not self._disconnecting and not self._stop_lock.locked()):
            observer_timing = {
                "clock": "time.perf_counter_ns", "alignment_verified": False,
                "budget_s": self._observer_window.max_wait_s,
                "requested_ns": sample_performance_ns(),
            }
            await self._observer_window.wait()
            observer_timing["completed_ns"] = sample_performance_ns()
            if self._pump.fault is not None and self._latched_fault is None:
                raise self._pump.fault
        snapshot = await self._read_snapshot(
            read_purpose="state", observer_timing=observer_timing,
        )
        robot_state = (
            BackendState.FAULT
            if self._latched_fault is not None
            else snapshot.robot_state
        )
        return RobotStateMessage(
            server_mono_ns=snapshot.captured_ns,
            ack_seq=self._command_id,
            mode=TeleopMode.READY,
            robot_state=robot_state,
            actual_tcp=snapshot.actual_tcp,
            actual_q=snapshot.actual_q,
            gripper=snapshot.gripper,
            sample_age_ms=0.0,
            fault=self._latched_fault or snapshot.estop,
            backend=self._backend_label,
            real_robot_mode=self.settings.mode,
        )

    async def preflight(self) -> BackendPreflight:
        snapshot = await self._read_snapshot(read_purpose="preflight")
        result = self._preflight_from_snapshot(snapshot)
        self._preflight_ready = result.ready
        if (
            result.ready
            and self.settings.mode == "control"
            and not self._pump.running
            and self._pump.fault is None
        ):
            await self._pump.start()
        return result

    def _preflight_from_snapshot(
        self,
        snapshot: LebaiSnapshot,
        *,
        allow_singular: bool = False,
    ) -> BackendPreflight:
        tcp_matches = self._tcp_matches(snapshot.tcp_setting)
        reason: str | None = None
        if self._latched_fault is not None:
            reason = self._latched_fault
        elif self._motion_calls.unknown_methods:
            reason = "motion_rpc_result_unknown"
        elif self._snapshot_calls.unknown_methods:
            reason = "snapshot_rpc_result_unknown"
        elif snapshot.robot_state is not BackendState.IDLE:
            reason = "robot_not_idle"
        elif snapshot.running_motion is not None:
            reason = "motion_running"
        elif snapshot.estop is not None:
            reason = snapshot.estop
        elif not tcp_matches:
            reason = "tcp_mismatch"
        elif not self._joints_inside_limits(snapshot.actual_q):
            reason = "joint_outside_soft_limits"
        elif not self._tcp_inside_startup_envelope(snapshot.actual_tcp):
            reason = "tcp_outside_startup_envelope"
        elif (
            self.settings.mode == "control"
            and not self._capabilities.control_ready
        ):
            reason = "sdk_capability_missing"
        elif (
            self.settings.mode == "control"
            and not allow_singular
            and self._cartesian_singularity_guarded(snapshot.actual_q)
        ):
            reason = "singular_configuration"
        elif self.settings.mode == "readonly":
            reason = "real_robot_readonly"
        return BackendPreflight(
            ready=reason is None,
            reason=reason,
            robot_state=snapshot.robot_state,
            actual_tcp=snapshot.actual_tcp,
            actual_q=snapshot.actual_q,
            tcp_matches=tcp_matches,
            capabilities=self._capabilities.names,
        )

    @staticmethod
    def _cartesian_singularity_guarded(actual_q: JointVector) -> bool:
        return (
            abs(actual_q[2]) <= TELEOP_SINGULARITY_GUARD_RAD
            or abs(actual_q[4]) <= TELEOP_SINGULARITY_GUARD_RAD
        )

    def _ensure_command_snapshot_fresh(
        self,
        snapshot: LebaiSnapshot,
    ) -> None:
        if (
            self._clock() - snapshot.captured_ns
            > self._command_snapshot_max_age_ns()
        ):
            raise BackendCommandError("robot_state_stale")

    async def _send_target(self, request: PvatRequest) -> None:
        lifecycle: dict[str, object] = {}
        error_code: str | None = None
        cancelled = False
        observer_lease = (
            self._observer_window.begin()
            if (self._pump.is_current(request.generation)
                and self._latched_fault is None and not self._disconnecting
                and not self._stop_lock.locked()) else None
        )
        try:
            try:
                await self._send_target_impl(request, lifecycle)
            except _MotionSnapshotFault as error:
                await self._stop_for_motion_fault(str(error))
        except asyncio.CancelledError:
            cancelled = True
            raise
        except BackendCommandError as error:
            error_code = str(error)
            raise
        except Exception:
            error_code = "pvat_pump_failed"
            raise
        finally:
            self._observer_window.finish(observer_lease)
            if request.completion is not None:
                request.completion._resolve(
                    "cancelled" if cancelled else "failed" if error_code else
                    "invalidated" if request.completion.invalidated else "handled",
                    sdk_outcome=lifecycle.get("rpc_outcome", "not_started"),
                    sdk_request_id=lifecycle.get("request_id"),
                    error=error_code,
                )
            # The implementation has released the SDK lock before recording,
            # including early returns after generation invalidation.
            if lifecycle:
                self._schedule_sdk_diagnostic(lifecycle)

    def _schedule_sdk_diagnostic(self, event: dict[str, object]) -> None:
        # Never wait for logging before publishing a motion error. Even a
        # callback that ignores cancellation cannot create unbounded tasks.
        if len(self._sdk_diagnostic_tasks) >= 16:
            self._sdk_diagnostics_dropped += 1
            return
        event["diagnostics_dropped_total"] = self._sdk_diagnostics_dropped
        task = asyncio.create_task(self._emit_diagnostic_event(event))
        self._sdk_diagnostic_tasks.add(task)
        timer = asyncio.get_running_loop().call_later(0.05, task.cancel)

        def completed(done: asyncio.Task[None]) -> None:
            timer.cancel()
            self._sdk_diagnostic_tasks.discard(done)
            if not done.cancelled():
                done.exception()

        task.add_done_callback(completed)

    async def _send_target_impl(
        self, request: PvatRequest, lifecycle: dict[str, object]
    ) -> None:
        if not self._motion_dispatch_is_current(request):
            return
        handler_started_ns = self._clock()
        read_floor = self._snapshot_read_sequence
        client = self._client
        performance_timing: dict[str, object] = {
            "clock": "time.perf_counter_ns", "alignment_verified": False,
            "handler_started_ns": sample_performance_ns(),
        }
        snapshot = await self._read_snapshot(read_purpose="motion")
        motion_snapshot = snapshot
        performance_timing["snapshot_finished_ns"] = sample_performance_ns()
        performance_timing["snapshot_timing"] = copy.deepcopy(snapshot.diagnostic_timing or {})
        snapshot_finished_ns = self._clock()
        if not self._motion_dispatch_is_current(request):
            return
        self._check_known_snapshot_fault(client, request.generation)
        runtime_fault = self._runtime_motion_fault(snapshot)
        if runtime_fault is not None:
            await self._stop_for_motion_fault(runtime_fault)
        self._ensure_command_snapshot_fresh(snapshot)
        accepted: tuple[_SolvedCandidate, float] | None = None
        previous_pvat_gap_ms: float | None = None
        pvat_reference_started_ns: int | None = None
        last_recoverable_error: str | None = None
        diagnostic_events: list[dict[str, object]] = []
        lock_requested_ns = self._clock()
        performance_timing["command_lock_requested_ns"] = sample_performance_ns()
        async with self._sdk_lock:
            lock_acquired_ns = self._clock()
            performance_timing["command_lock_acquired_ns"] = sample_performance_ns()
            selected: _SolvedCandidate | None = None
            if not self._motion_dispatch_is_current(request):
                return
            self._check_known_snapshot_fault(client, request.generation)
            # A newer observation supplies current inputs, not permission to
            # extend the lifetime of this motion request's original read.
            self._ensure_command_snapshot_fresh(motion_snapshot)
            snapshot = self._dispatch_snapshot(client, request.generation, read_floor)
            performance_timing["dispatch_snapshot_timing"] = copy.deepcopy(snapshot.diagnostic_timing or {})
            performance_timing["dispatch_snapshot_read_purpose"] = snapshot.read_purpose
            performance_timing["dispatch_snapshot_changed"] = snapshot is not motion_snapshot
            if request.catch_up_from is not None:
                # An earlier in-flight advance can change acceptance after
                # admission. Never silently pursue that different target.
                if request.catch_up_from != self._accepted_target_ref:
                    if request.completion is not None:
                        request.completion._invalidate()
                        request.completion._resolve(
                            "invalidated", error="catch_up_target_changed",
                            accepted_command=self._accepted_target_ref,
                        )
                    return
                self._ensure_command_snapshot_fresh(snapshot)
                selected = self._build_catch_up_candidate(snapshot, request)
            elif self._backend_label == "LEBAI_FAKE":
                for fraction, candidate in recovery_candidates(
                    self._last_sent_tcp,
                    request.target,
                    fake=True,
                ):
                    self._ensure_command_snapshot_fresh(snapshot)
                    try:
                        selected = await self._solve_candidate(
                            client,
                            snapshot,
                            candidate,
                            fraction=fraction,
                            command_id=request.command_id,
                            diagnostic_events=diagnostic_events,
                        )
                    except BackendCommandError as error:
                        if not self._pump.is_current(request.generation):
                            return
                        if str(error) not in RECOVERABLE_IK_ERRORS:
                            raise
                        last_recoverable_error = str(error)
                        continue
                    break
            else:
                try:
                    selected = await self._select_advancing_candidate(
                        client,
                        snapshot,
                        request,
                        diagnostic_events=diagnostic_events,
                    )
                except BackendCommandError as error:
                    if str(error) not in RECOVERABLE_IK_ERRORS:
                        raise
                    last_recoverable_error = str(error)
            if not self._motion_dispatch_is_current(request):
                return
            if selected is not None:
                if not self._pump.is_current(request.generation):
                    return
                self._check_known_snapshot_fault(client, request.generation)
                self._ensure_command_snapshot_fresh(motion_snapshot)
                self._ensure_command_snapshot_fresh(snapshot)
                pvat_started = self._clock()
                performance_timing["candidate_finished_ns"] = sample_performance_ns()
                if self._last_pvat_started_ns is not None:
                    previous_pvat_gap_ms = (
                        pvat_started - self._last_pvat_started_ns
                    ) / 1_000_000
                self._sdk_request_id += 1
                # Local call evidence, not an acknowledgement of remote execution.
                lifecycle.update({
                    "kind": "sdk_request_lifecycle",
                    "diagnostic_timing": performance_timing,
                    "method": "move_pvat",
                    "request_id": self._sdk_request_id,
                    "command_id": request.command_id,
                    "generation": request.generation,
                    "submission_id": request.submission_id,
                    "started_ns": pvat_started,
                    "rpc_outcome": "started",
                    "p": list(selected.point.q),
                    "v": list(selected.point.qd),
                    "a": list(selected.point.qdd),
                    "horizon_s": selected.point.horizon_s,
                    "point_inputs": _pvat_point_inputs(snapshot, selected.point),
                })
                self._pvat_request_id_in_flight = self._sdk_request_id
                sdk_entered = False

                async def send_if_authorized() -> bool:
                    nonlocal selected, pvat_reference_started_ns, sdk_entered
                    # wait_for schedules another task: recheck inside that task,
                    # immediately before SDK entry, with no intervening await.
                    if not self._motion_dispatch_is_current(request):
                        return False
                    self._check_known_snapshot_fault(client, request.generation)
                    self._ensure_command_snapshot_fresh(motion_snapshot)
                    self._ensure_command_snapshot_fresh(snapshot)
                    pvat_reference_started_ns = self._pvat_reference_now()
                    reference_rebuilt = False
                    if (
                        selected.point.reference_source == "previous_command_velocity"
                        and self._pvat_velocity_reference_at(pvat_reference_started_ns) is None
                    ):
                        # Selection and wait_for task scheduling can outlive the
                        # previous point. Rebuild from the same admitted snapshot
                        # and IK solution, with the original limits and no I/O.
                        selected = replace(selected, point=build_pvat_point(
                            solution_q=selected.metrics.solution_q,
                            actual_q=snapshot.actual_q,
                            actual_qd=snapshot.actual_qd,
                            previous_qd=None,
                            limits=self._pvat_limits,
                        ))
                        reference_rebuilt = True
                        lifecycle.update(
                            p=list(selected.point.q), v=list(selected.point.qd),
                            a=list(selected.point.qdd),
                            point_inputs=_pvat_point_inputs(snapshot, selected.point),
                        )
                    lifecycle["pvat_reference_timing"] = {
                        "clock": "time.perf_counter_ns" if self._pvat_clock is time.perf_counter_ns else "injected",
                        "reference_check_ns": pvat_reference_started_ns,
                        "previous_reference_check_ns": self._last_pvat_clock_started_ns,
                        "rebuilt_at_sdk_entry": reference_rebuilt,
                    }
                    if not self._motion_dispatch_is_current(request):
                        return False
                    # A synchronous rebuild can also consume the snapshot's
                    # remaining lifetime. Retain the original read's limit.
                    self._check_known_snapshot_fault(client, request.generation)
                    self._ensure_command_snapshot_fresh(motion_snapshot)
                    self._ensure_command_snapshot_fresh(snapshot)
                    self._begin_motion_epoch()
                    sdk_entered = True
                    self._motion_write_stop_required = True
                    await client.move_pvat(
                        list(selected.point.q), list(selected.point.qd),
                        list(selected.point.qdd), selected.point.horizon_s,
                    )
                    return True

                try:
                    performance_timing["sdk_await_started_ns"] = sample_performance_ns()
                    sent = await self._await_motion_call(
                        "move_pvat", send_if_authorized, .06,
                        generation=request.generation, entered=lambda: sdk_entered,
                    )
                    if not sent:
                        lifecycle["rpc_outcome"] = "not_started"
                        lifecycle["revocation_reason"] = (
                            "pump_generation_revoked"
                            if not self._pump.is_current(request.generation)
                            else "teleop_motion_authority_revoked"
                        )
                        return
                    lifecycle["rpc_outcome"] = "returned"
                except asyncio.CancelledError:
                    lifecycle["rpc_outcome"] = "cancelled"
                    raise
                except TimeoutError:
                    lifecycle["rpc_outcome"] = "timeout"
                    if not self._pump.is_current(request.generation):
                        return
                    raise BackendCommandError("sdk_timeout:move_pvat") from None
                except BackendCommandError as error:
                    if not sdk_entered:
                        lifecycle["rpc_outcome"] = "not_started"
                        lifecycle["revocation_reason"] = str(error)
                        # Preserve snapshot fault identity so the wrapper uses
                        # the original stop path after releasing the SDK lock.
                        raise
                    lifecycle["rpc_outcome"] = "error"
                    lifecycle["error_type"] = type(error).__name__
                    if not self._pump.is_current(request.generation):
                        return
                    raise BackendCommandError("sdk_call_failed:move_pvat") from None
                except Exception as error:
                    lifecycle["rpc_outcome"] = "error"
                    lifecycle["error_type"] = type(error).__name__
                    if not self._pump.is_current(request.generation):
                        return
                    raise BackendCommandError(
                        "sdk_call_failed:move_pvat"
                    ) from None
                finally:
                    performance_timing["sdk_await_completed_ns"] = sample_performance_ns()
                    self._pvat_request_id_in_flight = None
                    lifecycle["completed_ns"] = self._clock()
                    lifecycle["invalidated"] = (
                        not self._pump.is_current(request.generation)
                        or (request.completion is not None and request.completion.invalidated)
                    )
                pvat_latency_ms = max(
                    0.0,
                    (self._clock() - pvat_started) / 1_000_000,
                )
                send_finished_ns = self._clock()
                if not self._motion_dispatch_is_current(
                    request, sdk_outcome="returned", sdk_request_id=self._sdk_request_id,
                ):
                    lifecycle["invalidated"] = True
                    lifecycle["revocation_reason"] = (
                        "pump_generation_revoked"
                        if not self._pump.is_current(request.generation)
                        else "teleop_motion_authority_revoked"
                    )
                    return
                if selected.pvat_mode == "catch_up":
                    self._constraint = "tracking_lag"
                    self._constraint_error = "ik_tracking_lag"
                    self._constraint_feedback = CommandFeedback(
                        request.reference, "ik_tracking_lag", "catch_up",
                    )
                elif selected.recovery_fraction == 1.0:
                    self._constraint = None
                    self._constraint_error = None
                    self._constraint_feedback = None
                    self._consecutive_ik_failures = 0
                else:
                    self._note_soft_constraint(
                        last_recoverable_error or "ik_unreachable",
                        persistent=False,
                        command=request.reference,
                        outcome="interpolated_advance",
                    )
                self._previous_sent_qd = selected.point.qd
                self._last_pvat_started_ns = pvat_started
                self._last_pvat_clock_started_ns = pvat_reference_started_ns
                if selected.advances_target:
                    self._last_sent_tcp = selected.target.model_copy(
                        deep=True
                    )
                    self._last_accepted_solution_q = (
                        selected.metrics.solution_q
                    )
                    self._accepted_target_command_id = request.command_id
                    self._accepted_target_ref = request.reference
                self._command_id = request.command_id
                self._motion_accepted = True
                accepted = (selected, pvat_latency_ms)
        if accepted is None:
            try:
                self._note_soft_constraint(
                    last_recoverable_error or "ik_unreachable",
                    persistent=self._backend_label != "LEBAI_FAKE",
                    command=request.reference,
                )
            except BackendCommandError as error:
                if request.completion is not None:
                    request.completion._resolve(
                        "ik_rejected", error=str(error),
                        accepted_command=self._accepted_target_ref,
                    )
                await self._emit_candidate_diagnostics(diagnostic_events, request)
                if not self._pump.is_current(request.generation):
                    return
                raise
            if request.completion is not None:
                request.completion._resolve(
                    "ik_rejected", error=self._constraint_error,
                    accepted_command=self._accepted_target_ref,
                )
            await self._emit_candidate_diagnostics(diagnostic_events, request)
            return
        selected, pvat_latency_ms = accepted
        # Dispatch result is available before potentially blocking/failing
        # recording. Later recorder failure still follows the existing fault path.
        if request.completion is not None:
            request.completion._resolve(
                "sdk_returned", sdk_outcome="returned",
                sdk_request_id=lifecycle.get("request_id"),
                accepted_command=self._accepted_target_ref,
                pvat_mode=selected.pvat_mode,
            )
        await self._emit_event(
            {
                "kind": "pvat_sent",
                "request_kind": "catch_up" if request.catch_up_from is not None else "target",
                "generation": request.generation,
                "submission_id": request.submission_id,
                "accepted_target_generation": (
                    self._accepted_target_ref.generation if self._accepted_target_ref else None
                ),
                "accepted_target_submission_id": (
                    self._accepted_target_ref.submission_id if self._accepted_target_ref else None
                ),
                "previous_pvat_gap_ms": previous_pvat_gap_ms,
                "pvat_reference_timing": copy.deepcopy(lifecycle.get("pvat_reference_timing")),
                "timing_ms": {
                    "snapshot_read": (snapshot_finished_ns - handler_started_ns) / 1_000_000,
                    "snapshot_lock_wait": motion_snapshot.sdk_lock_wait_ms,
                    "command_lock_wait": (lock_acquired_ns - lock_requested_ns) / 1_000_000,
                    "candidate_selection": (pvat_started - lock_acquired_ns) / 1_000_000,
                    "send_sdk": pvat_latency_ms,
                    "handler_to_send_complete": (send_finished_ns - handler_started_ns) / 1_000_000,
                    "snapshot_age_at_send": (pvat_started - snapshot.captured_ns) / 1_000_000,
                    "motion_snapshot_age_at_send": (pvat_started - motion_snapshot.captured_ns) / 1_000_000,
                },
                "diagnostic_timing": {
                    **performance_timing,
                    "snapshot_timing": copy.deepcopy(motion_snapshot.diagnostic_timing or {}),
                },
                "command_id": request.command_id,
                "requested_tcp": request.target.model_dump(),
                "target_tcp": selected.target.model_dump(),
                "recovery_fraction": selected.recovery_fraction,
                "ik_solution_q": list(selected.metrics.solution_q),
                "solution_step_rad": selected.metrics.solution_step_rad,
                "tracking_error_rad": selected.metrics.tracking_error_rad,
                "pvat_mode": selected.pvat_mode,
                "accepted_target_command_id": (
                    self._accepted_target_command_id
                ),
                "p": list(selected.point.q),
                "v": list(selected.point.qd),
                "a": list(selected.point.qdd),
                "horizon_s": selected.point.horizon_s,
                "sdk_latency_ms": pvat_latency_ms,
                "point_inputs": _pvat_point_inputs(snapshot, selected.point),
            }
        )
        await self._emit_candidate_diagnostics(diagnostic_events, request)

    async def _stop_for_motion_fault(self, fault: str) -> None:
        # Must be called outside the SDK lock: stop acquires that same lock.
        self._latched_fault = fault
        self._preflight_ready = False
        self._motion_accepted = False
        self._pump.invalidate()
        self._reset_pvat_history()
        try:
            await self.stop(StopReason.FAULT)
        except asyncio.CancelledError:
            raise
        except Exception:
            if self._latched_fault == fault:
                self._latched_fault = "stop_unverified"
        if self._latched_fault == fault:
            raise BackendCommandError(fault)
        raise BackendCommandError(self._latched_fault or "stop_unverified")

    def _remember_snapshot_fault(self, read: _SnapshotRead, fault: str) -> None:
        if (self._pump.running and read.sequence and read.client is self._client
                and read.owner is self._snapshot_calls
                and read.generation == self._pump.generation):
            previous = self._snapshot_read_fault
            # Keep the first known fault for this prepared generation. A read
            # already in flight when motion enters also belongs to this fence;
            # later health or recorder errors cannot erase its source.
            if (previous is None or previous[0].client is not read.client
                    or previous[0].generation != read.generation):
                self._snapshot_read_fault = (read, fault)

    def _check_known_snapshot_fault(
        self, client: LebaiClientProtocol | None, generation: int,
    ) -> None:
        if client is None or client is not self._client:
            raise BackendCommandError("robot_disconnected")
        fault = self._snapshot_read_fault
        if (fault is not None and fault[0].client is client
                and fault[0].generation == generation):
            raise _MotionSnapshotFault(fault[1])

    def _dispatch_snapshot(
        self, client: LebaiClientProtocol, generation: int, read_floor: int,
    ) -> LebaiSnapshot:
        self._check_known_snapshot_fault(client, generation)
        read = self._latest_snapshot_read
        if (read is None or read.client is not client or read.generation != generation
                or read.sequence <= read_floor or read.snapshot is None):
            raise _MotionSnapshotFault("robot_state_stale")
        self._ensure_command_snapshot_fresh(read.snapshot)
        fault_code = self._runtime_motion_fault(read.snapshot)
        if fault_code is not None:
            raise _MotionSnapshotFault(fault_code)
        if not self._tcp_matches(read.snapshot.tcp_setting):
            raise _MotionSnapshotFault("tcp_mismatch")
        return read.snapshot

    async def _emit_candidate_diagnostics(
        self, events: list[dict[str, object]], request: PvatRequest,
    ) -> None:
        for event in events:
            event["generation"] = request.generation
            event["submission_id"] = request.submission_id
            await self._emit_diagnostic_event(event)

    async def _solve_candidate(
        self,
        client: LebaiClientProtocol,
        snapshot: LebaiSnapshot,
        target: Pose,
        *,
        fraction: float,
        command_id: int,
        diagnostic_events: list[dict[str, object]],
    ) -> _SolvedCandidate:
        solution = await self._inverse_kinematics(client, snapshot, target)
        metrics = evaluate_ik_candidate(
            solution,
            snapshot.actual_q,
            previous_solution_q=self._last_accepted_solution_q,
        )
        try:
            return self._build_advancing_candidate(
                snapshot,
                target,
                metrics,
                fraction,
            )
        except BackendCommandError as error:
            if str(error) in {
                "ik_joint_limit",
                "ik_joint_jump",
                "ik_tracking_diverged",
                "joint_speed_limit",
            }:
                solution_q = metrics.solution_q
                delta_q = tuple(
                    solved - actual
                    for solved, actual in zip(
                        solution_q,
                        snapshot.actual_q,
                        strict=True,
                    )
                )
                target_rotation_delta = (
                    Rotation.from_quat(target.q)
                    * Rotation.from_quat(snapshot.actual_tcp.q).inv()
                )
                prior_pvat_sent = self._last_sent_tcp is not None
                diagnostic_events.append(
                    {
                        "kind": "ik_candidate_rejected",
                        "reason": str(error),
                        "command_id": command_id,
                        "target_translation_delta_m": math.dist(
                            target.p,
                            snapshot.actual_tcp.p,
                        ),
                        "target_rotation_delta_deg": math.degrees(
                            target_rotation_delta.magnitude()
                        ),
                        "actual_q": list(snapshot.actual_q),
                        "solution_q": list(solution_q),
                        "delta_q": list(delta_q),
                        "max_abs_delta_q": max(
                            abs(component) for component in delta_q
                        ),
                        "max_joint_step_rad": (
                            self.settings.control.max_joint_step_rad
                        ),
                        "prior_pvat_sent": prior_pvat_sent,
                        "previous_command_id": (
                            self._accepted_target_command_id
                            if prior_pvat_sent
                            else None
                        ),
                    }
                )
            raise _CandidateRejected(str(error), metrics) from None

    async def _select_advancing_candidate(
        self,
        client: LebaiClientProtocol,
        snapshot: LebaiSnapshot,
        request: PvatRequest,
        *,
        diagnostic_events: list[dict[str, object]],
    ) -> _SolvedCandidate | None:
        fraction = 1.0
        for attempt in range(3):
            if not self._pump.is_current(request.generation):
                return None
            if fraction == 1.0:
                target = request.target
            else:
                recovery_start = self._last_sent_tcp
                if recovery_start is None:
                    raise BackendCommandError("ik_joint_jump")
                target = interpolate_pose(
                    recovery_start,
                    request.target,
                    fraction,
                )
            try:
                selected = await self._solve_candidate(
                    client,
                    snapshot,
                    target,
                    fraction=fraction,
                    command_id=request.command_id,
                    diagnostic_events=diagnostic_events,
                )
            except _CandidateRejected as error:
                if not self._pump.is_current(request.generation):
                    return None
                if (
                    str(error) == "ik_tracking_diverged"
                    and self._last_accepted_solution_q is not None
                    and self._last_sent_tcp is not None
                ):
                    diagnostic_events.append(
                        {
                            "kind": "ik_tracking_backpressure",
                            "command_id": request.command_id,
                            "actual_q": list(snapshot.actual_q),
                            "accepted_solution_q": list(
                                self._last_accepted_solution_q
                            ),
                            "requested_solution_q": list(
                                error.metrics.solution_q
                            ),
                            "tracking_error_rad": (
                                error.metrics.tracking_error_rad
                            ),
                            "max_joint_tracking_error_rad": (
                                self.settings.control
                                .max_joint_tracking_error_rad
                            ),
                            "pvat_mode": "catch_up",
                            "accepted_target_command_id": (
                                self._accepted_target_command_id
                            ),
                        }
                    )
                    return self._build_catch_up_candidate(snapshot, request)
                if (
                    str(error) != "ik_joint_jump"
                    or attempt == 2
                    or self._last_sent_tcp is None
                ):
                    raise
                fraction = proportional_recovery_fraction(
                    error.metrics.solution_step_rad,
                    self.settings.control.max_joint_step_rad,
                    previous_fraction=None if attempt == 0 else fraction,
                )
                continue
            except BackendCommandError:
                if not self._pump.is_current(request.generation):
                    return None
                raise
            if not self._pump.is_current(request.generation):
                return None
            return selected
        raise AssertionError("unreachable_recovery_loop")

    def _build_catch_up_candidate(
        self,
        snapshot: LebaiSnapshot,
        request: PvatRequest,
    ) -> _SolvedCandidate:
        solution = self._last_accepted_solution_q
        target = self._last_sent_tcp
        if solution is None or target is None:
            raise BackendCommandError("ik_joint_jump")
        metrics = evaluate_ik_candidate(
            solution,
            snapshot.actual_q,
            previous_solution_q=solution,
        )
        if (
            metrics.tracking_error_rad
            > self.settings.control.max_joint_tracking_error_rad
        ):
            raise BackendCommandError("ik_tracking_diverged")
        point = build_pvat_point(
            solution_q=solution,
            actual_q=snapshot.actual_q,
            actual_qd=snapshot.actual_qd,
            previous_qd=self._continuous_pvat_velocity(),
            limits=self._pvat_limits,
        )
        return _SolvedCandidate(
            target=target.model_copy(deep=True),
            metrics=metrics,
            point=point,
            recovery_fraction=0.0,
            pvat_mode="catch_up",
            advances_target=False,
        )

    async def _inverse_kinematics(
        self,
        client: LebaiClientProtocol,
        snapshot: LebaiSnapshot,
        target: Pose,
    ) -> JointVector:
        try:
            solution = await self._await_motion_call(
                "kinematics_inverse", lambda: client.kinematics_inverse(
                    pose_to_lebai(target),
                    list(
                        self._last_accepted_solution_q
                        or snapshot.actual_q
                    ),
                ),
                .20, generation=self._pump.generation,
            )
        except TimeoutError:
            raise BackendCommandError("sdk_timeout:ik") from None
        except Exception:
            raise BackendCommandError("sdk_call_failed:ik") from None
        if solution is None:
            raise BackendCommandError("ik_unreachable")
        return joint_vector(list(solution), "ik_solution")

    async def _await_motion_call(self, method, operation, timeout_s, *, generation, entered=None):
        # A trusted bounded proxy can reject before invoking its native client.
        # Entering that proxy is not itself evidence of a native RPC entry.
        counter = getattr(self._client, '_native_motion_entry_count', None)
        if callable(counter):
            baseline = counter(method)
            if type(baseline) is not int or baseline < 0:
                raise BackendCommandError('invalid_motion_entry_evidence')
            def native_entered():
                try:
                    value = counter(method)
                    return type(value) is not int or value < baseline or value > baseline
                except Exception:
                    return True  # Missing evidence never proves non-entry.
            entered = native_entered
        def revoked():
            return (self._disconnecting or self._stop_lock.locked()
                or self._pump.generation != generation
                or self._motion_authority is not None and self._motion_authority() is not True)
        try:
            return await self._motion_calls.call(method, operation, timeout_s,
                revoked=revoked, entered=entered)
        finally:
            if method in self._motion_calls.unknown_methods:
                self._preflight_ready = False
                if method == "move_pvat":
                    self._latched_fault = "stop_unverified"
            self._schedule_sdk_diagnostic(dict(kind="motion_rpc_ownership",
                **self._motion_calls.snapshot()))

    def _build_advancing_candidate(
        self,
        snapshot: LebaiSnapshot,
        target: Pose,
        metrics: IkCandidateMetrics,
        fraction: float,
    ) -> _SolvedCandidate:
        try:
            point = build_pvat_point(
                solution_q=metrics.solution_q,
                actual_q=snapshot.actual_q,
                actual_qd=snapshot.actual_qd,
                previous_qd=self._continuous_pvat_velocity(),
                limits=self._pvat_limits,
            )
        except BackendCommandError as error:
            if (
                str(error) == "ik_tracking_diverged"
                and metrics.solution_step_rad
                > self.settings.control.max_joint_step_rad
            ):
                raise BackendCommandError("ik_joint_jump") from None
            raise
        if (
            metrics.solution_step_rad
            > self.settings.control.max_joint_step_rad
        ):
            raise BackendCommandError("ik_joint_jump")
        return _SolvedCandidate(
            target=target.model_copy(deep=True),
            metrics=metrics,
            point=point,
            recovery_fraction=fraction,
            pvat_mode=(
                "advance" if fraction == 1.0 else "interpolated_advance"
            ),
            advances_target=True,
        )

    def _note_soft_constraint(
        self,
        reason: str,
        *,
        persistent: bool = True,
        command: CommandRef | None = None,
        outcome: Literal["rejected", "interpolated_advance"] = "rejected",
    ) -> None:
        self._consecutive_ik_failures += 1
        self._constraint_error = (
            "ik_unreachable" if reason in {"ik_unreachable", "ik_invalid"}
            else reason
        )
        self._constraint_feedback = CommandFeedback(command, self._constraint_error, outcome)
        self._constraint = (
            "ik_boundary"
            if self._constraint_error == "ik_unreachable"
            else (
                "motion_continuity_boundary"
                if self._constraint_error
                in {"ik_joint_jump", "joint_speed_limit"}
                else "joint_boundary"
            )
        )
        if persistent and self._consecutive_ik_failures >= 5:
            raise BackendCommandError("ik_failure_persistent")

    def _continuous_pvat_velocity(self) -> JointVector | None:
        return self._pvat_velocity_reference_at(self._pvat_reference_now())

    def _pvat_reference_now(self) -> int | None:
        # This dedicated local lifetime clock does not replace the VR or stop
        # clocks, and does not reuse the diagnostic clock-anchor helpers.
        try:
            value = self._pvat_clock()
        except Exception:
            return None
        return value if type(value) is int and value >= 0 else None

    def _pvat_velocity_reference_at(self, reference_now_ns: int | None) -> JointVector | None:
        # The controller decelerates when the point stream exceeds its
        # horizon. A previous commanded velocity is then not a valid initial
        # condition: let build_pvat_point use the observed joint velocity.
        if (self._last_pvat_started_ns is None
                or self._last_pvat_clock_started_ns is None
                or reference_now_ns is None):
            return None
        elapsed_ns = self._clock() - self._last_pvat_started_ns
        reference_elapsed_ns = reference_now_ns - self._last_pvat_clock_started_ns
        horizon_ns = self._pvat_limits.horizon_s * 1_000_000_000
        # Pair timestamps only within their own clocks. Keeping both checks
        # prevents coarse clock ticks from extending the previous reference.
        if not (0 <= elapsed_ns < horizon_ns and 0 <= reference_elapsed_ns < horizon_ns):
            return None
        return self._previous_sent_qd

    def _reset_pvat_history(self) -> None:
        self._previous_sent_qd = None
        self._last_pvat_started_ns = None
        self._last_pvat_clock_started_ns = None
        self._last_sent_tcp = None
        self._last_accepted_solution_q = None
        self._accepted_target_command_id = None
        self._accepted_target_ref = None

    async def _emit_event(self, event: dict[str, object]) -> None:
        if self._event_callback is None:
            return
        try:
            await self._event_callback(event, self._clock())
        except Exception:
            raise BackendCommandError("recording_unavailable") from None

    async def _emit_diagnostic_event(
        self,
        event: dict[str, object],
    ) -> None:
        if self._event_callback is None:
            return
        try:
            await self._event_callback(event, self._clock())
        except Exception:
            return

    async def read_stop_observation(self) -> dict[str, object]:
        """Read evidence despite a latched fault; never clear it or issue writes.

        The commissioning caller uses this only after stopping, not as another
        concurrent SDK poller during motion. This is not a motion preflight.
        """
        snapshot = await self._read_snapshot(
            emit_kinematics=False, include_raw_kinematics=self._stop_diagnostic_sampling,
            read_purpose="stop_observation",
        )
        observation = {
            "captured_ns": snapshot.captured_ns,
            "read_requested_ns": snapshot.read_requested_ns,
            "read_completed_ns": snapshot.read_completed_ns,
            "read_purpose": snapshot.read_purpose,
            "observed_ns": self._clock(),
            "raw_robot_state": snapshot.raw_robot_state,
            "robot_state": snapshot.robot_state.value,
            "estop": snapshot.estop,
            "latched_fault": self._latched_fault,
            "running_motion": snapshot.running_motion,
            "raw_running_motion": snapshot.raw_running_motion,
            "motion_state": snapshot.motion_state,
            "actual_q": list(snapshot.actual_q),
            "actual_qd": list(snapshot.actual_qd),
            "actual_qdd": list(snapshot.actual_qdd),
            "actual_tcp": snapshot.actual_tcp.model_dump(mode="json"),
            "target_q": list(snapshot.target_q),
            "target_qd": list(snapshot.target_qd),
            "target_tcp": snapshot.target_tcp.model_dump(mode="json"),
            "sdk_latencies_ms": dict(snapshot.sdk_latencies_ms),
            "snapshot_lock_wait_ms": snapshot.sdk_lock_wait_ms,
        }
        if snapshot.raw_kinematics is not None:
            observation["raw_kinematics"] = copy.deepcopy(snapshot.raw_kinematics)
        return observation

    async def _read_snapshot(
        self, *, emit_kinematics: bool = True, include_raw_kinematics: bool = False,
        read_purpose: str = "unspecified",
        observer_timing: dict[str, object] | None = None,
    ) -> LebaiSnapshot:
        if self._snapshot_calls.unknown_methods:
            raise BackendCommandError("snapshot_rpc_result_unknown")
        owner = self._snapshot_calls
        read = _SnapshotRead(self._client, self._pump.generation, owner=owner)
        owner.active += 1  # Include lock waiting, decoding and logging, not only RPCs.
        try:
            return await self._read_snapshot_impl(
                read, emit_kinematics=emit_kinematics,
                include_raw_kinematics=include_raw_kinematics,
                read_purpose=read_purpose, observer_timing=observer_timing,
            )
        except BaseException as error:
            self._remember_snapshot_fault(
                read, str(error) if isinstance(error, BackendCommandError) else "robot_state_stale",
            )
            raise
        finally:
            owner.active -= 1

    async def _read_snapshot_impl(
        self, read: _SnapshotRead, *, emit_kinematics: bool,
        include_raw_kinematics: bool, read_purpose: str,
        observer_timing: dict[str, object] | None,
    ) -> LebaiSnapshot:
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        latencies: dict[str, float] = {}
        performance_timing: dict[str, object] = {
            "clock": "time.perf_counter_ns", "alignment_verified": False,
            "requested_ns": sample_performance_ns(),
        }
        if observer_timing is not None:
            performance_timing["observer_window"] = dict(observer_timing)
        read_requested_ns = self._clock()
        read_generation = self._pump.generation
        async with self._sdk_lock:
            if (client is not self._client or read.owner is not self._snapshot_calls
                    or self._snapshot_calls.closed):
                raise BackendCommandError("snapshot_read_client_changed")
            self._snapshot_read_sequence += 1
            read.sequence = self._snapshot_read_sequence
            self._latest_snapshot_read = read
            performance_timing["acquired_ns"] = sample_performance_ns()
            captured_ns = self._clock()
            sdk_lock_wait_ms = (captured_ns - read_requested_ns) / 1_000_000
            deadline_ns = captured_ns + _SNAPSHOT_READ_TIMEOUT_NS
            connected = await self._timed(
                "is_connected",
                client.is_connected,
                latencies,
                deadline_ns=deadline_ns,
            )
            if not connected:
                raise BackendCommandError("robot_disconnected")
            raw_state = await self._timed(
                "get_robot_state",
                client.get_robot_state,
                latencies,
                deadline_ns=deadline_ns,
            )
            raw_estop = await self._timed(
                "get_estop_reason",
                client.get_estop_reason,
                latencies,
                deadline_ns=deadline_ns,
            )
            raw_kin = await self._timed(
                "get_kin_data",
                client.get_kin_data,
                latencies,
                deadline_ns=deadline_ns,
            )
            # Capture the already-returned payload before another SDK await.
            # Diagnostic tail only: no extra RPC and no inferred missing fields.
            raw_kinematics = (copy.deepcopy(dict(raw_kin))
                              if include_raw_kinematics and isinstance(raw_kin, Mapping)
                              else None)
            raw_tcp = await self._timed(
                "get_tcp",
                client.get_tcp,
                latencies,
                deadline_ns=deadline_ns,
            )
            raw_claw = await self._timed(
                "get_claw",
                client.get_claw,
                latencies,
                deadline_ns=deadline_ns,
            )
            raw_running = await self._timed(
                "get_running_motion",
                client.get_running_motion,
                latencies,
                deadline_ns=deadline_ns,
            )
            robot_state = map_robot_state(raw_state)
            running_motion = _running_motion(raw_running)
            motion_state = None
            # STOP/PAUSED may retain a motion ID. Resolve it explicitly, just
            # as in IDLE; never infer FINISHED from zero speed or a constant ID.
            # This does not change the raw/mapped robot state or motion gates.
            settled_state = raw_state.strip().upper() if isinstance(raw_state, str) else raw_state
            if (
                settled_state in _STOP_SETTLED_STATES
                and running_motion is not None
            ):
                raw_motion_state = await self._timed(
                    "get_motion_state",
                    lambda: client.get_motion_state(running_motion),
                    latencies,
                    deadline_ns=deadline_ns,
                    timeout_error="sdk_timeout:get_motion_state",
                )
                if isinstance(raw_motion_state, str):
                    motion_state = raw_motion_state.strip().upper()
                if motion_state == "FINISHED":
                    running_motion = None
        performance_timing["completed_ns"] = sample_performance_ns()
        read_completed_ns = self._clock()
        if not isinstance(raw_kin, Mapping):
            raise BackendCommandError("invalid_sdk_kin_data")
        if not isinstance(raw_tcp, Mapping):
            raise BackendCommandError("invalid_sdk_pose")
        if not isinstance(raw_claw, Mapping):
            raise BackendCommandError("invalid_sdk_claw")
        snapshot = LebaiSnapshot(
            captured_ns=captured_ns,
            read_requested_ns=read_requested_ns,
            read_completed_ns=read_completed_ns,
            read_purpose=read_purpose,
            robot_state=robot_state,
            estop=estop_fault(raw_estop),
            actual_q=joint_vector(
                raw_kin.get("actual_joint_pose"),
                "actual_joint_pose",
            ),
            actual_qd=joint_vector(
                raw_kin.get("actual_joint_speed"),
                "actual_joint_speed",
            ),
            actual_qdd=joint_vector(
                raw_kin.get("actual_joint_acc"),
                "actual_joint_acc",
            ),
            target_q=joint_vector(
                raw_kin.get("target_joint_pose"),
                "target_joint_pose",
            ),
            target_qd=joint_vector(
                raw_kin.get("target_joint_speed"),
                "target_joint_speed",
            ),
            target_qdd=joint_vector(
                raw_kin.get("target_joint_acc"),
                "target_joint_acc",
            ),
            actual_tcp=_pose_field(raw_kin, "actual_tcp_pose"),
            target_tcp=_pose_field(raw_kin, "target_tcp_pose"),
            actual_flange=_pose_field(raw_kin, "actual_flange_pose"),
            tcp_setting=pose_from_lebai(raw_tcp),
            gripper=self._gripper_from_claw(raw_claw),
            running_motion=running_motion,
            sdk_latencies_ms=latencies,
            raw_robot_state=raw_state,
            sdk_lock_wait_ms=sdk_lock_wait_ms,
            raw_kinematics=raw_kinematics,
            raw_running_motion=raw_running,
            motion_state=motion_state,
            diagnostic_timing=performance_timing,
        )
        self._snapshot = snapshot
        read.snapshot = snapshot
        fault = self._runtime_motion_fault(snapshot)
        if fault is None and not self._tcp_matches(snapshot.tcp_setting):
            fault = "tcp_mismatch"
        if fault is not None:
            self._remember_snapshot_fault(read, fault)
        # A queued/in-flight old read must not replace current public evidence.
        # The previous sample may remain visible with its original generation.
        if client is self._client and read_generation == self._pump.generation:
            self._motion_observation = JointObservation(
                generation=read_generation, captured_ns=snapshot.captured_ns,
                read_completed_ns=snapshot.read_completed_ns, read_purpose=read_purpose,
                actual_q=tuple(snapshot.actual_q), actual_qd=tuple(snapshot.actual_qd),
                actual_qdd=tuple(snapshot.actual_qdd),
            )
        if emit_kinematics:
            await self._emit_kinematics(snapshot)
        if client is not self._client or read.owner is not self._snapshot_calls or read.owner.closed:
            raise BackendCommandError("snapshot_read_client_changed")
        return snapshot

    async def _timed(
        self,
        name: str,
        operation: Callable[[], Awaitable[Any]],
        latencies: dict[str, float],
        *,
        deadline_ns: int | None = None,
        timeout_error: str | None = None,
    ) -> Any:
        started = self._clock()
        deadline_limited = False
        try:
            timeout_s = 0.20
            if deadline_ns is not None:
                remaining_ns = deadline_ns - self._clock()
                if remaining_ns <= 0:
                    raise BackendCommandError("robot_state_stale")
                timeout_s = remaining_ns / 1_000_000_000
                deadline_limited = True
            owner, client = self._snapshot_calls, self._client
            if owner.unknown_methods:
                raise BackendCommandError("snapshot_rpc_result_unknown")
            entered, unknown = self._snapshot_native_evidence(client, name)
            try:
                result = await owner.call(name, operation, timeout_s, entered=entered, native_result_unknown=unknown,
                    revoked=lambda: owner.closed or owner is not self._snapshot_calls or client is not self._client)
                if owner.closed or owner is not self._snapshot_calls or client is not self._client:
                    raise BackendCommandError("snapshot_read_client_changed")
            finally:
                if owner.unknown_methods and owner is self._snapshot_calls:
                    self._preflight_ready = False
                if owner.unknown_methods or owner.closed:
                    self._schedule_sdk_diagnostic(dict(kind='snapshot_rpc_ownership', **owner.snapshot()))
            if deadline_ns is not None and self._clock() > deadline_ns:
                raise BackendCommandError("robot_state_stale")
            return result
        except TimeoutError:
            if deadline_limited and timeout_error is None:
                raise BackendCommandError("robot_state_stale") from None
            raise BackendCommandError(
                timeout_error or f"sdk_timeout:{name}"
            ) from None
        except BackendCommandError:
            raise
        except Exception:
            raise BackendCommandError(f"sdk_call_failed:{name}") from None
        finally:
            latencies[name] = max(0.0, (self._clock() - started) / 1_000_000)

    def _command_snapshot_max_age_ns(self) -> int:
        return _SNAPSHOT_READ_TIMEOUT_NS + int(
            1 / self.settings.control.state_hz * 1_000_000_000
        )

    @staticmethod
    def _snapshot_native_evidence(client, name):
        counter = getattr(client, '_native_snapshot_entry_count', None)
        status = getattr(client, '_native_snapshot_result_unknown', None)
        entered = None
        if callable(counter):
            baseline = counter(name)
            if type(baseline) is not int or baseline < 0:
                raise BackendCommandError('invalid_snapshot_entry_evidence')
            def entered():
                try:
                    value = counter(name)
                    return type(value) is not int or value != baseline
                except Exception:
                    return True
        return entered, (lambda: status(name)) if callable(status) else None

    def _runtime_motion_fault(self, snapshot: LebaiSnapshot) -> str | None:
        if snapshot.estop is not None:
            return snapshot.estop
        if snapshot.robot_state not in {BackendState.IDLE, BackendState.MOVING}:
            return f"robot_state_{snapshot.robot_state.value.lower()}"
        return None

    def _motion_boundary_fault(self, snapshot: LebaiSnapshot) -> str | None:
        if self._latched_fault is not None:
            return self._latched_fault
        runtime_fault = self._runtime_motion_fault(snapshot)
        if runtime_fault is not None:
            return runtime_fault
        if not self._tcp_matches(snapshot.tcp_setting):
            return "tcp_mismatch"
        if not self._joints_inside_limits(snapshot.actual_q):
            return "joint_outside_soft_limits"
        if not self._tcp_inside_startup_envelope(snapshot.actual_tcp):
            return "tcp_outside_startup_envelope"
        if not self._capabilities.control_ready:
            return "sdk_capability_missing"
        return None

    async def _emit_kinematics(self, snapshot: LebaiSnapshot) -> None:
        if self._event_callback is None:
            return
        event = {
            "kind": "robot_kinematics",
            # Local sequential SDK-read boundaries, not device sample times.
            "snapshot_read_requested_ns": snapshot.read_requested_ns,
            "snapshot_read_started_ns": snapshot.captured_ns,
            "snapshot_read_completed_ns": snapshot.read_completed_ns,
            "snapshot_read_purpose": snapshot.read_purpose,
            "snapshot_lock_wait_ms": snapshot.sdk_lock_wait_ms,
            "diagnostic_timing": copy.deepcopy(snapshot.diagnostic_timing or {}),
            "raw_robot_state": snapshot.raw_robot_state,
            "robot_state": snapshot.robot_state.value,
            "estop": snapshot.estop,
            "running_motion": snapshot.running_motion,
            "raw_running_motion": snapshot.raw_running_motion,
            "motion_state": snapshot.motion_state,
            "actual_q": list(snapshot.actual_q),
            "actual_qd": list(snapshot.actual_qd),
            "actual_qdd": list(snapshot.actual_qdd),
            "target_q": list(snapshot.target_q),
            "target_qd": list(snapshot.target_qd),
            "target_qdd": list(snapshot.target_qdd),
            "actual_tcp": snapshot.actual_tcp.model_dump(),
            "target_tcp": snapshot.target_tcp.model_dump(),
            "gripper": snapshot.gripper,
            "sdk_latencies_ms": dict(snapshot.sdk_latencies_ms),
        }
        try:
            await self._event_callback(event, snapshot.captured_ns)
        except Exception:
            raise BackendCommandError("recording_unavailable") from None

    def _tcp_matches(self, actual: Pose) -> bool:
        expected = self.settings.expected_tcp
        expected_pose = pose_from_lebai(
            {
                "x": expected.x,
                "y": expected.y,
                "z": expected.z,
                "rz": expected.rz,
                "ry": expected.ry,
                "rx": expected.rx,
            }
        )
        translation_error = math.dist(actual.p, expected_pose.p)
        rotation_error = (
            Rotation.from_quat(actual.q)
            * Rotation.from_quat(expected_pose.q).inv()
        ).magnitude()
        return (
            translation_error <= self.settings.tcp_position_tolerance_m
            and math.degrees(rotation_error)
            <= self.settings.tcp_rotation_tolerance_deg
        )

    def _joints_inside_limits(self, q: JointVector) -> bool:
        margin = self.settings.joint_limit_margin_rad
        return all(
            lower + margin <= value <= upper - margin
            for value, lower, upper in zip(
                q,
                self.settings.soft_joint_min_rad,
                self.settings.soft_joint_max_rad,
                strict=True,
            )
        )

    def _tcp_inside_startup_envelope(self, pose: Pose) -> bool:
        return all(
            lower <= value <= upper
            for value, lower, upper in zip(
                pose.p,
                self.settings.startup_tcp_min_m,
                self.settings.startup_tcp_max_m,
                strict=True,
            )
        )

    def _gripper_from_claw(self, claw: Mapping[str, object]) -> float:
        amplitude = _finite_claw_value(claw.get("amplitude"))
        open_value = self.settings.gripper.open_amplitude_percent
        closed_value = self.settings.gripper.closed_amplitude_percent
        if open_value == closed_value:
            raise BackendCommandError("invalid_gripper_amplitude_config")
        normalized = (amplitude - open_value) / (closed_value - open_value)
        return min(1.0, max(0.0, normalized))

    def _require_control(self) -> None:
        if self._disconnecting:
            raise BackendCommandError("disconnect_in_progress")
        if self.settings.mode != "control":
            raise BackendCommandError("real_robot_readonly")
        if self._client is None:
            raise BackendCommandError("robot_disconnected")
        if self._snapshot_calls.unknown_methods:
            raise BackendCommandError("snapshot_rpc_result_unknown")
        if self._motion_calls.unknown_methods:
            raise BackendCommandError("motion_rpc_result_unknown")

    def _begin_motion_epoch(self) -> None:
        if self._snapshot_calls.unknown_methods:
            raise BackendCommandError("snapshot_rpc_result_unknown")
        if self._motion_calls.unknown_methods:
            raise BackendCommandError("motion_rpc_result_unknown")
        if self._disconnecting:
            raise BackendCommandError("disconnect_in_progress")
        if self._stop_lock.locked():
            raise BackendCommandError("stop_in_progress")
        if self._stop_transaction is not None:
            if self._stop_transaction.unresolved or self._latched_fault is not None:
                raise BackendCommandError(self._latched_fault or "stop_unverified")
            self._stop_transaction = None


def _pose_field(payload: Mapping[str, object], field: str) -> Pose:
    value = payload.get(field)
    if not isinstance(value, Mapping):
        raise BackendCommandError("invalid_sdk_pose")
    return pose_from_lebai(value)


def _running_motion(value: object) -> object | None:
    if value is None:
        return None
    if isinstance(value, bool):
        raise BackendCommandError("invalid_sdk_running_motion")
    if value == 0:
        return None
    return value


def _finite_claw_value(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BackendCommandError("invalid_sdk_claw")
    result = float(value)
    if not math.isfinite(result) or not 0 <= result <= 100:
        raise BackendCommandError("invalid_sdk_claw")
    return result
