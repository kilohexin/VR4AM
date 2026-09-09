from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from typing import Any, Literal

from scipy.spatial.transform import Rotation

from app.config import TELEOP_SINGULARITY_GUARD_RAD, LebaiSettings
from app.robots.base import (
    BackendCommandError,
    BackendPreflight,
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
from app.robots.lebai_pvat import PvatLimits, PvatPoint, build_pvat_point
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


@dataclass(frozen=True)
class _SolvedCandidate:
    target: Pose
    metrics: IkCandidateMetrics
    point: PvatPoint
    recovery_fraction: float
    pvat_mode: PvatMode
    advances_target: bool


class _CandidateRejected(BackendCommandError):
    def __init__(self, reason: str, metrics: IkCandidateMetrics) -> None:
        super().__init__(reason)
        self.metrics = metrics


class RealLebaiAdapter:
    def __init__(
        self,
        settings: LebaiSettings,
        *,
        client_factory: ClientFactory = connect_real_client,
        clock: Callable[[], int] = time.monotonic_ns,
        sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
        event_callback: EventCallback | None = None,
        backend_label: Literal["LEBAI", "LEBAI_FAKE"] = "LEBAI",
        pump_clock: Callable[[], float] | None = None,
        pump_sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
    ) -> None:
        self.settings = settings
        self._client_factory = client_factory
        self._clock = clock
        self._sleep = sleep
        self._event_callback = event_callback
        self._backend_label = backend_label
        self._client: LebaiClientProtocol | None = None
        self._sdk_lock = asyncio.Lock()
        self._snapshot: LebaiSnapshot | None = None
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
        self._last_sent_tcp: Pose | None = None
        self._last_accepted_solution_q: JointVector | None = None
        self._accepted_target_command_id: int | None = None
        self._constraint: str | None = None
        self._constraint_error: str | None = None
        self._consecutive_ik_failures = 0
        self._latched_fault: str | None = None
        self._latched_fault_from_cancelled_stop = False
        self._preflight_ready = False

    @property
    def constraint(self) -> str | None:
        return self._constraint

    @property
    def pump_fault(self) -> BackendCommandError | None:
        return self._pump.fault

    async def connect(self) -> None:
        if self._client is not None:
            return
        try:
            client = await asyncio.wait_for(
                self._client_factory(self.settings.ip),
                timeout=3.0,
            )
        except TimeoutError:
            raise BackendCommandError("sdk_timeout:connect") from None
        if not await client.is_connected():
            raise BackendCommandError("robot_disconnected")
        self._client = client
        self._capabilities = detect_capabilities(client)
        try:
            snapshot = await self._read_snapshot()
            preflight = self._preflight_from_snapshot(snapshot)
            self._preflight_ready = preflight.ready
            if self.settings.mode == "control" and preflight.ready:
                await self._pump.start()
        except BaseException:
            self._client = None
            self._snapshot = None
            self._preflight_ready = False
            raise

    async def disconnect(self) -> None:
        if (
            self.settings.mode == "control"
            and self._motion_accepted
            and self._client is not None
        ):
            await self.stop(StopReason.SHUTDOWN)
        await self._pump.stop()
        self._reset_pvat_history()
        self._snapshot = None
        self._client = None
        self._motion_accepted = False
        self._preflight_ready = False

    async def command_tcp(self, target: Pose, command_id: int) -> None:
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
        self._pump.submit(target, command_id)
        if self._constraint_error is not None:
            raise BackendCommandError(self._constraint_error)

    async def set_gripper(self, value: float) -> None:
        self._require_control()
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        if not self._preflight_ready:
            raise BackendCommandError("preflight_not_ready")
        snapshot = await self._read_snapshot()
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
        async with self._sdk_lock:
            try:
                await asyncio.wait_for(
                    client.set_claw(gripper.max_force_percent, amplitude),
                    timeout=0.20,
                )
            except TimeoutError:
                raise BackendCommandError("sdk_timeout:set_claw") from None
            except Exception:
                raise BackendCommandError("sdk_call_failed:set_claw") from None

    async def stop(self, reason: StopReason) -> None:
        if self.settings.mode == "readonly":
            return
        diagnostics: dict[str, Any] = {
            "kind": "stop_diagnostics",
            "reason": reason.value,
            "initial_fault": self._latched_fault,
            "started_ns": self._clock(),
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
            # Never delay the stop or its escalation on the recorder. Flush only
            # after the safety work, outside the SDK lock, with a bounded wait.
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
        self._pump.invalidate()
        self._reset_pvat_history()
        self._preflight_ready = False
        client = self._client
        if client is None:
            self._latch_unverified_stop()
            raise BackendCommandError("robot_disconnected")
        try:
            async with self._sdk_lock:
                await self._call_stop_rpc(client, "stop_move", diagnostics)
        except asyncio.CancelledError:
            await asyncio.shield(
                self._fail_unverified_stop(
                    client,
                    diagnostics=diagnostics,
                    from_cancelled_stop=True,
                )
            )
            raise
        except TimeoutError:
            await self._fail_unverified_stop(client, diagnostics=diagnostics)
            raise BackendCommandError("sdk_timeout:stop_move") from None
        except Exception:
            await self._fail_unverified_stop(client, diagnostics=diagnostics)
            raise BackendCommandError("sdk_call_failed:stop_move") from None
        started_ns = self._clock()
        stable_since_ns: int | None = None
        stable_anchor: LebaiSnapshot | None = None
        try:
            while True:
                snapshot = await self._read_snapshot(emit_kinematics=False)
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
                    "observed_ns": now_ns,
                    "raw_robot_state": snapshot.raw_robot_state,
                    "estop": snapshot.estop,
                    "running_motion": snapshot.running_motion,
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
                    elif now_ns - stable_since_ns >= 300_000_000:
                        self._motion_accepted = False
                        if reason is StopReason.DISCONNECT:
                            self._resolve_cancelled_stop_with_verified_disconnect()
                        return
                else:
                    stable_since_ns = None
                    stable_anchor = None
                if now_ns - started_ns >= 500_000_000:
                    self._latched_fault = "stop_incomplete"
                    await self._fail_unverified_stop(
                        client,
                        diagnostics=diagnostics,
                        preserve_fault=True,
                    )
                    raise BackendCommandError("stop_incomplete")
                await self._sleep(0.02)
        except asyncio.CancelledError:
            await asyncio.shield(
                self._fail_unverified_stop(
                    client,
                    diagnostics=diagnostics,
                    from_cancelled_stop=True,
                )
            )
            raise
        except BackendCommandError as error:
            if str(error) == "stop_incomplete":
                raise
            await self._fail_unverified_stop(client, diagnostics=diagnostics)
            if str(error) == "robot_disconnected":
                raise BackendCommandError(
                    "stop_unverified_disconnected"
                ) from None
            raise
        except Exception:
            await self._fail_unverified_stop(client, diagnostics=diagnostics)
            raise BackendCommandError("stop_unverified") from None

    def _latch_unverified_stop(
        self,
        *,
        from_cancelled_stop: bool = False,
    ) -> None:
        self._latched_fault = "stop_unverified"
        self._latched_fault_from_cancelled_stop = from_cancelled_stop
        self._motion_accepted = False
        self._preflight_ready = False
        self._pump.invalidate()
        self._reset_pvat_history()

    async def _fail_unverified_stop(
        self,
        client: LebaiClientProtocol,
        *,
        diagnostics: dict[str, Any],
        preserve_fault: bool = False,
        from_cancelled_stop: bool = False,
    ) -> None:
        if not preserve_fault:
            self._latch_unverified_stop(
                from_cancelled_stop=from_cancelled_stop,
            )
        else:
            self._motion_accepted = False
            self._preflight_ready = False
            self._pump.invalidate()
            self._reset_pvat_history()
        await self._safety_escalate_stop_sys(client, diagnostics)

    def _resolve_cancelled_stop_with_verified_disconnect(self) -> None:
        if not self._latched_fault_from_cancelled_stop:
            return
        if self._latched_fault != "stop_unverified":
            return
        self._latched_fault = None
        self._latched_fault_from_cancelled_stop = False

    async def _safety_escalate_stop_sys(
        self,
        client: LebaiClientProtocol,
        diagnostics: dict[str, Any],
    ) -> None:
        async with self._sdk_lock:
            try:
                connected = await self._call_stop_rpc(client, "is_connected", diagnostics)
            except Exception:
                connected = False
            if not connected:
                return
            try:
                await self._call_stop_rpc(client, "stop_sys", diagnostics)
            except Exception:
                return

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        self._require_control()
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        snapshot = await self._read_snapshot()
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
        snapshot = await self._read_snapshot()
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
        if client is None:
            raise BackendCommandError("robot_disconnected")
        if self._pump.has_pending:
            raise BackendCommandError("home_command_pending")
        self._preflight_ready = False
        self._pump.invalidate()
        self._reset_pvat_history()
        on_phase("homing")
        try:
            async with self._sdk_lock:
                self._motion_accepted = True
                motion_id = await asyncio.wait_for(
                    client.movej(
                        list(target_q),
                        self.settings.control.max_joint_acceleration_radps2,
                        options.max_speed_radps,
                        0.0,
                        0.0,
                    ),
                    timeout=0.20,
                )
        except asyncio.CancelledError:
            await asyncio.shield(self.stop(StopReason.HOME))
            raise
        except TimeoutError:
            await self.stop(StopReason.HOME)
            raise BackendCommandError("sdk_timeout:movej") from None
        except Exception:
            await self.stop(StopReason.HOME)
            raise BackendCommandError("sdk_call_failed:movej") from None
        started_ns = self._clock()
        stable_since_ns: int | None = None
        stabilizing = False
        while True:
            snapshot = await self._read_snapshot()
            async with self._sdk_lock:
                try:
                    motion_state = await asyncio.wait_for(
                        client.get_motion_state(motion_id),
                        timeout=0.20,
                    )
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
                await self.stop(StopReason.HOME)
                raise BackendCommandError("home_timeout")
            await self._sleep(0.02)

    async def get_state(self) -> RobotStateMessage:
        if self._pump.fault is not None and self._latched_fault is None:
            raise self._pump.fault
        snapshot = await self._read_snapshot()
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
        )

    async def preflight(self) -> BackendPreflight:
        snapshot = await self._read_snapshot()
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
        handler_started_ns = self._clock()
        snapshot = await self._read_snapshot()
        snapshot_finished_ns = self._clock()
        if not self._pump.is_current(request.generation):
            return
        self._ensure_command_snapshot_fresh(snapshot)
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        runtime_fault = self._runtime_motion_fault(snapshot)
        if runtime_fault is not None:
            self._latched_fault = runtime_fault
            self._preflight_ready = False
            self._motion_accepted = False
            self._pump.invalidate()
            self._reset_pvat_history()
            try:
                await self.stop(StopReason.FAULT)
            except asyncio.CancelledError:
                raise
            except Exception:
                if self._latched_fault == runtime_fault:
                    self._latched_fault = "stop_unverified"
            if self._latched_fault == runtime_fault:
                raise BackendCommandError(runtime_fault)
            raise BackendCommandError(self._latched_fault or "stop_unverified")
        accepted: tuple[_SolvedCandidate, float] | None = None
        previous_pvat_gap_ms: float | None = None
        last_recoverable_error: str | None = None
        diagnostic_events: list[dict[str, object]] = []
        lock_requested_ns = self._clock()
        async with self._sdk_lock:
            lock_acquired_ns = self._clock()
            selected: _SolvedCandidate | None = None
            if self._backend_label == "LEBAI_FAKE":
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
            if selected is None and not self._pump.is_current(
                request.generation
            ):
                return
            if selected is not None:
                if not self._pump.is_current(request.generation):
                    return
                self._ensure_command_snapshot_fresh(snapshot)
                pvat_started = self._clock()
                if self._last_pvat_started_ns is not None:
                    previous_pvat_gap_ms = (
                        pvat_started - self._last_pvat_started_ns
                    ) / 1_000_000
                try:
                    await asyncio.wait_for(
                        client.move_pvat(
                            list(selected.point.q),
                            list(selected.point.qd),
                            list(selected.point.qdd),
                            selected.point.horizon_s,
                        ),
                        timeout=0.06,
                    )
                except TimeoutError:
                    if not self._pump.is_current(request.generation):
                        return
                    raise BackendCommandError("sdk_timeout:move_pvat") from None
                except Exception:
                    if not self._pump.is_current(request.generation):
                        return
                    raise BackendCommandError(
                        "sdk_call_failed:move_pvat"
                    ) from None
                pvat_latency_ms = max(
                    0.0,
                    (self._clock() - pvat_started) / 1_000_000,
                )
                send_finished_ns = self._clock()
                if not self._pump.is_current(request.generation):
                    return
                if selected.pvat_mode == "catch_up":
                    self._constraint = "motion_continuity_boundary"
                    self._constraint_error = "ik_tracking_lag"
                elif selected.recovery_fraction == 1.0:
                    self._constraint = None
                    self._constraint_error = None
                    self._consecutive_ik_failures = 0
                else:
                    self._note_soft_constraint(
                        last_recoverable_error or "ik_unreachable",
                        persistent=False,
                    )
                self._previous_sent_qd = selected.point.qd
                self._last_pvat_started_ns = pvat_started
                if selected.advances_target:
                    self._last_sent_tcp = selected.target.model_copy(
                        deep=True
                    )
                    self._last_accepted_solution_q = (
                        selected.metrics.solution_q
                    )
                    self._accepted_target_command_id = request.command_id
                self._command_id = request.command_id
                self._motion_accepted = True
                accepted = (selected, pvat_latency_ms)
        if accepted is None:
            try:
                self._note_soft_constraint(
                    last_recoverable_error or "ik_unreachable",
                    persistent=self._backend_label != "LEBAI_FAKE",
                )
            except BackendCommandError:
                for event in diagnostic_events:
                    await self._emit_diagnostic_event(event)
                if not self._pump.is_current(request.generation):
                    return
                raise
            for event in diagnostic_events:
                await self._emit_diagnostic_event(event)
            return
        selected, pvat_latency_ms = accepted
        await self._emit_event(
            {
                "kind": "pvat_sent",
                "previous_pvat_gap_ms": previous_pvat_gap_ms,
                "timing_ms": {
                    "snapshot_read": (snapshot_finished_ns - handler_started_ns) / 1_000_000,
                    "snapshot_lock_wait": snapshot.sdk_lock_wait_ms,
                    "command_lock_wait": (lock_acquired_ns - lock_requested_ns) / 1_000_000,
                    "candidate_selection": (pvat_started - lock_acquired_ns) / 1_000_000,
                    "send_sdk": pvat_latency_ms,
                    "handler_to_send_complete": (send_finished_ns - handler_started_ns) / 1_000_000,
                    "snapshot_age_at_send": (pvat_started - snapshot.captured_ns) / 1_000_000,
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
            }
        )
        for event in diagnostic_events:
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
            solution = await asyncio.wait_for(
                client.kinematics_inverse(
                    pose_to_lebai(target),
                    list(
                        self._last_accepted_solution_q
                        or snapshot.actual_q
                    ),
                ),
                timeout=0.20,
            )
        except TimeoutError:
            raise BackendCommandError("sdk_timeout:ik") from None
        except Exception:
            raise BackendCommandError("sdk_call_failed:ik") from None
        if solution is None:
            raise BackendCommandError("ik_unreachable")
        return joint_vector(list(solution), "ik_solution")

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
    ) -> None:
        self._consecutive_ik_failures += 1
        self._constraint_error = (
            "ik_unreachable" if reason in {"ik_unreachable", "ik_invalid"}
            else reason
        )
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
        # The controller decelerates when the point stream exceeds its
        # horizon. A previous commanded velocity is then not a valid initial
        # condition: let build_pvat_point use the observed joint velocity.
        if self._last_pvat_started_ns is None:
            return None
        elapsed_ns = self._clock() - self._last_pvat_started_ns
        if not 0 <= elapsed_ns < self._pvat_limits.horizon_s * 1_000_000_000:
            return None
        return self._previous_sent_qd

    def _reset_pvat_history(self) -> None:
        self._previous_sent_qd = None
        self._last_pvat_started_ns = None
        self._last_sent_tcp = None
        self._last_accepted_solution_q = None
        self._accepted_target_command_id = None

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

    async def _read_snapshot(self, *, emit_kinematics: bool = True) -> LebaiSnapshot:
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        latencies: dict[str, float] = {}
        read_requested_ns = self._clock()
        async with self._sdk_lock:
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
            if (
                robot_state is BackendState.IDLE
                and running_motion is not None
            ):
                raw_motion_state = await self._timed(
                    "get_motion_state",
                    lambda: client.get_motion_state(running_motion),
                    latencies,
                    deadline_ns=deadline_ns,
                    timeout_error="sdk_timeout:get_motion_state",
                )
                if str(raw_motion_state).strip().upper() == "FINISHED":
                    running_motion = None
        if not isinstance(raw_kin, Mapping):
            raise BackendCommandError("invalid_sdk_kin_data")
        if not isinstance(raw_tcp, Mapping):
            raise BackendCommandError("invalid_sdk_pose")
        if not isinstance(raw_claw, Mapping):
            raise BackendCommandError("invalid_sdk_claw")
        snapshot = LebaiSnapshot(
            captured_ns=captured_ns,
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
        )
        self._snapshot = snapshot
        if emit_kinematics:
            await self._emit_kinematics(snapshot)
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
            result = await asyncio.wait_for(operation(), timeout=timeout_s)
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
            "snapshot_lock_wait_ms": snapshot.sdk_lock_wait_ms,
            "raw_robot_state": snapshot.raw_robot_state,
            "robot_state": snapshot.robot_state.value,
            "estop": snapshot.estop,
            "running_motion": snapshot.running_motion,
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
        if self.settings.mode != "control":
            raise BackendCommandError("real_robot_readonly")
        if self._client is None:
            raise BackendCommandError("robot_disconnected")


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
