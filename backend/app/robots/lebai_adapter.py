from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Awaitable, Callable, Mapping
from dataclasses import dataclass
from typing import Any

from scipy.spatial.transform import Rotation

from app.config import LebaiSettings
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
from app.robots.lebai_sdk_bridge import (
    LebaiClientProtocol,
    SdkCapabilities,
    connect_real_client,
    detect_capabilities,
)
from app.robots.lebai_pump import PvatPump, PvatRequest
from app.robots.lebai_pvat import PvatLimits, build_pvat_point
from app.schemas.messages import (
    BackendState,
    JointVector,
    Pose,
    RobotStateMessage,
    TeleopMode,
)

ClientFactory = Callable[[str], Awaitable[LebaiClientProtocol]]
EventCallback = Callable[[dict[str, object], int], Awaitable[None]]


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


class RealLebaiAdapter:
    def __init__(
        self,
        settings: LebaiSettings,
        *,
        client_factory: ClientFactory = connect_real_client,
        clock: Callable[[], int] = time.monotonic_ns,
        event_callback: EventCallback | None = None,
    ) -> None:
        self.settings = settings
        self._client_factory = client_factory
        self._clock = clock
        self._event_callback = event_callback
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
            max_joint_step_rad=settings.control.max_joint_step_rad,
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
        )
        self._previous_sent_qd: JointVector | None = None
        self._constraint: str | None = None
        self._constraint_error: str | None = None
        self._consecutive_ik_failures = 0

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
            if (
                self.settings.mode == "control"
                and self._preflight_from_snapshot(snapshot).ready
            ):
                await self._pump.start()
        except BaseException:
            self._client = None
            self._snapshot = None
            raise

    async def disconnect(self) -> None:
        if (
            self.settings.mode == "control"
            and self._motion_accepted
            and self._client is not None
        ):
            await self.stop(StopReason.SHUTDOWN)
        await self._pump.stop()
        self._snapshot = None
        self._client = None
        self._motion_accepted = False

    async def command_tcp(self, target: Pose, command_id: int) -> None:
        self._require_control()
        snapshot = self._snapshot
        if snapshot is None:
            raise BackendCommandError("robot_state_stale")
        max_age_ns = int(
            (1 / self.settings.control.state_hz + 0.04) * 1_000_000_000
        )
        if self._clock() - snapshot.captured_ns > max_age_ns:
            raise BackendCommandError("robot_state_stale")
        if self._pump.fault is not None:
            raise self._pump.fault
        if not self._pump.running:
            raise BackendCommandError("preflight_not_ready")
        self._pump.submit(target, command_id)
        if self._constraint_error is not None:
            raise BackendCommandError(self._constraint_error)

    async def set_gripper(self, value: float) -> None:
        self._require_control()
        raise BackendCommandError("real_robot_control_not_implemented")

    async def stop(self, reason: StopReason) -> None:
        if self.settings.mode == "readonly":
            return
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        self._pump.invalidate()
        self._previous_sent_qd = None
        async with self._sdk_lock:
            try:
                await asyncio.wait_for(client.stop_move(), timeout=0.20)
            except TimeoutError:
                raise BackendCommandError("sdk_timeout:stop_move") from None
            except Exception:
                raise BackendCommandError("sdk_call_failed:stop_move") from None
        self._motion_accepted = False

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        self._require_control()
        raise BackendCommandError("real_robot_control_not_implemented")

    async def get_state(self) -> RobotStateMessage:
        if self._pump.fault is not None:
            raise self._pump.fault
        snapshot = await self._read_snapshot()
        return RobotStateMessage(
            server_mono_ns=snapshot.captured_ns,
            ack_seq=self._command_id,
            mode=TeleopMode.READY,
            robot_state=snapshot.robot_state,
            actual_tcp=snapshot.actual_tcp,
            actual_q=snapshot.actual_q,
            gripper=snapshot.gripper,
            sample_age_ms=0.0,
            fault=snapshot.estop,
        )

    async def preflight(self) -> BackendPreflight:
        snapshot = await self._read_snapshot()
        result = self._preflight_from_snapshot(snapshot)
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
    ) -> BackendPreflight:
        tcp_matches = self._tcp_matches(snapshot.tcp_setting)
        reason: str | None = None
        if snapshot.robot_state is not BackendState.IDLE:
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

    async def _send_target(self, request: PvatRequest) -> None:
        snapshot = await self._read_snapshot()
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        try:
            async with self._sdk_lock:
                try:
                    solution = await asyncio.wait_for(
                        client.kinematics_inverse(
                            pose_to_lebai(request.target),
                            list(snapshot.actual_q),
                        ),
                        timeout=0.20,
                    )
                except TimeoutError:
                    raise BackendCommandError("sdk_timeout:ik") from None
                except Exception:
                    raise BackendCommandError("sdk_call_failed:ik") from None
                if not self._pump.is_current(request.generation):
                    return
                if solution is None:
                    raise BackendCommandError("ik_unreachable")
                point = build_pvat_point(
                    solution_q=solution,
                    actual_q=snapshot.actual_q,
                    actual_qd=snapshot.actual_qd,
                    previous_qd=self._previous_sent_qd,
                    limits=self._pvat_limits,
                )
                pvat_started = self._clock()
                try:
                    await asyncio.wait_for(
                        client.move_pvat(
                            list(point.q),
                            list(point.qd),
                            list(point.qdd),
                            point.horizon_s,
                        ),
                        timeout=0.06,
                    )
                except TimeoutError:
                    raise BackendCommandError("sdk_timeout:move_pvat") from None
                except Exception:
                    raise BackendCommandError(
                        "sdk_call_failed:move_pvat"
                    ) from None
                pvat_latency_ms = max(
                    0.0,
                    (self._clock() - pvat_started) / 1_000_000,
                )
        except BackendCommandError as error:
            if str(error) in {
                "ik_unreachable",
                "ik_invalid",
                "ik_joint_limit",
                "ik_joint_jump",
                "joint_speed_limit",
            }:
                self._note_soft_constraint(str(error))
                return
            raise
        self._constraint = None
        self._constraint_error = None
        self._consecutive_ik_failures = 0
        self._previous_sent_qd = point.qd
        self._command_id = request.command_id
        self._motion_accepted = True
        await self._emit_event(
            {
                "kind": "pvat_sent",
                "command_id": request.command_id,
                "target_tcp": request.target.model_dump(),
                "p": list(point.q),
                "v": list(point.qd),
                "a": list(point.qdd),
                "horizon_s": point.horizon_s,
                "sdk_latency_ms": pvat_latency_ms,
            }
        )

    def _note_soft_constraint(self, reason: str) -> None:
        self._consecutive_ik_failures += 1
        self._constraint_error = (
            "ik_unreachable" if reason in {"ik_unreachable", "ik_invalid"}
            else reason
        )
        self._constraint = (
            "ik_boundary"
            if self._constraint_error == "ik_unreachable"
            else "joint_boundary"
        )
        if self._consecutive_ik_failures >= 5:
            raise BackendCommandError("ik_failure_persistent")

    async def _emit_event(self, event: dict[str, object]) -> None:
        if self._event_callback is None:
            return
        try:
            await self._event_callback(event, self._clock())
        except Exception:
            raise BackendCommandError("recording_unavailable") from None

    async def _read_snapshot(self) -> LebaiSnapshot:
        client = self._client
        if client is None:
            raise BackendCommandError("robot_disconnected")
        latencies: dict[str, float] = {}
        async with self._sdk_lock:
            connected = await self._timed(
                "is_connected",
                client.is_connected,
                latencies,
            )
            if not connected:
                raise BackendCommandError("robot_disconnected")
            raw_state = await self._timed(
                "get_robot_state",
                client.get_robot_state,
                latencies,
            )
            raw_estop = await self._timed(
                "get_estop_reason",
                client.get_estop_reason,
                latencies,
            )
            raw_kin = await self._timed(
                "get_kin_data",
                client.get_kin_data,
                latencies,
            )
            raw_tcp = await self._timed(
                "get_tcp",
                client.get_tcp,
                latencies,
            )
            raw_claw = await self._timed(
                "get_claw",
                client.get_claw,
                latencies,
            )
            raw_running = await self._timed(
                "get_running_motion",
                client.get_running_motion,
                latencies,
            )
        if not isinstance(raw_kin, Mapping):
            raise BackendCommandError("invalid_sdk_kin_data")
        if not isinstance(raw_tcp, Mapping):
            raise BackendCommandError("invalid_sdk_pose")
        if not isinstance(raw_claw, Mapping):
            raise BackendCommandError("invalid_sdk_claw")
        snapshot = LebaiSnapshot(
            captured_ns=self._clock(),
            robot_state=map_robot_state(raw_state),
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
            running_motion=_running_motion(raw_running),
            sdk_latencies_ms=latencies,
        )
        self._snapshot = snapshot
        await self._emit_kinematics(snapshot)
        return snapshot

    async def _timed(
        self,
        name: str,
        operation: Callable[[], Awaitable[Any]],
        latencies: dict[str, float],
    ) -> Any:
        started = self._clock()
        try:
            return await asyncio.wait_for(operation(), timeout=0.20)
        except TimeoutError:
            raise BackendCommandError(f"sdk_timeout:{name}") from None
        except BackendCommandError:
            raise
        except Exception:
            raise BackendCommandError(f"sdk_call_failed:{name}") from None
        finally:
            latencies[name] = max(0.0, (self._clock() - started) / 1_000_000)

    async def _emit_kinematics(self, snapshot: LebaiSnapshot) -> None:
        if self._event_callback is None:
            return
        event = {
            "kind": "robot_kinematics",
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
