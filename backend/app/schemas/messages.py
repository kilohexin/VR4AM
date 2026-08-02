from __future__ import annotations

import json
import math
from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

PROTOCOL_VERSION = 1
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
JointVector = tuple[float, float, float, float, float, float]
RuntimeBackend = Literal["SIMULATOR", "LEBAI", "LEBAI_FAKE"]
FaultResetRejectReason = Literal[
    "no_fault",
    "stop_incomplete",
    "backend_moving",
    "unrecoverable_fault",
    "control_loop_unavailable",
]
ConstraintKind = Literal[
    "workspace_boundary",
    "ik_boundary",
    "joint_boundary",
    "self_collision",
]
RecoveryPhase = Literal["stopping", "homing", "stabilizing"]
RehearsalPhaseName = Literal[
    "identity_preflight",
    "home",
    "arm_and_anchor",
    "translate",
    "rotate",
    "gripper",
    "pick_place",
    "soft_boundary",
    "tracking_loss",
    "recovery_and_home",
    "final_stop",
    "finalize",
]
RehearsalOutcome = Literal["passed", "failed", "aborted"]


class StrictMessage(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


def _finite(values: tuple[float, ...], name: str) -> tuple[float, ...]:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")
    return values


def _normalize_quaternion(value: Quat, name: str = "quaternion") -> Quat:
    _finite(value, name)
    norm = math.sqrt(sum(component * component for component in value))
    if not 0.9 <= norm <= 1.1:
        raise ValueError(f"{name} norm must be between 0.9 and 1.1")
    return tuple(component / norm for component in value)  # type: ignore[return-value]


class Pose(StrictMessage):
    p: Vec3
    q: Quat

    @field_validator("p")
    @classmethod
    def validate_position(cls, value: Vec3) -> Vec3:
        return _finite(value, "position")  # type: ignore[return-value]

    @field_validator("q")
    @classmethod
    def validate_quaternion(cls, value: Quat) -> Quat:
        return _normalize_quaternion(value)


class ControllerState(Pose):
    grip: bool
    trigger: Annotated[float, Field(ge=0.0, le=1.0)]


class VRFrame(StrictMessage):
    v: Literal[1]
    type: Literal["vr_frame"]
    session_id: str = Field(min_length=1, max_length=64)
    seq: int = Field(ge=0)
    client_mono_ms: float = Field(ge=0)
    tracking_valid: bool
    visibility: Literal["visible", "visible-blurred", "hidden"]
    right: ControllerState
    head_q: Quat | None = None

    @field_validator("head_q")
    @classmethod
    def validate_head_quaternion(cls, value: Quat | None) -> Quat | None:
        return None if value is None else _normalize_quaternion(value, "head quaternion")


class TeleopMode(StrEnum):
    DISCONNECTED = "DISCONNECTED"
    READY = "READY"
    ARMED = "ARMED"
    ACTIVE = "ACTIVE"
    HOLD = "HOLD"
    STALE = "STALE"
    FAULT = "FAULT"
    DISARMED = "DISARMED"


class BackendState(StrEnum):
    DISCONNECTED = "DISCONNECTED"
    IDLE = "IDLE"
    MOVING = "MOVING"
    HOLD = "HOLD"
    FAULT = "FAULT"


class RobotStateMessage(StrictMessage):
    v: Literal[1] = 1
    type: Literal["robot_state"] = "robot_state"
    server_mono_ns: int = Field(ge=0)
    ack_seq: int | None = Field(default=None, ge=0)
    mode: TeleopMode
    robot_state: BackendState
    actual_tcp: Pose
    actual_q: JointVector
    gripper: Annotated[float, Field(ge=0.0, le=1.0)]
    sample_age_ms: float | None = Field(default=None, ge=0)
    fault: str | None = None
    constraint: ConstraintKind | None = None
    recovery_phase: RecoveryPhase | None = None
    backend: Literal["SIMULATOR", "LEBAI", "LEBAI_FAKE"] | None = None
    real_robot_mode: Literal["readonly", "control"] | None = None
    preflight_ready: bool | None = None
    preflight_reason: str | None = None


class DiagnosticEvent(StrictMessage):
    event_id: int = Field(ge=1)
    server_mono_ns: int = Field(ge=0)
    kind: str = Field(min_length=1, max_length=64)
    critical: bool
    payload: dict[str, object]

    @field_validator("payload")
    @classmethod
    def validate_payload(cls, value: dict[str, object]) -> dict[str, object]:
        _validate_json_payload(value)
        return value


class DiagnosticsMessage(StrictMessage):
    v: Literal[1] = 1
    type: Literal["diagnostics"] = "diagnostics"
    server_mono_ns: int = Field(ge=0)
    runtime: RuntimeBackend
    hardware_verified: Literal[False] = False
    control_generation: int = Field(ge=0)
    actual_qd: JointVector | None = None
    actual_qdd: JointVector | None = None
    target_q: JointVector | None = None
    target_qd: JointVector | None = None
    target_qdd: JointVector | None = None
    target_tcp: Pose | None = None
    sdk_latencies_ms: dict[str, float]
    pvat_send_hz: float | None = Field(default=None, ge=0)
    log_session_dir: str | None = None
    dropped_events: int = Field(ge=0)
    recent_events: tuple[DiagnosticEvent, ...]

    @field_validator("sdk_latencies_ms")
    @classmethod
    def validate_sdk_latencies(
        cls,
        value: dict[str, float],
    ) -> dict[str, float]:
        for latency in value.values():
            if not math.isfinite(latency) or latency < 0:
                raise ValueError("SDK latencies must be finite and nonnegative")
        return value

    @field_validator("pvat_send_hz")
    @classmethod
    def validate_pvat_send_hz(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("PVAT send rate must be finite")
        return value


def _validate_json_payload(value: object) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("diagnostic payload must contain only finite values")
    if isinstance(value, dict):
        for item in value.values():
            _validate_json_payload(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _validate_json_payload(item)
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("diagnostic payload must be JSON safe") from None


class ClientControlMessage(StrictMessage):
    v: Literal[1]
    type: Literal[
        "hello",
        "arm_request",
        "disarm",
        "reset_fault",
        "home_request",
        "ping",
    ]
    request_id: str = Field(min_length=1, max_length=64)
    client_mono_ms: float | None = Field(default=None, ge=0)


class OfflineRehearsalBeginMessage(StrictMessage):
    v: Literal[1]
    type: Literal["offline_rehearsal_begin"]
    request_id: str = Field(min_length=1, max_length=64)
    plan_version: Literal[1]

    @field_validator("plan_version", mode="before")
    @classmethod
    def validate_plan_version(cls, value: object) -> object:
        if type(value) is not int:
            raise ValueError("plan_version must be the integer 1")
        return value


class OfflineRehearsalPhaseMessage(StrictMessage):
    v: Literal[1]
    type: Literal["offline_rehearsal_phase"]
    request_id: str = Field(min_length=1, max_length=64)
    run_id: str = Field(min_length=1, max_length=64)
    phase: RehearsalPhaseName
    status: Literal["passed", "failed"]
    started_client_ms: float = Field(ge=0)
    completed_client_ms: float = Field(ge=0)
    target: dict[str, object]
    measurements: dict[str, object]
    failure: dict[str, object] | None

    @field_validator("target", "measurements", "failure")
    @classmethod
    def validate_nested_payload(
        cls,
        value: dict[str, object] | None,
    ) -> dict[str, object] | None:
        if value is not None:
            _validate_json_payload(value)
        return value


class OfflineRehearsalFinishMessage(StrictMessage):
    v: Literal[1]
    type: Literal["offline_rehearsal_finish"]
    request_id: str = Field(min_length=1, max_length=64)
    run_id: str = Field(min_length=1, max_length=64)
    outcome: RehearsalOutcome
    failure: dict[str, object] | None

    @field_validator("failure")
    @classmethod
    def validate_failure_payload(
        cls,
        value: dict[str, object] | None,
    ) -> dict[str, object] | None:
        if value is not None:
            _validate_json_payload(value)
        return value


ClientMessage = (
    VRFrame
    | ClientControlMessage
    | OfflineRehearsalBeginMessage
    | OfflineRehearsalPhaseMessage
    | OfflineRehearsalFinishMessage
)
