from __future__ import annotations

import json
import math
from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

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
    "motion_continuity_boundary",
    "tracking_lag",
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


class TargetControlStatus(StrictMessage):
    tool_spin_supported: bool = Field(default=False,strict=True)
    policy: Literal['segmented_experimental']
    phase: Literal['inactive', 'ready', 'awaiting_reply', 'awaiting_window', 'awaiting_feedback', 'fault']
    samples_observed: int = Field(ge=0, strict=True)
    samples_required: int = Field(ge=3, strict=True)
    hardware_verified: Literal[False]

    @field_validator('hardware_verified', mode='before')
    @classmethod
    def require_false_boolean(cls, value):
        if type(value) is not bool or value:
            raise ValueError('hardware_verification_not_established')
        return value

    @model_validator(mode='after')
    def check_counts(self):
        if self.samples_observed > self.samples_required:
            raise ValueError('invalid_target_control_counts')
        return self


class FinitePositionInputMessage(StrictMessage):
    v: Literal[1]
    type: Literal['finite_position_input']
    session_id: str = Field(min_length=1,max_length=128)
    generation: int = Field(strict=True,ge=0,le=2**53-1)
    seq: int = Field(strict=True,ge=0,le=2**53-1)
    kind: Literal['held','release']
    vector: Vec3

    @field_validator('vector',mode='before')
    @classmethod
    def strict_vector(cls,value):
        from app.control.finite_position_types import vector
        return vector(value,3)

    @model_validator(mode='after')
    def release_is_zero(self):
        if self.kind=='release' and any(self.vector):raise ValueError('position_release_vector')
        return self


class FinitePositionStatus(StrictMessage):
    policy: Literal['finite_position_offline','finite_position_web']='finite_position_offline'
    phase: Literal['inactive','ready','planning','executing','draining','hold','fault']
    generation: int
    entered_steps: int
    motion_id: int|None=None
    reason: str|None=None
    hardware_verified: Literal[False]=False


class RobotStateMessage(StrictMessage):
    finite_position: FinitePositionStatus|None=None
    manual_input_ready: bool = Field(default=False,strict=True)
    preparation_required: bool = Field(default=False,strict=True)
    preparation_unlocked: bool = Field(default=False,strict=True)
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
    translation_scale: float | None = Field(default=None, gt=0)
    target_control: TargetControlStatus | None = None


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
    _validate_finite_payload(value)
    # Encode the complete payload once, after the original finite-value walk.
    # Re-encoding each subtree makes high-rate diagnostic validation redundant.
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("diagnostic payload must be JSON safe") from None


def _validate_finite_payload(value: object) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("diagnostic payload must contain only finite values")
    if isinstance(value, dict):
        for item in value.values():
            _validate_finite_payload(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _validate_finite_payload(item)


class GripperRequestMessage(StrictMessage):
    v: Literal[1]
    type: Literal['gripper_request']
    request_id: str = Field(min_length=1,max_length=64)
    value: float = Field(ge=0,le=1,strict=True)


class GripperCancelMessage(StrictMessage):
    v: Literal[1]
    type: Literal['gripper_cancel']
    request_id: str = Field(min_length=1,max_length=64)


class ClientControlMessage(StrictMessage):
    v: Literal[1]
    type: Literal[
        "hello",
        "arm_request",
        "prepare_request",
        "disarm",
        "reset_fault",
        "home_request",
        "ping",
    ]
    request_id: str = Field(min_length=1, max_length=64)
    client_mono_ms: float | None = Field(default=None, ge=0)


SimulationScaleRejectReason = Literal[
    "not_simulation",
    "not_stopped",
    "invalid_scale",
    "automation_active",
]


def _validate_translation_scale(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("translation_scale must be numeric")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("translation_scale must be finite")
    scaled = round(number * 10)
    if (
        not math.isclose(number * 10, scaled, abs_tol=1e-9)
        or not 5 <= scaled <= 100
    ):
        raise ValueError("translation_scale must be 0.5..10.0 in 0.1 steps")
    return scaled / 10


class SetSimulationScaleMessage(StrictMessage):
    v: Literal[1]
    type: Literal["set_simulation_scale"]
    request_id: str = Field(min_length=1, max_length=64)
    translation_scale: float

    @field_validator("translation_scale", mode="before")
    @classmethod
    def validate_translation_scale(cls, value: object) -> float:
        return _validate_translation_scale(value)


class SimulationScaleResultMessage(StrictMessage):
    v: Literal[1] = 1
    type: Literal["simulation_scale_result"] = "simulation_scale_result"
    request_id: str = Field(min_length=1, max_length=64)
    accepted: bool
    translation_scale: float
    reason: SimulationScaleRejectReason | None = None

    @field_validator("translation_scale", mode="before")
    @classmethod
    def validate_translation_scale(cls, value: object) -> float:
        return _validate_translation_scale(value)

    @model_validator(mode="after")
    def validate_outcome(self) -> "SimulationScaleResultMessage":
        if self.accepted == (self.reason is not None):
            raise ValueError("accepted result must omit reason; rejected result requires it")
        return self


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
    | FinitePositionInputMessage
    | GripperRequestMessage
    | GripperCancelMessage
    | SetSimulationScaleMessage
    | ClientControlMessage
    | OfflineRehearsalBeginMessage
    | OfflineRehearsalPhaseMessage
    | OfflineRehearsalFinishMessage
)
