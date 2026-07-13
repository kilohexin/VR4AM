from __future__ import annotations

import math
from enum import StrEnum
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator

PROTOCOL_VERSION = 1
Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
JointVector = tuple[float, float, float, float, float, float]


class StrictMessage(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)


def _finite(values: tuple[float, ...], name: str) -> tuple[float, ...]:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain only finite values")
    return values


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
        _finite(value, "quaternion")
        norm = math.sqrt(sum(component * component for component in value))
        if not 0.9 <= norm <= 1.1:
            raise ValueError("quaternion norm must be between 0.9 and 1.1")
        return tuple(component / norm for component in value)  # type: ignore[return-value]


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


class ClientControlMessage(StrictMessage):
    v: Literal[1]
    type: Literal["hello", "arm_request", "disarm", "ping"]
    request_id: str = Field(min_length=1, max_length=64)
    client_mono_ms: float | None = Field(default=None, ge=0)
