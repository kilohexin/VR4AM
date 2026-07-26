from __future__ import annotations

import math
from collections.abc import Mapping

from scipy.spatial.transform import Rotation

from app.robots.base import BackendCommandError
from app.schemas.messages import BackendState, JointVector, Pose


_POSE_FIELDS = ("x", "y", "z", "rz", "ry", "rx")
_STATE_CODES = {
    -1: "ERROR",
    0: "DISCONNECTED",
    1: "ESTOP",
    2: "BOOTING",
    3: "ROBOT_OFF",
    4: "ROBOT_ON",
    5: "IDLE",
    6: "PAUSED",
    7: "MOVING",
    8: "UPDATING",
    9: "STARTING",
    10: "STOPPING",
    11: "TEACHING",
    12: "STOP",
}
_FAULT_STATES = {"ERROR", "DISCONNECTED", "ESTOP"}
_ESTOP_CODES = {
    2: "system",
    3: "manual",
    4: "hard_estop",
    5: "collision",
    6: "joint_limit",
    7: "exceed",
    8: "trajectory_error",
    11: "comm_error",
    12: "can_error",
    13: "joint_error",
}


def pose_from_lebai(value: Mapping[str, object]) -> Pose:
    try:
        values = {field: _finite_number(value[field]) for field in _POSE_FIELDS}
        quat = Rotation.from_euler(
            "ZYX",
            [values["rz"], values["ry"], values["rx"]],
        ).as_quat()
        return Pose(
            p=(values["x"], values["y"], values["z"]),
            q=tuple(float(component) for component in quat),
        )
    except (KeyError, TypeError, ValueError):
        raise BackendCommandError("invalid_sdk_pose") from None


def pose_to_lebai(value: Pose) -> dict[str, float]:
    try:
        rz, ry, rx = Rotation.from_quat(value.q).as_euler("ZYX")
        result = {
            "x": _finite_number(value.p[0]),
            "y": _finite_number(value.p[1]),
            "z": _finite_number(value.p[2]),
            "rz": _finite_number(rz),
            "ry": _finite_number(ry),
            "rx": _finite_number(rx),
        }
    except (IndexError, TypeError, ValueError):
        raise BackendCommandError("invalid_sdk_pose") from None
    return result


def joint_vector(value: object, field: str) -> JointVector:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise BackendCommandError(f"invalid_sdk_joint_vector:{field}")
    try:
        result = tuple(_finite_number(component) for component in value)
    except (TypeError, ValueError):
        raise BackendCommandError(
            f"invalid_sdk_joint_vector:{field}"
        ) from None
    return result  # type: ignore[return-value]


def map_robot_state(value: object) -> BackendState:
    if isinstance(value, bool):
        raise BackendCommandError("invalid_sdk_robot_state")
    if isinstance(value, int):
        name = _STATE_CODES.get(value)
    elif isinstance(value, str):
        name = value.strip().upper()
        if name not in set(_STATE_CODES.values()):
            name = None
    else:
        name = None
    if name is None:
        raise BackendCommandError("invalid_sdk_robot_state")
    if name in _FAULT_STATES:
        return BackendState.FAULT
    if name == "IDLE":
        return BackendState.IDLE
    if name == "MOVING":
        return BackendState.MOVING
    return BackendState.HOLD


def estop_fault(value: object) -> str | None:
    if value is None or value == 0:
        return None
    if isinstance(value, bool):
        raise BackendCommandError("invalid_sdk_estop_reason")
    if isinstance(value, int):
        return f"estop:{_ESTOP_CODES.get(value, f'unknown_{value}')}"
    if isinstance(value, str):
        normalized = value.strip().lower().replace(" ", "_")
        if normalized in {"", "0", "none"}:
            return None
        aliases = {
            "hardestop": "hard_estop",
            "jointlimit": "joint_limit",
            "trajectoryerror": "trajectory_error",
            "commerror": "comm_error",
            "canerror": "can_error",
            "jointerror": "joint_error",
        }
        return f"estop:{aliases.get(normalized, normalized)}"
    raise BackendCommandError("invalid_sdk_estop_reason")


def _finite_number(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("not a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("not finite")
    return result
