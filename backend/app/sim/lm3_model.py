from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any


MODEL_CONFIG = (
    Path(__file__).resolve().parents[3]
    / "config"
    / "lm3_visual_kinematics_v1.json"
)


def _finite_tuple(value: Any, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise RuntimeError(f"invalid_visual_kinematics:{name}")
    result = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in result):
        raise RuntimeError(f"invalid_visual_kinematics:{name}")
    return result


def _positive_float(payload: dict[str, Any], key: str) -> float:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"invalid_visual_kinematics:{key}")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise RuntimeError(f"invalid_visual_kinematics:{key}")
    return result


def _load_config() -> dict[str, Any]:
    payload = json.loads(MODEL_CONFIG.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("version") != 1:
        raise RuntimeError("invalid_visual_kinematics:version")
    joints = payload.get("joints")
    tool = payload.get("tool")
    if not isinstance(joints, list) or len(joints) != 6 or not isinstance(tool, dict):
        raise RuntimeError("invalid_visual_kinematics:structure")
    names = [joint.get("name") for joint in joints if isinstance(joint, dict)]
    axes = [joint.get("axis") for joint in joints if isinstance(joint, dict)]
    if len(names) != 6 or len(set(names)) != 6 or any(axis not in "xyz" for axis in axes):
        raise RuntimeError("invalid_visual_kinematics:joints")
    for index, joint in enumerate(joints):
        _finite_tuple(joint.get("offset_m"), 3, f"joint_{index}_offset")
    _finite_tuple(payload.get("home_q"), 6, "home_q")
    _finite_tuple(tool.get("root_offset_m"), 3, "tool_root_offset")
    rotation = _finite_tuple(tool.get("rotation_xyzw"), 4, "tool_rotation")
    if not 0.9 <= math.sqrt(sum(component * component for component in rotation)) <= 1.1:
        raise RuntimeError("invalid_visual_kinematics:tool_rotation")
    _finite_tuple(tool.get("tcp_offset_m"), 3, "tcp_offset")
    _finite_tuple(tool.get("forward_axis"), 3, "tool_forward")
    for key in (
        "joint_window_rad",
        "max_joint_speed_radps",
        "max_joint_accel_radps2",
    ):
        _positive_float(payload, key)
    return payload


_CONFIG = _load_config()
_JOINTS = _CONFIG["joints"]
_TOOL = _CONFIG["tool"]


@dataclass(frozen=True)
class LM3Model:
    home_q: tuple[float, ...] = _finite_tuple(_CONFIG["home_q"], 6, "home_q")
    joint_names: tuple[str, ...] = tuple(joint["name"] for joint in _JOINTS)
    joint_axes: tuple[str, ...] = tuple(joint["axis"] for joint in _JOINTS)
    joint_offsets_m: tuple[tuple[float, ...], ...] = tuple(
        _finite_tuple(joint["offset_m"], 3, "joint_offset") for joint in _JOINTS
    )
    tool_node: str = str(_TOOL["node"])
    tool_root_offset_m: tuple[float, ...] = _finite_tuple(
        _TOOL["root_offset_m"], 3, "tool_root_offset"
    )
    tool_rotation_xyzw: tuple[float, ...] = _finite_tuple(
        _TOOL["rotation_xyzw"], 4, "tool_rotation"
    )
    tcp_offset_m: tuple[float, ...] = _finite_tuple(
        _TOOL["tcp_offset_m"], 3, "tcp_offset"
    )
    tool_forward_axis: tuple[float, ...] = _finite_tuple(
        _TOOL["forward_axis"], 3, "tool_forward"
    )
    joint_window_rad: float = _positive_float(_CONFIG, "joint_window_rad")
    max_joint_speed_radps: float = _positive_float(
        _CONFIG, "max_joint_speed_radps"
    )
    max_joint_accel_radps2: float = _positive_float(
        _CONFIG, "max_joint_accel_radps2"
    )
