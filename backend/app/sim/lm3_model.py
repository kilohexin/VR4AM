from __future__ import annotations

import hashlib
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


def model_config_sha256(path: Path = MODEL_CONFIG) -> str:
    """Fingerprint model bytes, normalizing only Windows CRLF to LF.

    Keep parameter, whitespace and all other byte changes detectable while
    preserving compatibility with fixtures generated from Git's LF content.
    """
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()

CHAIN_POINT_NAMES = frozenset(
    {
        "base",
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "tcp",
    }
)


@dataclass(frozen=True)
class CollisionSegment:
    name: str
    start: str
    end: str
    radius_m: float


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
    collision = payload.get("collision")
    if (
        not isinstance(joints, list)
        or len(joints) != 6
        or not isinstance(tool, dict)
        or not isinstance(collision, dict)
    ):
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
    _positive_float(collision, "safety_margin_m")
    segments = collision.get("segments")
    check_pairs = collision.get("check_pairs")
    if not isinstance(segments, list) or not isinstance(check_pairs, list):
        raise RuntimeError("invalid_visual_kinematics:collision")
    segment_names: set[str] = set()
    for segment in segments:
        if not isinstance(segment, dict):
            raise RuntimeError("invalid_visual_kinematics:collision_segment")
        name = segment.get("name")
        start = segment.get("start")
        end = segment.get("end")
        if (
            not isinstance(name, str)
            or not name
            or name in segment_names
            or start not in CHAIN_POINT_NAMES
            or end not in CHAIN_POINT_NAMES
            or start == end
        ):
            raise RuntimeError("invalid_visual_kinematics:collision_segment")
        _positive_float(segment, "radius_m")
        segment_names.add(name)
    seen_pairs: set[frozenset[str]] = set()
    for pair in check_pairs:
        if (
            not isinstance(pair, list)
            or len(pair) != 2
            or not all(isinstance(name, str) for name in pair)
            or pair[0] == pair[1]
            or pair[0] not in segment_names
            or pair[1] not in segment_names
        ):
            raise RuntimeError("invalid_visual_kinematics:collision_pair")
        unordered_pair = frozenset(pair)
        if unordered_pair in seen_pairs:
            raise RuntimeError("invalid_visual_kinematics:collision_pair")
        seen_pairs.add(unordered_pair)
    return payload


_CONFIG = _load_config()
_JOINTS = _CONFIG["joints"]
_TOOL = _CONFIG["tool"]
_COLLISION = _CONFIG["collision"]
_COLLISION_SEGMENTS = tuple(
    CollisionSegment(
        name=str(segment["name"]),
        start=str(segment["start"]),
        end=str(segment["end"]),
        radius_m=_positive_float(segment, "radius_m"),
    )
    for segment in _COLLISION["segments"]
)
_COLLISION_CHECK_PAIRS = tuple(
    (str(pair[0]), str(pair[1])) for pair in _COLLISION["check_pairs"]
)


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
    collision_segments: tuple[CollisionSegment, ...] = _COLLISION_SEGMENTS
    collision_check_pairs: tuple[tuple[str, str], ...] = (
        _COLLISION_CHECK_PAIRS
    )
    collision_safety_margin_m: float = _positive_float(
        _COLLISION, "safety_margin_m"
    )
