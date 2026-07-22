from __future__ import annotations

import os
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "default.yaml"


@dataclass(frozen=True)
class Settings:
    backend: str
    control_hz: int
    state_hz: int
    stale_ms: int
    disarm_ms: int
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    max_linear_accel_mps2: float
    max_angular_accel_radps2: float
    translation_scale: float
    rotation_scale: float
    workspace_radius_m: float
    rotation_dead_zone_deg: float
    constraint_clear_ms: int
    joint_speed_radps: float
    joint_accel_radps2: float
    joint_window_rad: float
    home_joint_speed_radps: float
    home_timeout_s: float
    home_position_tolerance_rad: float
    home_velocity_tolerance_radps: float
    home_stable_ms: int

    @classmethod
    def load(cls) -> "Settings":
        path = Path(os.environ.get("VR4ARM_CONFIG", DEFAULT_CONFIG))
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise RuntimeError("invalid_config")
        backend = payload.get("backend")
        if backend != "simulator":
            raise RuntimeError("real_robot_disabled")
        return cls(
            backend=backend,
            control_hz=_positive_int(payload, "control_hz"),
            state_hz=_positive_int(payload, "state_hz"),
            stale_ms=_positive_int(payload, "stale_ms"),
            disarm_ms=_positive_int(payload, "disarm_ms"),
            max_linear_speed_mps=_positive_float(payload, "max_linear_speed_mps"),
            max_angular_speed_radps=_positive_float(payload, "max_angular_speed_radps"),
            max_linear_accel_mps2=_positive_float(payload, "max_linear_accel_mps2"),
            max_angular_accel_radps2=_positive_float(payload, "max_angular_accel_radps2"),
            translation_scale=_positive_float(payload, "translation_scale"),
            rotation_scale=_positive_float(payload, "rotation_scale"),
            workspace_radius_m=_positive_float(payload, "workspace_radius_m"),
            rotation_dead_zone_deg=_positive_float(payload, "rotation_dead_zone_deg"),
            constraint_clear_ms=_positive_int(payload, "constraint_clear_ms"),
            joint_speed_radps=_positive_float(payload, "joint_speed_radps"),
            joint_accel_radps2=_positive_float(payload, "joint_accel_radps2"),
            joint_window_rad=_positive_float(payload, "joint_window_rad"),
            home_joint_speed_radps=_positive_float(payload, "home_joint_speed_radps"),
            home_timeout_s=_positive_float(payload, "home_timeout_s"),
            home_position_tolerance_rad=_positive_float(
                payload, "home_position_tolerance_rad"
            ),
            home_velocity_tolerance_radps=_positive_float(
                payload, "home_velocity_tolerance_radps"
            ),
            home_stable_ms=_positive_int(payload, "home_stable_ms"),
        )


def _positive_float(payload: dict[str, Any], key: str) -> float:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"invalid_config:{key}")
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise RuntimeError(f"invalid_config:{key}")
    return result


def _positive_int(payload: dict[str, Any], key: str) -> int:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise RuntimeError(f"invalid_config:{key}")
    return value
