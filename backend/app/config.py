from __future__ import annotations

import os
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "default.yaml"
REAL_ROBOT_CONFIRMATION = "I_UNDERSTAND_REAL_ROBOT_MOTION"
TELEOP_SINGULARITY_GUARD_RAD = math.radians(5.0)

BackendKind = Literal["simulator", "lebai"]
RealRobotMode = Literal["readonly", "control"]


@dataclass(frozen=True)
class TcpExpectation:
    x: float
    y: float
    z: float
    rz: float
    ry: float
    rx: float


@dataclass(frozen=True)
class LebaiControlSettings:
    loop_hz: int
    state_hz: int
    pvat_send_hz: int
    pvat_horizon_s: float
    max_tcp_speed_mps: float
    max_tcp_rotation_radps: float
    max_tcp_acceleration_mps2: float
    max_tcp_angular_acceleration_radps2: float
    max_joint_speed_radps: float
    max_joint_acceleration_radps2: float
    max_joint_step_rad: float
    max_tcp_step_m: float
    max_tcp_rotation_step_deg: float
    max_relative_translation_m: float
    max_relative_rotation_deg: float
    translation_scale: float


@dataclass(frozen=True)
class LebaiGripperSettings:
    max_force_percent: int
    command_hz: int
    open_amplitude_percent: int
    closed_amplitude_percent: int


@dataclass(frozen=True)
class LebaiSettings:
    mode: RealRobotMode
    ip: str
    expected_tcp: TcpExpectation
    home_q: tuple[float, float, float, float, float, float]
    teleop_ready_q: tuple[float, float, float, float, float, float]
    soft_joint_min_rad: tuple[float, float, float, float, float, float]
    soft_joint_max_rad: tuple[float, float, float, float, float, float]
    joint_limit_margin_rad: float
    startup_tcp_min_m: tuple[float, float, float]
    startup_tcp_max_m: tuple[float, float, float]
    tcp_position_tolerance_m: float
    tcp_rotation_tolerance_deg: float
    control: LebaiControlSettings
    gripper: LebaiGripperSettings


@dataclass(frozen=True)
class Settings:
    backend: BackendKind
    lebai: LebaiSettings | None
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
    def load(cls, path: Path | None = None) -> "Settings":
        selected = path or Path(
            os.environ.get("VR4ARM_CONFIG", DEFAULT_CONFIG)
        )
        payload = yaml.safe_load(selected.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise RuntimeError("invalid_config")
        backend = payload.get("backend")
        if backend not in {"simulator", "lebai"}:
            raise RuntimeError("invalid_config:backend")
        lebai = _parse_lebai(payload) if backend == "lebai" else None
        common_payload = payload if backend == "simulator" else _default_payload()
        return cls(
            backend=backend,
            lebai=lebai,
            control_hz=_positive_int(common_payload, "control_hz"),
            state_hz=_positive_int(common_payload, "state_hz"),
            stale_ms=_positive_int(common_payload, "stale_ms"),
            disarm_ms=_positive_int(common_payload, "disarm_ms"),
            max_linear_speed_mps=_positive_float(
                common_payload, "max_linear_speed_mps"
            ),
            max_angular_speed_radps=_positive_float(
                common_payload, "max_angular_speed_radps"
            ),
            max_linear_accel_mps2=_positive_float(
                common_payload, "max_linear_accel_mps2"
            ),
            max_angular_accel_radps2=_positive_float(
                common_payload, "max_angular_accel_radps2"
            ),
            translation_scale=_positive_float(common_payload, "translation_scale"),
            rotation_scale=_positive_float(common_payload, "rotation_scale"),
            workspace_radius_m=_positive_float(
                common_payload, "workspace_radius_m"
            ),
            rotation_dead_zone_deg=_positive_float(
                common_payload, "rotation_dead_zone_deg"
            ),
            constraint_clear_ms=_positive_int(
                common_payload, "constraint_clear_ms"
            ),
            joint_speed_radps=_positive_float(common_payload, "joint_speed_radps"),
            joint_accel_radps2=_positive_float(common_payload, "joint_accel_radps2"),
            joint_window_rad=_positive_float(common_payload, "joint_window_rad"),
            home_joint_speed_radps=_positive_float(
                common_payload, "home_joint_speed_radps"
            ),
            home_timeout_s=_positive_float(common_payload, "home_timeout_s"),
            home_position_tolerance_rad=_positive_float(
                common_payload, "home_position_tolerance_rad"
            ),
            home_velocity_tolerance_radps=_positive_float(
                common_payload, "home_velocity_tolerance_radps"
            ),
            home_stable_ms=_positive_int(common_payload, "home_stable_ms"),
        )


def _default_payload() -> dict[str, Any]:
    payload = yaml.safe_load(DEFAULT_CONFIG.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("invalid_config")
    return payload


def _parse_lebai(payload: dict[str, Any]) -> LebaiSettings:
    real = _mapping(payload, "real_robot")
    mode = real.get("mode")
    if mode not in {"readonly", "control"}:
        raise RuntimeError("invalid_config:mode")
    if (
        mode == "control"
        and os.environ.get("VR4ARM_REAL_ROBOT_CONFIRM")
        != REAL_ROBOT_CONFIRMATION
    ):
        raise RuntimeError("real_robot_confirmation_required")

    ip = real.get("ip")
    if not isinstance(ip, str) or not ip.strip():
        raise RuntimeError("invalid_config:ip")

    tcp_payload = _mapping(real, "expected_tcp")
    expected_tcp = TcpExpectation(
        **{
            key: _finite_float(tcp_payload, key)
            for key in ("x", "y", "z", "rz", "ry", "rx")
        }
    )
    home_q = _finite_tuple(real, "home_q", 6)
    teleop_ready_q = _finite_tuple(real, "teleop_ready_q", 6)
    joint_min = _finite_tuple(real, "soft_joint_min_rad", 6)
    joint_max = _finite_tuple(real, "soft_joint_max_rad", 6)
    margin = _positive_float(real, "joint_limit_margin_rad")
    if any(
        lower + margin >= upper - margin
        for lower, upper in zip(joint_min, joint_max, strict=True)
    ):
        raise RuntimeError("invalid_config:soft_joint_limits")
    if any(
        value < lower + margin or value > upper - margin
        for value, lower, upper in zip(
            home_q,
            joint_min,
            joint_max,
            strict=True,
        )
    ):
        raise RuntimeError("invalid_config:home_q")
    if any(
        value < lower + margin or value > upper - margin
        for value, lower, upper in zip(
            teleop_ready_q,
            joint_min,
            joint_max,
            strict=True,
        )
    ):
        raise RuntimeError("invalid_config:teleop_ready_q")
    if (
        abs(teleop_ready_q[2]) <= TELEOP_SINGULARITY_GUARD_RAD
        or abs(teleop_ready_q[4]) <= TELEOP_SINGULARITY_GUARD_RAD
    ):
        raise RuntimeError("invalid_config:teleop_ready_q_singular")

    startup_min = _finite_tuple(real, "startup_tcp_min_m", 3)
    startup_max = _finite_tuple(real, "startup_tcp_max_m", 3)
    if any(
        lower >= upper
        for lower, upper in zip(startup_min, startup_max, strict=True)
    ):
        raise RuntimeError("invalid_config:startup_tcp_bounds")

    control_payload = _mapping(real, "control")
    control = LebaiControlSettings(
        loop_hz=_positive_int(control_payload, "loop_hz"),
        state_hz=_positive_int(control_payload, "state_hz"),
        pvat_send_hz=_positive_int(control_payload, "pvat_send_hz"),
        pvat_horizon_s=_positive_float(control_payload, "pvat_horizon_s"),
        max_tcp_speed_mps=_positive_float(
            control_payload, "max_tcp_speed_mps"
        ),
        max_tcp_rotation_radps=_positive_float(
            control_payload, "max_tcp_rotation_radps"
        ),
        max_tcp_acceleration_mps2=_positive_float(
            control_payload, "max_tcp_acceleration_mps2"
        ),
        max_tcp_angular_acceleration_radps2=_positive_float(
            control_payload, "max_tcp_angular_acceleration_radps2"
        ),
        max_joint_speed_radps=_positive_float(
            control_payload, "max_joint_speed_radps"
        ),
        max_joint_acceleration_radps2=_positive_float(
            control_payload, "max_joint_acceleration_radps2"
        ),
        max_joint_step_rad=_positive_float(
            control_payload, "max_joint_step_rad"
        ),
        max_tcp_step_m=_positive_float(control_payload, "max_tcp_step_m"),
        max_tcp_rotation_step_deg=_positive_float(
            control_payload, "max_tcp_rotation_step_deg"
        ),
        max_relative_translation_m=_positive_float(
            control_payload, "max_relative_translation_m"
        ),
        max_relative_rotation_deg=_positive_float(
            control_payload, "max_relative_rotation_deg"
        ),
        translation_scale=_positive_float(control_payload, "translation_scale"),
    )
    if control.pvat_send_hz > control.loop_hz:
        raise RuntimeError("invalid_config:pvat_send_hz")
    if control.pvat_horizon_s <= 1 / control.pvat_send_hz:
        raise RuntimeError("invalid_config:pvat_horizon_s")

    gripper_payload = _mapping(real, "gripper")
    gripper = LebaiGripperSettings(
        max_force_percent=_bounded_int(
            gripper_payload,
            "max_force_percent",
            minimum=1,
            maximum=30,
        ),
        command_hz=_positive_int(gripper_payload, "command_hz"),
        open_amplitude_percent=_bounded_int(
            gripper_payload,
            "open_amplitude_percent",
            minimum=0,
            maximum=100,
        ),
        closed_amplitude_percent=_bounded_int(
            gripper_payload,
            "closed_amplitude_percent",
            minimum=0,
            maximum=100,
        ),
    )
    return LebaiSettings(
        mode=mode,
        ip=ip.strip(),
        expected_tcp=expected_tcp,
        home_q=home_q,
        teleop_ready_q=teleop_ready_q,
        soft_joint_min_rad=joint_min,
        soft_joint_max_rad=joint_max,
        joint_limit_margin_rad=margin,
        startup_tcp_min_m=startup_min,
        startup_tcp_max_m=startup_max,
        tcp_position_tolerance_m=_positive_float(
            real, "tcp_position_tolerance_m"
        ),
        tcp_rotation_tolerance_deg=_positive_float(
            real, "tcp_rotation_tolerance_deg"
        ),
        control=control,
        gripper=gripper,
    )


def _positive_float(payload: dict[str, Any], key: str) -> float:
    result = _finite_float(payload, key)
    if result <= 0:
        raise RuntimeError(f"invalid_config:{key}")
    return result


def _positive_int(payload: dict[str, Any], key: str) -> int:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise RuntimeError(f"invalid_config:{key}")
    return value


def _finite_float(payload: dict[str, Any], key: str) -> float:
    value = payload.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"invalid_config:{key}")
    result = float(value)
    if not math.isfinite(result):
        raise RuntimeError(f"invalid_config:{key}")
    return result


def _finite_tuple(
    payload: dict[str, Any],
    key: str,
    length: int,
) -> tuple:
    value = payload.get(key)
    if not isinstance(value, list) or len(value) != length:
        raise RuntimeError(f"invalid_config:{key}")
    result = tuple(
        _finite_float({"value": item}, "value")
        for item in value
    )
    return result


def _mapping(payload: dict[str, Any], key: str) -> dict[str, Any]:
    value = payload.get(key)
    if not isinstance(value, dict):
        raise RuntimeError(f"invalid_config:{key}")
    return value


def _bounded_int(
    payload: dict[str, Any],
    key: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    value = payload.get(key)
    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or value < minimum
        or value > maximum
    ):
        raise RuntimeError(f"invalid_config:{key}")
    return value
