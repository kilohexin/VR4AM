from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from app.robots.base import BackendCommandError
from app.schemas.messages import JointVector


@dataclass(frozen=True)
class PvatLimits:
    horizon_s: float
    max_joint_speed_radps: float
    max_joint_acceleration_radps2: float
    max_joint_tracking_error_rad: float
    soft_joint_min_rad: JointVector
    soft_joint_max_rad: JointVector


@dataclass(frozen=True)
class PvatPoint:
    q: JointVector
    qd: JointVector
    qdd: JointVector
    horizon_s: float


def build_pvat_point(
    solution_q: object,
    actual_q: JointVector,
    actual_qd: JointVector,
    previous_qd: JointVector | None,
    limits: PvatLimits,
) -> PvatPoint:
    solution = _vector(solution_q, "ik_invalid")
    actual = _vector(actual_q, "invalid_actual_joint_state")
    actual_speed = _vector(actual_qd, "joint_speed_limit")
    reference_speed = _vector(
        previous_qd if previous_qd is not None else actual_speed,
        "joint_speed_limit",
    )
    joint_min = _vector(limits.soft_joint_min_rad, "invalid_pvat_limits")
    joint_max = _vector(limits.soft_joint_max_rad, "invalid_pvat_limits")
    if (
        not np.isfinite(limits.horizon_s)
        or limits.horizon_s <= 0
        or not np.isfinite(limits.max_joint_tracking_error_rad)
        or limits.max_joint_tracking_error_rad <= 0
        or not np.all(joint_min < joint_max)
    ):
        raise BackendCommandError("invalid_pvat_limits")
    if np.any(solution < joint_min) or np.any(solution > joint_max):
        raise BackendCommandError("ik_joint_limit")
    if (
        np.max(np.abs(actual_speed)) > limits.max_joint_speed_radps
        or np.max(np.abs(reference_speed)) > limits.max_joint_speed_radps
    ):
        raise BackendCommandError("joint_speed_limit")

    delta_q = solution - actual
    if np.max(np.abs(delta_q)) > limits.max_joint_tracking_error_rad:
        raise BackendCommandError("ik_tracking_diverged")
    desired_qd = delta_q / limits.horizon_s
    qd = _scale_to_max_abs(
        desired_qd,
        limits.max_joint_speed_radps,
    )

    velocity_delta = qd - reference_speed
    max_velocity_delta = (
        limits.max_joint_acceleration_radps2 * limits.horizon_s
    )
    bounded_delta = _scale_to_max_abs(
        velocity_delta,
        max_velocity_delta,
    )
    qd = np.clip(
        reference_speed + bounded_delta,
        -limits.max_joint_speed_radps,
        limits.max_joint_speed_radps,
    )
    qdd = np.clip(
        bounded_delta / limits.horizon_s,
        -limits.max_joint_acceleration_radps2,
        limits.max_joint_acceleration_radps2,
    )
    bounded_q = actual + qd * limits.horizon_s
    return PvatPoint(
        q=_joint_tuple(bounded_q),
        qd=_joint_tuple(qd),
        qdd=_joint_tuple(qdd),
        horizon_s=float(limits.horizon_s),
    )


def _vector(value: object, reason: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        raise BackendCommandError(reason) from None
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise BackendCommandError(reason)
    return vector


def _scale_to_max_abs(vector: np.ndarray, max_abs: float) -> np.ndarray:
    largest_component = float(np.max(np.abs(vector)))
    if largest_component <= max_abs:
        return vector
    scaled = vector * (max_abs / largest_component)
    return np.clip(scaled, -max_abs, max_abs)


def _joint_tuple(value: np.ndarray) -> JointVector:
    return tuple(float(component) for component in value)  # type: ignore[return-value]
