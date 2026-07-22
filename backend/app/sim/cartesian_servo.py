from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.ik import IKError
from app.sim.kinematics import forward_pose, geometric_jacobian
from app.sim.lm3_model import LM3Model


@dataclass(frozen=True)
class CartesianServoResult:
    q: tuple[float, float, float, float, float, float]
    joint_velocity: tuple[float, float, float, float, float, float]
    position_error_m: float
    orientation_error_rad: float
    joint_limited: bool


def _clip_norm(values: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(values))
    if not np.isfinite(norm):
        raise IKError("ik_singular")
    if norm > maximum:
        return values * (maximum / norm)
    return values


def _damped_pseudoinverse(
    jacobian: np.ndarray,
    *,
    base_damping: float = 0.03,
    singular_threshold: float = 0.08,
) -> np.ndarray:
    try:
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        minimum = float(np.min(singular_values)) if singular_values.size else 0.0
        damping = base_damping + max(0.0, singular_threshold - minimum)
        system = jacobian @ jacobian.T + damping * damping * np.eye(
            jacobian.shape[0]
        )
        result = jacobian.T @ np.linalg.solve(system, np.eye(jacobian.shape[0]))
    except (FloatingPointError, ValueError, np.linalg.LinAlgError) as exc:
        raise IKError("ik_singular") from exc
    if not np.all(np.isfinite(result)):
        raise IKError("ik_singular")
    return result


def _pose_errors(target: Pose, current: Pose) -> tuple[np.ndarray, np.ndarray]:
    try:
        position = np.asarray(target.p, dtype=float) - np.asarray(current.p, dtype=float)
        orientation = (
            Rotation.from_quat(target.q) * Rotation.from_quat(current.q).inv()
        ).as_rotvec()
    except ValueError as exc:
        raise IKError("ik_singular") from exc
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(orientation)):
        raise IKError("ik_singular")
    return position, orientation


def cartesian_servo_step(
    target: Pose,
    actual_q: np.ndarray | tuple[float, ...],
    model: LM3Model,
    *,
    dt: float = 0.02,
) -> CartesianServoResult:
    q = np.asarray(actual_q, dtype=float)
    if q.shape != (6,) or not np.all(np.isfinite(q)) or not np.isfinite(dt) or dt <= 0:
        raise IKError("ik_singular")

    current = forward_pose(q, model)
    position_error, orientation_error = _pose_errors(target, current)
    jacobian = geometric_jacobian(q, model)
    position_jacobian = jacobian[:3]
    rotation_jacobian = jacobian[3:]

    position_command = _clip_norm(4.0 * position_error, 0.30)
    position_inverse = _damped_pseudoinverse(position_jacobian)
    position_velocity = position_inverse @ position_command

    nullspace = np.eye(6) - position_inverse @ position_jacobian
    rotation_command = _clip_norm(2.5 * orientation_error, 1.5)
    remaining_rotation = rotation_command - rotation_jacobian @ position_velocity
    projected_rotation_jacobian = rotation_jacobian @ nullspace
    rotation_velocity = (
        nullspace
        @ _damped_pseudoinverse(projected_rotation_jacobian)
        @ remaining_rotation
    )

    joint_velocity = position_velocity + rotation_velocity
    if not np.all(np.isfinite(joint_velocity)):
        raise IKError("ik_singular")
    peak_speed = float(np.max(np.abs(joint_velocity)))
    if peak_speed > model.max_joint_speed_radps:
        joint_velocity *= model.max_joint_speed_radps / peak_speed

    candidate = q + joint_velocity * dt
    lower = np.asarray(model.home_q) - model.joint_window_rad
    upper = np.asarray(model.home_q) + model.joint_window_rad
    next_q = np.clip(candidate, lower, upper)
    joint_limited = not np.allclose(candidate, next_q, atol=1e-12, rtol=0.0)
    bounded_velocity = (next_q - q) / dt
    solved = forward_pose(next_q, model)
    solved_position_error, solved_orientation_error = _pose_errors(target, solved)
    return CartesianServoResult(
        q=tuple(float(value) for value in next_q),
        joint_velocity=tuple(float(value) for value in bounded_velocity),
        position_error_m=float(np.linalg.norm(solved_position_error)),
        orientation_error_rad=float(np.linalg.norm(solved_orientation_error)),
        joint_limited=joint_limited,
    )
