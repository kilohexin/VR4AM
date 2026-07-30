from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


class IKError(RuntimeError):
    pass


@dataclass(frozen=True)
class IKResult:
    q: tuple[float, float, float, float, float, float]
    position_error_m: float
    orientation_error_rad: float
    iterations: int


def _error(target: Pose, current: Pose) -> np.ndarray:
    try:
        with np.errstate(all="raise"):
            position = np.asarray(target.p, dtype=float) - np.asarray(current.p, dtype=float)
            rotation = (
                Rotation.from_quat(target.q) * Rotation.from_quat(current.q).inv()
            ).as_rotvec()
            error = np.concatenate([position, rotation])
    except (FloatingPointError, RuntimeWarning, ValueError) as exc:
        raise IKError("ik_singular") from exc
    if not np.all(np.isfinite(error)):
        raise IKError("ik_singular")
    return error


def _norm(values: np.ndarray) -> float:
    try:
        with np.errstate(all="raise"):
            result = float(np.linalg.norm(values))
    except (FloatingPointError, RuntimeWarning, ValueError) as exc:
        raise IKError("ik_singular") from exc
    if not np.isfinite(result):
        raise IKError("ik_singular")
    return result


def _jacobian(q: np.ndarray, model: LM3Model, epsilon: float = 1e-5) -> np.ndarray:
    columns = []
    for index in range(6):
        plus, minus = q.copy(), q.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        try:
            with np.errstate(all="raise"):
                delta = _error(forward_pose(plus, model), forward_pose(minus, model)) / (
                    2 * epsilon
                )
        except (FloatingPointError, RuntimeWarning, ValueError) as exc:
            raise IKError("ik_singular") from exc
        if not np.all(np.isfinite(delta)):
            raise IKError("ik_singular")
        columns.append(delta)
    jacobian = np.column_stack(columns)
    if not np.all(np.isfinite(jacobian)):
        raise IKError("ik_singular")
    return jacobian


def solve_ik(
    target: Pose, seed_q: Sequence[float], model: LM3Model, max_iterations: int = 30
) -> IKResult:
    q_ref = np.asarray(model.home_q)
    lower_q = q_ref - model.joint_window_rad
    upper_q = q_ref + model.joint_window_rad
    q = np.asarray(seed_q, dtype=float).copy()
    projected_to_joint_window = False
    if q.shape != (6,):
        raise IKError("invalid_joint_count")
    if not np.all(np.isfinite(q)):
        raise IKError("ik_singular")
    if np.any(np.abs(q - q_ref) > model.joint_window_rad):
        raise IKError("joint_safety_window")
    for iteration in range(1, max_iterations + 1):
        current = forward_pose(q, model)
        error = _error(target, current)
        position_error = _norm(error[:3])
        orientation_error = _norm(error[3:])
        if position_error <= 0.002 and orientation_error <= np.deg2rad(1.0):
            return IKResult(
                tuple(float(value) for value in q), position_error, orientation_error, iteration
            )
        jacobian = _jacobian(q, model)
        if not np.all(np.isfinite(jacobian)):
            raise IKError("ik_singular")
        damping = 0.04
        try:
            with np.errstate(all="raise"):
                system = jacobian @ jacobian.T + damping * damping * np.eye(6)
                delta = jacobian.T @ np.linalg.solve(system, error)
        except (FloatingPointError, RuntimeWarning, ValueError, np.linalg.LinAlgError) as exc:
            raise IKError("ik_singular") from exc
        if not np.all(np.isfinite(delta)):
            raise IKError("ik_singular")
        delta = np.clip(delta, -0.12, 0.12)
        if not np.all(np.isfinite(delta)):
            raise IKError("ik_singular")
        try:
            with np.errstate(all="raise"):
                candidate = q + delta
        except (FloatingPointError, RuntimeWarning, ValueError) as exc:
            raise IKError("ik_singular") from exc
        if not np.all(np.isfinite(candidate)):
            raise IKError("ik_singular")
        if np.any(candidate < lower_q) or np.any(candidate > upper_q):
            projected_to_joint_window = True
            candidate = np.clip(candidate, lower_q, upper_q)
        q = candidate
    at_joint_boundary = np.any(
        np.isclose(np.abs(q - q_ref), model.joint_window_rad, atol=1e-8, rtol=0.0)
    )
    if projected_to_joint_window and at_joint_boundary:
        raise IKError("joint_safety_window")
    raise IKError("ik_unreachable")
