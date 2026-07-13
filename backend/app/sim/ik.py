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
    position = np.asarray(target.p) - np.asarray(current.p)
    rotation = (Rotation.from_quat(target.q) * Rotation.from_quat(current.q).inv()).as_rotvec()
    return np.concatenate([position, rotation])


def _jacobian(q: np.ndarray, model: LM3Model, epsilon: float = 1e-5) -> np.ndarray:
    columns = []
    for index in range(6):
        plus, minus = q.copy(), q.copy()
        plus[index] += epsilon
        minus[index] -= epsilon
        delta = _error(forward_pose(plus, model), forward_pose(minus, model)) / (2 * epsilon)
        columns.append(delta)
    return np.column_stack(columns)


def solve_ik(
    target: Pose, seed_q: Sequence[float], model: LM3Model, max_iterations: int = 30
) -> IKResult:
    q_ref = np.asarray(model.home_q)
    q = np.asarray(seed_q, dtype=float).copy()
    if q.shape != (6,):
        raise IKError("invalid_joint_count")
    if not np.all(np.isfinite(q)):
        raise IKError("ik_singular")
    if np.any(np.abs(q - q_ref) > model.joint_window_rad):
        raise IKError("joint_safety_window")
    for iteration in range(1, max_iterations + 1):
        current = forward_pose(q, model)
        error = _error(target, current)
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = float(np.linalg.norm(error[3:]))
        if position_error <= 0.002 and orientation_error <= np.deg2rad(1.0):
            return IKResult(
                tuple(float(value) for value in q), position_error, orientation_error, iteration
            )
        jacobian = _jacobian(q, model)
        if not np.all(np.isfinite(jacobian)):
            raise IKError("ik_singular")
        damping = 0.04
        delta = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping * damping * np.eye(6), error
        )
        delta = np.clip(delta, -0.12, 0.12)
        candidate = q + delta
        if np.any(np.abs(candidate - q_ref) > model.joint_window_rad):
            raise IKError("joint_safety_window")
        q = candidate
    raise IKError("ik_unreachable")
