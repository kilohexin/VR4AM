from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.lm3_model import LM3Model


def _checked_joints(q: Sequence[float]) -> np.ndarray:
    values = np.asarray(q, dtype=float)
    if values.shape != (6,):
        raise ValueError("LM3 requires six joints")
    if not np.all(np.isfinite(values)):
        raise ValueError("LM3 joint values must be finite")
    return values


def _axis_vector(axis: str) -> np.ndarray:
    result = np.zeros(3, dtype=float)
    result["xyz".index(axis)] = 1.0
    return result


def _translation(offset: Sequence[float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(offset, dtype=float)
    return transform


def _rotation(matrix: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = matrix
    return transform


def _chain(
    q: Sequence[float], model: LM3Model
) -> tuple[np.ndarray, tuple[np.ndarray, ...], tuple[np.ndarray, ...]]:
    values = _checked_joints(q)
    transform = np.eye(4)
    origins: list[np.ndarray] = []
    axes_world: list[np.ndarray] = []
    for value, axis, offset in zip(
        values, model.joint_axes, model.joint_offsets_m, strict=True
    ):
        transform = transform @ _translation(offset)
        axis_local = _axis_vector(axis)
        origins.append(transform[:3, 3].copy())
        axes_world.append(transform[:3, :3] @ axis_local)
        transform = transform @ _rotation(
            Rotation.from_rotvec(axis_local * value).as_matrix()
        )
    transform = transform @ _translation(model.tool_root_offset_m)
    transform = transform @ _rotation(
        Rotation.from_quat(model.tool_rotation_xyzw).as_matrix()
    )
    transform = transform @ _translation(model.tcp_offset_m)
    if not np.all(np.isfinite(transform)) or not math.isclose(
        float(np.linalg.det(transform[:3, :3])), 1.0, abs_tol=1e-8
    ):
        raise ValueError("invalid LM3 transform")
    return transform, tuple(origins), tuple(axes_world)


def forward_matrix(q: Sequence[float], model: LM3Model) -> np.ndarray:
    return _chain(q, model)[0]


def forward_pose(q: Sequence[float], model: LM3Model) -> Pose:
    matrix = forward_matrix(q, model)
    return Pose(
        p=tuple(float(value) for value in matrix[:3, 3]),
        q=tuple(float(value) for value in Rotation.from_matrix(matrix[:3, :3]).as_quat()),
    )


def geometric_jacobian(q: Sequence[float], model: LM3Model) -> np.ndarray:
    matrix, origins, axes_world = _chain(q, model)
    tcp = matrix[:3, 3]
    jacobian = np.zeros((6, 6), dtype=float)
    for index, (origin, axis_world) in enumerate(zip(origins, axes_world, strict=True)):
        jacobian[:3, index] = np.cross(axis_world, tcp - origin)
        jacobian[3:, index] = axis_world
    if not np.all(np.isfinite(jacobian)):
        raise ValueError("invalid LM3 jacobian")
    return jacobian
