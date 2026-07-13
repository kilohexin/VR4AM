from collections.abc import Sequence
import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.lm3_model import LM3Model


def _mdh(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    ca, sa, ct, st = np.cos(alpha), np.sin(alpha), np.cos(theta), np.sin(theta)
    return np.array([[ct, -st, 0, a], [st * ca, ct * ca, -sa, -d * sa], [st * sa, ct * sa, ca, d * ca], [0, 0, 0, 1]], dtype=float)


def forward_matrix(q: Sequence[float], model: LM3Model) -> np.ndarray:
    if len(q) != 6:
        raise ValueError("LM3 requires six joints")
    transform = np.eye(4)
    for theta, d, a, alpha in zip(q, model.d_m, model.a_prev_m, model.alpha_prev_rad):
        transform = transform @ _mdh(float(theta), d, a, alpha)
    tcp = np.eye(4)
    tcp[:3, 3] = model.tcp_offset_m
    return transform @ tcp


def forward_pose(q: Sequence[float], model: LM3Model) -> Pose:
    matrix = forward_matrix(q, model)
    return Pose(p=tuple(matrix[:3, 3]), q=tuple(Rotation.from_matrix(matrix[:3, :3]).as_quat()))
