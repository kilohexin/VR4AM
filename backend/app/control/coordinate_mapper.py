from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from app.schemas.messages import Pose

DEFAULT_R_BX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


class CoordinateMapper:
    def __init__(self, translation_scale: float = 0.8, rotation_scale: float = 1.0, r_bx: np.ndarray = DEFAULT_R_BX):
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.r_bx = np.asarray(r_bx, dtype=float)
        if not np.allclose(self.r_bx.T @ self.r_bx, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(self.r_bx), 1.0):
            raise ValueError("r_bx must be a proper rotation")
        self._hand_anchor: Pose | None = None
        self._tcp_anchor: Pose | None = None

    def capture(self, hand: Pose, tcp: Pose) -> None:
        self._hand_anchor = hand.model_copy(deep=True)
        self._tcp_anchor = tcp.model_copy(deep=True)

    def clear(self) -> None:
        self._hand_anchor = None
        self._tcp_anchor = None

    def target(self, hand: Pose) -> Pose:
        if self._hand_anchor is None or self._tcp_anchor is None:
            raise RuntimeError("anchor_not_captured")
        p = np.asarray(self._tcp_anchor.p) + self.r_bx @ (
            self.translation_scale * (np.asarray(hand.p) - np.asarray(self._hand_anchor.p))
        )
        r_now = Rotation.from_quat(hand.q).as_matrix()
        r_anchor = Rotation.from_quat(self._hand_anchor.q).as_matrix()
        delta_x = r_now @ r_anchor.T
        delta_b = Rotation.from_matrix(self.r_bx @ delta_x @ self.r_bx.T)
        if self.rotation_scale != 1.0:
            slerp = Slerp([0.0, 1.0], Rotation.concatenate([Rotation.identity(), delta_b]))
            delta_b = slerp([self.rotation_scale])[0]
        target_rotation = delta_b * Rotation.from_quat(self._tcp_anchor.q)
        return Pose(p=tuple(p), q=tuple(target_rotation.as_quat()))
