from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose

DEFAULT_R_BX = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


class CoordinateMapper:
    def __init__(
        self,
        translation_scale: float = 0.5,
        rotation_scale: float = 0.5,
        rotation_dead_zone_deg: float = 2.0,
    ) -> None:
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.rotation_dead_zone_rad = np.deg2rad(rotation_dead_zone_deg)
        self.r_bx = DEFAULT_R_BX.copy()
        self._hand_anchor: Pose | None = None
        self._tcp_anchor: Pose | None = None

    def capture(
        self,
        hand: Pose,
        tcp: Pose,
        head_q: tuple[float, float, float, float] | None = None,
    ) -> None:
        world_up = np.array([0.0, 1.0, 0.0])
        head_rotation = Rotation.from_quat(head_q or (0.0, 0.0, 0.0, 1.0))
        user_forward = head_rotation.apply((0.0, 0.0, -1.0))
        user_forward[1] = 0.0
        forward_norm = float(np.linalg.norm(user_forward))
        if not np.isfinite(forward_norm) or forward_norm < 1e-6:
            raise RuntimeError("invalid_control_basis")
        user_forward /= forward_norm
        user_right = np.cross(user_forward, world_up)
        user_right /= np.linalg.norm(user_right)
        user_basis_x = np.column_stack((user_forward, user_right, world_up))
        robot_semantic_basis = np.diag((1.0, -1.0, 1.0))
        r_bx = robot_semantic_basis @ user_basis_x.T
        if (
            not np.allclose(r_bx.T @ r_bx, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(r_bx), 1.0)
        ):
            raise RuntimeError("invalid_control_basis")
        self.r_bx = r_bx
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
        rotation_vector = delta_b.as_rotvec()
        angle = float(np.linalg.norm(rotation_vector))
        effective_angle = max(0.0, angle - self.rotation_dead_zone_rad)
        if effective_angle == 0.0 or angle == 0.0:
            delta_b = Rotation.identity()
        else:
            delta_b = Rotation.from_rotvec(
                rotation_vector / angle * effective_angle * self.rotation_scale
            )
        target_rotation = delta_b * Rotation.from_quat(self._tcp_anchor.q)
        return Pose(p=tuple(p), q=tuple(target_rotation.as_quat()))
