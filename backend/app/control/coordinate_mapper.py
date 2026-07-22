from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose

DEFAULT_R_BX = np.eye(3)
DEFAULT_R_TH = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
    ]
)


class CoordinateMapper:
    def __init__(
        self,
        translation_scale: float = 0.5,
        rotation_scale: float = 0.5,
        rotation_dead_zone_deg: float = 2.0,
        controller_to_tool_rotation: np.ndarray | None = None,
    ) -> None:
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.rotation_dead_zone_rad = np.deg2rad(rotation_dead_zone_deg)
        self.r_bx = DEFAULT_R_BX.copy()
        self.r_th = np.asarray(
            DEFAULT_R_TH
            if controller_to_tool_rotation is None
            else controller_to_tool_rotation,
            dtype=float,
        ).copy()
        if (
            self.r_th.shape != (3, 3)
            or not np.all(np.isfinite(self.r_th))
            or not np.allclose(self.r_th.T @ self.r_th, np.eye(3), atol=1e-8)
            or not np.isclose(np.linalg.det(self.r_th), 1.0)
        ):
            raise ValueError("invalid_controller_to_tool_rotation")
        self._hand_anchor: Pose | None = None
        self._tcp_anchor: Pose | None = None

    def capture(
        self,
        hand: Pose,
        tcp: Pose,
        head_q: tuple[float, float, float, float] | None = None,
    ) -> None:
        _ = head_q  # Retained for protocol compatibility; deltas already use XR scene axes.
        self.r_bx = DEFAULT_R_BX.copy()
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
        delta_h = r_anchor.T @ r_now
        delta_t = Rotation.from_matrix(self.r_th @ delta_h @ self.r_th.T)
        rotation_vector = delta_t.as_rotvec()
        angle = float(np.linalg.norm(rotation_vector))
        effective_angle = max(0.0, angle - self.rotation_dead_zone_rad)
        if effective_angle == 0.0 or angle == 0.0:
            delta_t = Rotation.identity()
        else:
            delta_t = Rotation.from_rotvec(
                rotation_vector / angle * effective_angle * self.rotation_scale
            )
        target_rotation = Rotation.from_quat(self._tcp_anchor.q) * delta_t
        return Pose(p=tuple(p), q=tuple(target_rotation.as_quat()))
