from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose


class CoordinateMapper:
    def __init__(
        self,
        translation_scale: float = 1.0,
        rotation_scale: float = 1.0,
        rotation_dead_zone_deg: float = 2.0,
    ) -> None:
        if not np.isclose(rotation_scale, 1.0):
            raise ValueError("rotation_scale_must_equal_one")
        self._translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.rotation_dead_zone_rad = np.deg2rad(rotation_dead_zone_deg)
        self._hand_anchor: Pose | None = None
        self._tcp_anchor: Pose | None = None

    @property
    def translation_scale(self) -> float:
        return self._translation_scale

    @property
    def has_anchor(self) -> bool:
        return self._hand_anchor is not None or self._tcp_anchor is not None

    def set_translation_scale(self, value: float) -> None:
        if self.has_anchor:
            raise RuntimeError("translation_scale_requires_no_anchor")
        self._translation_scale = value

    def capture(
        self,
        hand: Pose,
        tcp: Pose,
        head_q: tuple[float, float, float, float] | None = None,
    ) -> None:
        _ = head_q  # Retained for protocol compatibility; deltas already use XR scene axes.
        self._hand_anchor = hand.model_copy(deep=True)
        self._tcp_anchor = tcp.model_copy(deep=True)

    def clear(self) -> None:
        self._hand_anchor = None
        self._tcp_anchor = None

    def target(self, hand: Pose) -> Pose:
        if self._hand_anchor is None or self._tcp_anchor is None:
            raise RuntimeError("anchor_not_captured")
        p = np.asarray(self._tcp_anchor.p) + self._translation_scale * (
            np.asarray(hand.p) - np.asarray(self._hand_anchor.p)
        )
        r_now = Rotation.from_quat(hand.q)
        r_anchor = Rotation.from_quat(self._hand_anchor.q)
        delta_world = r_now * r_anchor.inv()
        rotation_vector = delta_world.as_rotvec()
        angle = float(np.linalg.norm(rotation_vector))
        if angle <= self.rotation_dead_zone_rad:
            delta_world = Rotation.identity()
        target_rotation = delta_world * Rotation.from_quat(self._tcp_anchor.q)
        return Pose(p=tuple(p), q=tuple(target_rotation.as_quat()))
