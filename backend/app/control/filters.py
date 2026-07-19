from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from app.schemas.messages import Pose


class PoseFilter:
    def __init__(self, cutoff_hz: float = 8.0) -> None:
        self.cutoff_hz = cutoff_hz
        self.value: Pose | None = None

    def reset(self, pose: Pose) -> None:
        self.value = pose.model_copy(deep=True)

    def clear(self) -> None:
        self.value = None

    def update(self, pose: Pose, dt: float) -> Pose:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt_must_be_positive_finite")
        if self.value is None:
            self.reset(pose)
            return pose

        alpha = 1 - math.exp(-2 * math.pi * self.cutoff_hz * dt)
        position = np.asarray(self.value.p) + alpha * (
            np.asarray(pose.p) - np.asarray(self.value.p)
        )
        rotations = Rotation.concatenate(
            [Rotation.from_quat(self.value.q), Rotation.from_quat(pose.q)]
        )
        orientation = Slerp([0.0, 1.0], rotations)([alpha])[0].as_quat()
        self.value = Pose(p=tuple(position), q=tuple(orientation))
        return self.value
