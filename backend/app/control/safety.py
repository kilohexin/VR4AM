from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose


class SafetyViolation(RuntimeError):
    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


class SafetyLimiter:
    def __init__(
        self,
        anchor: tuple[float, float, float] | None = None,
        max_linear_speed: float = 0.15,
        max_angular_speed: float = 0.6,
        max_linear_accel: float = 0.4,
        max_angular_accel: float = 1.2,
        envelope: float = 0.25,
    ) -> None:
        self.anchor = np.asarray(anchor, dtype=float) if anchor is not None else None
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.envelope = envelope
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def set_anchor(self, anchor: tuple[float, float, float]) -> None:
        self.anchor = np.asarray(anchor, dtype=float)
        self.linear_velocity[:] = 0
        self.angular_velocity[:] = 0

    def limit(self, previous: Pose, requested: Pose, dt: float) -> Pose:
        target = np.asarray(requested.p, dtype=float)
        if self.anchor is not None and np.any(np.abs(target - self.anchor) > self.envelope):
            raise SafetyViolation("workspace_violation")

        start = np.asarray(previous.p, dtype=float)
        desired_velocity = (target - start) / dt
        speed = float(np.linalg.norm(desired_velocity))
        if speed > self.max_linear_speed:
            desired_velocity *= self.max_linear_speed / speed

        velocity_delta = desired_velocity - self.linear_velocity
        velocity_delta_norm = float(np.linalg.norm(velocity_delta))
        max_velocity_delta = self.max_linear_accel * dt
        if velocity_delta_norm > max_velocity_delta:
            velocity_delta *= max_velocity_delta / velocity_delta_norm
        self.linear_velocity += velocity_delta
        position = start + self.linear_velocity * dt

        start_rotation = Rotation.from_quat(previous.q)
        requested_rotation = Rotation.from_quat(requested.q)
        relative_rotation = requested_rotation * start_rotation.inv()
        desired_angular_velocity = relative_rotation.as_rotvec() / dt
        angular_speed = float(np.linalg.norm(desired_angular_velocity))
        if angular_speed > self.max_angular_speed:
            desired_angular_velocity *= self.max_angular_speed / angular_speed

        angular_velocity_delta = desired_angular_velocity - self.angular_velocity
        angular_velocity_delta_norm = float(np.linalg.norm(angular_velocity_delta))
        max_angular_velocity_delta = self.max_angular_accel * dt
        if angular_velocity_delta_norm > max_angular_velocity_delta:
            angular_velocity_delta *= max_angular_velocity_delta / angular_velocity_delta_norm
        self.angular_velocity += angular_velocity_delta
        limited_rotation = Rotation.from_rotvec(self.angular_velocity * dt) * start_rotation

        return Pose(p=tuple(position), q=tuple(limited_rotation.as_quat()))
