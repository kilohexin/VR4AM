from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose


class SafetyViolation(RuntimeError):
    def __init__(self, code: str) -> None:
        self.code = code
        super().__init__(code)


@dataclass(frozen=True)
class WorkspaceProjection:
    pose: Pose
    constrained: bool
    hold: bool = False


class SafetyLimiter:
    def __init__(
        self,
        anchor: tuple[float, float, float] | None = None,
        max_linear_speed: float = 0.15,
        max_angular_speed: float = 0.6,
        max_linear_accel: float = 0.4,
        max_angular_accel: float = 1.2,
        workspace_radius: float = 0.45,
        workspace_half_extent_m: float | None = None,
        max_rotation_from_anchor_rad: float | None = None,
        max_linear_step_m: float | None = None,
        max_angular_step_rad: float | None = None,
    ) -> None:
        for name, value in (
            ("workspace_half_extent_m", workspace_half_extent_m),
            ("max_rotation_from_anchor_rad", max_rotation_from_anchor_rad),
            ("max_linear_step_m", max_linear_step_m),
            ("max_angular_step_rad", max_angular_step_rad),
        ):
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name}_must_be_positive_finite")
        self.anchor = np.asarray(anchor, dtype=float) if anchor is not None else None
        self.anchor_rotation: Rotation | None = None
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_linear_accel = max_linear_accel
        self.max_angular_accel = max_angular_accel
        self.workspace_radius = workspace_radius
        self.workspace_half_extent_m = workspace_half_extent_m
        self.max_rotation_from_anchor_rad = max_rotation_from_anchor_rad
        self.max_linear_step_m = max_linear_step_m
        self.max_angular_step_rad = max_angular_step_rad
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def set_anchor(self, anchor: tuple[float, float, float]) -> None:
        self.anchor = np.asarray(anchor, dtype=float)
        self.anchor_rotation = None
        self.reset_motion()

    def set_pose_anchor(self, anchor: Pose) -> None:
        self.anchor = np.asarray(anchor.p, dtype=float)
        self.anchor_rotation = Rotation.from_quat(anchor.q)
        self.reset_motion()

    def reset_motion(self) -> None:
        self.linear_velocity[:] = 0
        self.angular_velocity[:] = 0

    def clear(self) -> None:
        self.anchor = None
        self.anchor_rotation = None
        self.reset_motion()

    def project_workspace(self, requested: Pose) -> WorkspaceProjection:
        if self.anchor is None:
            return WorkspaceProjection(requested, False)
        target = np.asarray(requested.p, dtype=float)
        displacement = target - self.anchor
        if (
            self.workspace_half_extent_m is not None
            and np.any(
                np.abs(displacement) > self.workspace_half_extent_m + 1e-12
            )
        ):
            return WorkspaceProjection(requested, True, True)
        if (
            self.max_rotation_from_anchor_rad is not None
            and self.anchor_rotation is not None
        ):
            requested_rotation = Rotation.from_quat(requested.q)
            orientation_delta = requested_rotation * self.anchor_rotation.inv()
            if (
                orientation_delta.magnitude()
                > self.max_rotation_from_anchor_rad + 1e-12
            ):
                return WorkspaceProjection(requested, True, True)
        distance = float(np.linalg.norm(displacement))
        if distance <= self.workspace_radius:
            return WorkspaceProjection(requested, False)
        projected = self.anchor + displacement * (self.workspace_radius / distance)
        return WorkspaceProjection(
            requested.model_copy(update={"p": tuple(projected)}),
            True,
        )

    def limit_motion(self, previous: Pose, requested: Pose, dt: float) -> Pose:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt_must_be_positive_finite")

        target = np.asarray(requested.p, dtype=float)
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
        if self.max_linear_step_m is not None:
            step = position - start
            step_norm = float(np.linalg.norm(step))
            if step_norm > self.max_linear_step_m:
                position = start + step * (self.max_linear_step_m / step_norm)

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
        if self.max_angular_step_rad is not None:
            limited_delta = limited_rotation * start_rotation.inv()
            limited_angle = limited_delta.magnitude()
            if limited_angle > self.max_angular_step_rad:
                rotation_axis = limited_delta.as_rotvec() / limited_angle
                limited_rotation = (
                    Rotation.from_rotvec(rotation_axis * self.max_angular_step_rad)
                    * start_rotation
                )

        return Pose(p=tuple(position), q=tuple(limited_rotation.as_quat()))

    def limit(self, previous: Pose, requested: Pose, dt: float) -> Pose:
        """Compatibility wrapper for callers that only need dynamic limiting."""
        return self.limit_motion(previous, requested, dt)
