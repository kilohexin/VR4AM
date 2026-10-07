"""Decelerate command history before retargeting; not a device stop policy."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.control.braking_approach import BrakingApproach, InfeasibleApproach, ray_velocity


def decelerate(velocity, acceleration, dt):
    speed = float(np.linalg.norm(velocity))
    if speed <= acceleration * dt + 1e-15:
        return np.zeros(3)
    return velocity * ((speed - acceleration * dt) / speed)


class BrakeBeforeRetarget(BrakingApproach):
    def __init__(self, limits):
        for key in ('max_linear_speed', 'max_angular_speed', 'max_linear_accel', 'max_angular_accel'):
            value = getattr(limits, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError('invalid_transactional_limits')
        super().__init__(limits)
        self.braking = False
        self.brake_episodes = 0

    def reset_motion(self):
        super().reset_motion()
        self.braking = False

    def limit_motion(self, previous, requested, dt):
        if not math.isfinite(dt) or not 0.02 <= dt <= 0.08:
            raise ValueError('unmodelled_interval')
        limits = self.limits
        start = Rotation.from_quat(previous.q)
        braking = self.braking
        if not braking:
            try:
                linear = ray_velocity(np.asarray(requested.p) - previous.p,
                    self.linear_velocity, dt, limits.max_linear_speed,
                    limits.max_linear_accel, limits.max_linear_step_m)
                angular = ray_velocity((Rotation.from_quat(requested.q) * start.inv()).as_rotvec(),
                    self.angular_velocity, dt, limits.max_angular_speed,
                    limits.max_angular_accel, limits.max_angular_step_rad)
            except InfeasibleApproach:
                braking = True
        if braking:
            # Latch the episode: input updates cannot interrupt deceleration.
            # Both channels reach zero before following the newest request.
            linear = decelerate(self.linear_velocity, limits.max_linear_accel, dt)
            angular = decelerate(self.angular_velocity, limits.max_angular_accel, dt)
        if (np.linalg.norm(linear) > limits.max_linear_speed + 1e-12
                or np.linalg.norm(angular) > limits.max_angular_speed + 1e-12
                or (limits.max_linear_step_m is not None
                    and np.linalg.norm(linear) * dt > limits.max_linear_step_m + 1e-12)
                or (limits.max_angular_step_rad is not None
                    and np.linalg.norm(angular) * dt > limits.max_angular_step_rad + 1e-12)):
            raise InfeasibleApproach('braking_step_exceeds_limits')
        result = Pose(p=tuple(np.asarray(previous.p) + linear * dt),
                      q=tuple((Rotation.from_rotvec(angular * dt) * start).as_quat()))
        # The old velocity can carry a command beyond the projected request.
        # Reject, rather than clip the generated step and corrupt its velocity.
        if limits.project_workspace(result).constrained:
            raise InfeasibleApproach('generated_step_outside_workspace')
        # Atomic commit: neither channel nor phase changes on any rejection.
        self.brake_episodes += int(braking and not self.braking)
        self.braking = braking and (np.linalg.norm(linear) > 0 or np.linalg.norm(angular) > 0)
        self.linear_velocity, self.angular_velocity = linear, angular
        return result
