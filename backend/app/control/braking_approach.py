"""Interval-average command approach model; not a physical braking guarantee."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose


class InfeasibleApproach(RuntimeError):
    pass


def ray_velocity(delta, previous_velocity, dt, speed_cap, accel_cap, step_cap):
    step_cap = math.inf if step_cap is None else step_cap
    distance = float(np.linalg.norm(delta))
    budget = accel_cap * dt
    if distance <= 1e-15:
        if np.linalg.norm(previous_velocity) > budget + 1e-12:
            raise InfeasibleApproach('zero_distance_with_velocity')
        return np.zeros(3)
    direction = delta / distance
    parallel = float(np.dot(previous_velocity, direction))
    perpendicular = previous_velocity - parallel * direction
    perpendicular_sq = float(np.dot(perpendicular, perpendicular))
    if perpendicular_sq > budget ** 2 + 1e-15:
        raise InfeasibleApproach('direction_change_exceeds_acceleration')
    radius = math.sqrt(max(0.0, budget ** 2 - perpendicular_sq))
    lower = max(0.0, parallel - radius)
    # s*dt + s²/(2*a) <= distance: reserve a conservative stopping distance
    # after this step. This is a model constraint, not a hardware guarantee.
    brake_speed = 2 * accel_cap * distance / (
        math.sqrt(budget ** 2 + 2 * accel_cap * distance) + budget
    )
    arrival_speed = distance / dt
    # Exact arrival keeps the interval velocity, instead of resetting it.
    # It can reach zero on the next interval, whose model minimum is 20 ms.
    if arrival_speed <= accel_cap * 0.02 + 1e-12:
        brake_speed = arrival_speed
    upper = min(speed_cap, step_cap / dt, arrival_speed, brake_speed, parallel + radius)
    if upper < lower - 1e-12:
        raise InfeasibleApproach('no_velocity_in_feasible_interval')
    return direction * max(0.0, upper)


class BrakingApproach:
    def __init__(self, limits):
        self.limits = limits
        self.linear_velocity = np.zeros(3)
        self.angular_velocity = np.zeros(3)

    def reset_motion(self):
        self.linear_velocity[:] = 0
        self.angular_velocity[:] = 0

    def limit_motion(self, previous, requested, dt):
        if not math.isfinite(dt) or not 0.02 <= dt <= 0.08:
            raise ValueError('unmodelled_interval')
        displacement = np.asarray(requested.p) - previous.p
        start_rotation = Rotation.from_quat(previous.q)
        rotation = (Rotation.from_quat(requested.q) * start_rotation.inv()).as_rotvec()
        linear = ray_velocity(
            displacement, self.linear_velocity, dt,
            self.limits.max_linear_speed, self.limits.max_linear_accel,
            self.limits.max_linear_step_m,
        )
        angular = ray_velocity(
            rotation, self.angular_velocity, dt,
            self.limits.max_angular_speed, self.limits.max_angular_accel,
            self.limits.max_angular_step_rad,
        )
        result = Pose(
            p=tuple(np.asarray(previous.p) + linear * dt),
            q=tuple((Rotation.from_rotvec(angular * dt) * start_rotation).as_quat()),
        )
        # Commit both channels only if both requests are feasible.
        self.linear_velocity, self.angular_velocity = linear, angular
        return result
