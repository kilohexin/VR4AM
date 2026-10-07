"""Test-only full arrival trajectory; controller interpolation is unknown.

Full plans are reference predictions, not robot feedback. A target change
while moving may require overshoot; a bounded duration search can reject
feasible trajectories outside its family. No jerk/collision/stop guarantee.
"""
from dataclasses import dataclass, replace
from math import comb

import numpy as np

from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_interpolation_probe import QuinticSegment, quintic_segment


@dataclass(frozen=True)
class SettlingLimits:
    motion: RampLimits = RampLimits()
    max_duration_s: float = 60.


@dataclass(frozen=True)
class SettlingPlan:
    segment: QuinticSegment
    target: tuple[float, ...]
    leaves_endpoint_interval: tuple[bool, ...]
    limits: SettlingLimits

    @property
    def duration_s(self):
        return self.segment.duration_s

    def state(self, elapsed_s):
        _time(elapsed_s)
        if elapsed_s >= self.duration_s:
            return self.target, (0.,)*6, (0.,)*6
        return self.segment.state(elapsed_s)

    def window(self, elapsed_s, horizon_s):
        _time(elapsed_s)
        if not np.isfinite(horizon_s) or horizon_s <= 0:
            raise ValueError('invalid_duration')
        if elapsed_s >= self.duration_s:
            raise ValueError('plan_complete')
        duration = min(float(horizon_s), self.duration_s-elapsed_s)
        start, scale = elapsed_s/self.duration_s, duration/self.duration_s
        # Exact polynomial substitution, not interpolation between samples.
        coefficients = tuple(tuple(sum(row[j]*comb(j, k)*start**(j-k)*scale**k
                                       for j in range(k, 6)) for k in range(6))
                             for row in self.segment.coefficients)
        return QuinticSegment(coefficients, duration)


def build_settling_plan(q0, v0, a0, target, limits=SettlingLimits()):
    q0, v0, a0, target = (_vector(x) for x in (q0, v0, a0, target))
    limits = _limits(limits)
    motion = limits.motion
    if np.max(np.abs(v0)) > motion.speed_cap+1e-12:
        raise ValueError('initial_speed_limit')
    if np.max(np.abs(a0)) > motion.acceleration_cap+1e-12:
        raise ValueError('initial_acceleration_limit')
    joint_min, joint_max = np.array(motion.joint_min), np.array(motion.joint_max)
    if np.any(q0 < joint_min) or np.any(q0 > joint_max):
        raise ValueError('initial_joint_limit')
    if np.any(target < joint_min) or np.any(target > joint_max):
        raise ValueError('target_joint_limit')
    duration = motion.duration_s
    if np.all(v0 == 0) and np.all(a0 == 0):
        distance = float(np.max(np.abs(target-q0)))
        duration = max(duration, 1.875*distance/motion.speed_cap,
                       np.sqrt((10/np.sqrt(3))*distance/motion.acceleration_cap))
    if duration > limits.max_duration_s:
        raise ValueError('no_feasible_settling_plan')
    while True:
        segment = quintic_segment(q0, v0, a0, target, np.zeros(6), np.zeros(6), duration)
        speed, acceleration = segment.peak_abs_derivatives()
        minimum, maximum = _position_range(segment)
        if (max(speed) <= motion.speed_cap+1e-12
                and max(acceleration) <= motion.acceleration_cap+1e-12
                and np.all(minimum >= joint_min-1e-12) and np.all(maximum <= joint_max+1e-12)):
            np.testing.assert_allclose(segment.state(duration), (target, np.zeros(6), np.zeros(6)), atol=1e-10, rtol=0)
            crossing = (minimum < np.minimum(q0, target)-1e-12) | (maximum > np.maximum(q0, target)+1e-12)
            return SettlingPlan(segment, tuple(float(x) for x in target), tuple(bool(x) for x in crossing), limits)
        if duration >= limits.max_duration_s:
            raise ValueError('no_feasible_settling_plan')
        duration = min(duration*1.25, limits.max_duration_s)


def retarget_settling_plan(plan, elapsed_s, target, limits=SettlingLimits()):
    state = plan.state(elapsed_s)
    target = _vector(target)
    limits = _limits(limits)
    if tuple(float(x) for x in target) == plan.target and limits == plan.limits:
        return plan, float(elapsed_s)
    return build_settling_plan(*state, target, limits), 0.


def _position_range(segment):
    lower, upper = [], []
    for row in segment.coefficients:
        roots = np.polynomial.polynomial.polyroots(np.polynomial.polynomial.polyder(row))
        times = [0., 1.]
        times.extend(float(np.clip(root.real, 0., 1.)) for root in roots
                     if abs(root.imag) <= 1e-10 and -1e-12 <= root.real <= 1+1e-12)
        values = np.polynomial.polynomial.polyval(times, row)
        lower.append(float(np.min(values)))
        upper.append(float(np.max(values)))
    return np.array(lower), np.array(upper)


def _limits(limits):
    motion = limits.motion
    joint_min, joint_max = _vector(motion.joint_min), _vector(motion.joint_max)
    values = (motion.duration_s, motion.speed_cap, motion.acceleration_cap, limits.max_duration_s)
    if (not np.all(np.isfinite(values)) or min(values) <= 0
            or limits.max_duration_s < motion.duration_s or not np.all(joint_min < joint_max)):
        raise ValueError('invalid_limits')
    return replace(limits, motion=replace(motion, joint_min=tuple(float(x) for x in joint_min),
                                         joint_max=tuple(float(x) for x in joint_max)))


def _vector(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ValueError('invalid_vector')
    return vector


def _time(value):
    if not np.isfinite(value) or value < 0:
        raise ValueError('invalid_sample_time')
