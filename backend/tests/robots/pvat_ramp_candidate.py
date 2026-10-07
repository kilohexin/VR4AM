"""Test-only continuous acceleration experiment; no controller/plant guarantee.

Accepts explicit initial PVA and integrates a linear acceleration ramp.
Does not solve target arrival, braking distance, collisions, jerk caps,
SDK timing or feedback estimation. Rejection is not a software stop.
"""
from dataclasses import dataclass

import numpy as np

from tests.robots.pvat_interpolation_probe import QuinticSegment


@dataclass(frozen=True)
class RampLimits:
    duration_s: float = .08
    speed_cap: float = .05
    acceleration_cap: float = .2
    joint_min: tuple[float, ...] = (-3.,)*6
    joint_max: tuple[float, ...] = (3.,)*6


@dataclass(frozen=True)
class RampCandidate:
    segment: QuinticSegment
    scale: float


def build_ramp_candidate(q0, v0, a0, target, limits=RampLimits()):
    """Try requested terminal acceleration, then a conservative common blend.

The baseline flips a0 to -a0. Feasibility is convex in terminal acceleration
for the fixed initial state. Bisection only searches that one line, so this
can reject a state for which a different acceleration ramp would be feasible.
Whole-interval speed reserve budgets a ramp from each instantaneous a to
zero in T; it does not reserve joint space or guarantee target arrival.
"""
    q0, v0, a0, target = (_vector(x) for x in (q0, v0, a0, target))
    joint_min, joint_max = (_vector(x) for x in (limits.joint_min, limits.joint_max))
    if (not np.all(np.isfinite((limits.duration_s, limits.speed_cap, limits.acceleration_cap)))
            or min(limits.duration_s, limits.speed_cap, limits.acceleration_cap) <= 0
            or not np.all(joint_min < joint_max)):
        raise ValueError('invalid_limits')
    if np.max(np.abs(v0)) > limits.speed_cap+1e-12:
        raise ValueError('initial_speed_limit')
    if np.max(np.abs(a0)) > limits.acceleration_cap+1e-12:
        raise ValueError('initial_acceleration_limit')
    if np.any(q0 < joint_min) or np.any(q0 > joint_max):
        raise ValueError('initial_joint_limit')
    if np.any(target < joint_min) or np.any(target > joint_max):
        raise ValueError('target_joint_limit')
    t = limits.duration_s
    desired_velocity = (target-q0)/t
    largest = float(np.max(np.abs(desired_velocity)))
    if largest > limits.speed_cap:
        desired_velocity *= limits.speed_cap/largest
    requested = np.clip(2*(desired_velocity-v0)/t-a0, -limits.acceleration_cap, limits.acceleration_cap)
    baseline = -a0

    def candidate(scale):
        terminal_acceleration = baseline+scale*(requested-baseline)
        coefficients = np.stack((q0, v0*t, .5*a0*t*t,
                                 (terminal_acceleration-a0)*t*t/6,
                                 np.zeros(6), np.zeros(6)), axis=1)
        segment = QuinticSegment(tuple(tuple(float(x) for x in row) for row in coefficients), float(t))
        return RampCandidate(segment, float(scale))

    def feasible(scale):
        a1 = baseline+scale*(requested-baseline)
        delta_acceleration = a1-a0
        velocity_min, velocity_max = _quadratic_range(v0, a0*t, .5*delta_acceleration*t)
        if (np.max(np.maximum(np.abs(velocity_min), np.abs(velocity_max))) > limits.speed_cap+1e-12
                or np.max(np.maximum(np.abs(a0), np.abs(a1))) > limits.acceleration_cap+1e-12):
            return False
        # v(t)+a(t)*T/2 budgets a neutral-acceleration ramp at any replan time.
        reserve_min, reserve_max = _quadratic_range(v0+.5*a0*t, .5*(a0+a1)*t, .5*delta_acceleration*t)
        if np.max(np.maximum(np.abs(reserve_min), np.abs(reserve_max))) > limits.speed_cap+1e-12:
            return False
        for index in range(6):
            row = (q0[index], v0[index]*t, .5*a0[index]*t*t, delta_acceleration[index]*t*t/6)
            extrema = [0., 1.]
            roots = np.polynomial.polynomial.polyroots((v0[index], a0[index]*t, .5*delta_acceleration[index]*t))
            extrema.extend(float(np.clip(root.real, 0., 1.)) for root in roots
                           if abs(root.imag) <= 1e-10 and -1e-12 <= root.real <= 1+1e-12)
            positions = np.polynomial.polynomial.polyval(extrema, row)
            if np.min(positions) < joint_min[index]-1e-12 or np.max(positions) > joint_max[index]+1e-12:
                return False
        return True

    if feasible(1.):
        return candidate(1.)
    if not feasible(0.):
        raise ValueError('no_feasible_ramp')
    lower, upper = 0., 1.
    for _ in range(40):
        middle = (lower+upper)/2
        if feasible(middle):
            lower = middle
        else:
            upper = middle
    return candidate(lower)


def _vector(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise ValueError('invalid_vector')
    return vector


def _quadratic_range(c0, c1, c2):
    critical = np.clip(np.divide(-c1, 2*c2, out=np.zeros(6), where=c2 != 0), 0., 1.)
    interior = c0+c1*critical+c2*critical**2
    end = c0+c1+c2
    return np.minimum(np.minimum(c0, end), interior), np.maximum(np.maximum(c0, end), interior)
