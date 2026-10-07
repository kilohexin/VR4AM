"""Offline mathematical reference only; never imported by robot control.

Quintic interpolation is a declared experiment, not a controller model.
Endpoint bounds are necessary scalar conditions, not sufficient conditions
for a continuous multi-joint trajectory, collision clearance, or safe motion.
"""
from dataclasses import dataclass

import numpy as np


def _vector(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError('invalid_vector')
    return result


def _duration(value):
    if not np.isfinite(value) or value <= 0:
        raise ValueError('invalid_duration')
    return float(value)


def _tuple(value):
    return tuple(float(x) for x in value)


@dataclass(frozen=True)
class QuinticSegment:
    coefficients: tuple[tuple[float, ...], ...]
    duration_s: float

    def state(self, time_s):
        if not np.isfinite(time_s) or not 0 <= time_s <= self.duration_s:
            raise ValueError('invalid_sample_time')
        fraction = time_s/self.duration_s
        result = []
        for derivative in range(3):
            result.append(tuple(float(np.polynomial.polynomial.polyval(
                fraction, np.polynomial.polynomial.polyder(row, m=derivative)
            )/self.duration_s**derivative) for row in self.coefficients))
        return tuple(result)

    def peak_abs_derivatives(self):
        result = []
        for derivative in (1, 2):
            maxima = []
            for row in self.coefficients:
                values = np.polynomial.polynomial.polyder(row, m=derivative)
                critical = np.polynomial.polynomial.polyder(row, m=derivative+1)
                roots = np.polynomial.polynomial.polyroots(critical)
                fractions = [0., 1.]
                for root in roots:
                    if abs(root.imag) <= 1e-10 and -1e-12 <= root.real <= 1+1e-12:
                        fractions.append(float(np.clip(root.real, 0., 1.)))
                maxima.append(max(abs(float(np.polynomial.polynomial.polyval(t, values)))
                                  for t in fractions)/self.duration_s**derivative)
            result.append(tuple(maxima))
        return tuple(result)


def quintic_segment(q0, v0, a0, q1, v1, a1, duration_s):
    """Fit all six PVA boundaries in normalized time to reduce conditioning."""
    t = _duration(duration_s)
    q0, v0, a0, q1, v1, a1 = (_vector(x) for x in (q0, v0, a0, q1, v1, a1))
    c0, c1, c2 = q0, v0*t, .5*a0*t*t
    position = q1-c0-c1-c2
    velocity = v1*t-c1-2*c2
    acceleration = a1*t*t-2*c2
    c3 = 10*position-4*velocity+.5*acceleration
    c4 = -15*position+7*velocity-acceleration
    c5 = 6*position-3*velocity+.5*acceleration
    coefficients = np.stack((c0, c1, c2, c3, c4, c5), axis=1)
    if not np.all(np.isfinite(coefficients)):
        raise ValueError('invalid_coefficients')
    return QuinticSegment(tuple(_tuple(row) for row in coefficients), t)


@dataclass(frozen=True)
class EndpointBounds:
    min_q: tuple[float, ...]
    max_q: tuple[float, ...]
    position_compatible: tuple[bool, ...]
    velocity_compatible: tuple[bool, ...]

    @property
    def compatible(self):
        return all(self.position_compatible) and all(self.velocity_compatible)


def reachable_endpoints(q0, v0, q1, v1, duration_s, speed_cap, acceleration_cap):
    """Integrate extreme capped velocities; terminal velocity checked separately.

Does not constrain jerk, acceleration boundary values, or simultaneous
position/velocity reachability, so passing these bounds is not sufficient.
"""
    t = _duration(duration_s)
    if not np.isfinite(speed_cap) or not np.isfinite(acceleration_cap) or speed_cap <= 0 or acceleration_cap <= 0:
        raise ValueError('invalid_cap')
    q0, v0, q1, v1 = (_vector(x) for x in (q0, v0, q1, v1))
    if np.max(np.abs(v0)) > speed_cap+1e-12:
        raise ValueError('invalid_initial_speed')
    toward_max = np.clip((speed_cap-v0)/acceleration_cap, 0., t)
    toward_min = np.clip((speed_cap+v0)/acceleration_cap, 0., t)
    max_q = q0+v0*toward_max+.5*acceleration_cap*toward_max**2+speed_cap*(t-toward_max)
    min_q = q0+v0*toward_min-.5*acceleration_cap*toward_min**2-speed_cap*(t-toward_min)
    position_ok = (q1 >= min_q-1e-12) & (q1 <= max_q+1e-12)
    velocity_ok = (np.abs(v1-v0) <= acceleration_cap*t+1e-12) & (np.abs(v1) <= speed_cap+1e-12)
    return EndpointBounds(_tuple(min_q), _tuple(max_q), tuple(bool(x) for x in position_ok), tuple(bool(x) for x in velocity_ok))
