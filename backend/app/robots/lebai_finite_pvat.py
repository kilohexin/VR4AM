"""Independent finite PVAT constructor; not selected by production control.

The declared quintic reference starts at the supplied PVA and ends at one
common fraction of the IK target with zero velocity and acceleration. Every
returned point describes a complete window, not the prefix of a longer plan.

Subdivided Bernstein hulls bound the whole polynomial, without sampled-only
checks or assuming fraction zero is feasible. The bounds are sufficient and
can reject feasible curves outside this family. This does not establish
controller interpolation, feedback freshness, Cartesian straightness,
collision clearance, jerk limits, or physical stopping. Rejection is not a
stop. Callers must retain command admission and the original stop policy.
"""
from __future__ import annotations

from dataclasses import dataclass
from numbers import Real

import numpy as np

from app.robots.base import BackendCommandError
from app.robots.lebai_pvat import PvatLimits, PvatPoint
from app.schemas.messages import JointVector

_NUMERIC_TOLERANCE = 1e-12
_HULL_SUBDIVISIONS = 4


@dataclass(frozen=True)
class FinitePvatCandidate:
    point: PvatPoint
    progress_fraction: float
    feasible_fraction_min: float
    peak_speed_bound: JointVector
    peak_acceleration_bound: JointVector
    position_min_bound: JointVector
    position_max_bound: JointVector


def _six(value: object, reason: str) -> np.ndarray:
    try:
        if any(isinstance(x, (bool, np.bool_)) for x in value):
            raise ValueError
        result = np.array(value, dtype=float, copy=True)
    except (ValueError, TypeError, OverflowError):
        raise BackendCommandError(reason) from None
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise BackendCommandError(reason)
    return result


def _tuple(value: np.ndarray) -> JointVector:
    return tuple(float(x) for x in value)  # type: ignore[return-value]


def _split(points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    levels = [points]
    while len(levels[-1]) > 1:
        last = levels[-1]
        levels.append(.5 * last[:-1] + .5 * last[1:])
    return np.stack([level[0] for level in levels]), np.stack([level[-1] for level in reversed(levels)])


def _hull_matrix(degree: int) -> np.ndarray:
    blocks = [np.eye(degree + 1)]
    for _ in range(_HULL_SUBDIVISIONS):
        blocks = [child for block in blocks for child in _split(block)]
    matrix = np.concatenate(blocks)
    matrix.setflags(write=False)
    return matrix


_HULL_MATRICES = {degree: _hull_matrix(degree) for degree in (3, 4, 5)}


def _hull(points: np.ndarray) -> np.ndarray:
    # Fixed dyadic convex weights: no subdivision work at command construction.
    return _HULL_MATRICES[len(points) - 1] @ points


def _controls(velocity: np.ndarray, acceleration: np.ndarray, delta: np.ndarray,
              time_s: float) -> tuple[np.ndarray, np.ndarray]:
    # Relative to actual q avoids differentiating nearly equal absolute angles.
    base = np.zeros((6, 6))
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            base[1] = velocity * (time_s / 5)
            base[2] = velocity * (2 * time_s / 5) + acceleration * (time_s ** 2 / 20)
    except FloatingPointError:
        raise BackendCommandError("finite_pvat_numeric_range") from None
    slope = np.zeros((6, 6))
    slope[3:] = delta
    return base, slope


def _families(base: np.ndarray, slope: np.ndarray, time_s: float):
    try:
        with np.errstate(over="raise", invalid="raise", divide="raise"):
            result = (
                (_hull(base), _hull(slope)),
                (_hull(np.diff(base, axis=0) * (5 / time_s)), _hull(np.diff(slope, axis=0) * (5 / time_s))),
                (_hull(np.diff(base, n=2, axis=0) * (20 / time_s ** 2)),
                 _hull(np.diff(slope, n=2, axis=0) * (20 / time_s ** 2))),
            )
    except FloatingPointError:
        raise BackendCommandError("finite_pvat_numeric_range") from None
    if any(not np.all(np.isfinite(array)) for pair in result for array in pair):
        raise BackendCommandError("finite_pvat_numeric_range")
    return result


def _fraction_interval(families, lower_bounds, upper_bounds) -> tuple[float, float]:
    lower, upper = 0., 1.
    for (base, slope), low, high in zip(families, lower_bounds, upper_bounds):
        fixed = slope == 0
        if np.any(fixed & ((base < low - _NUMERIC_TOLERANCE) | (base > high + _NUMERIC_TOLERANCE))):
            raise BackendCommandError("finite_pvat_no_feasible_window")
        moving = ~fixed
        lows = np.broadcast_to(low, base.shape)[moving]
        highs = np.broadcast_to(high, base.shape)[moving]
        if not np.any(moving):
            continue
        with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
            ends_a = (lows - base[moving]) / slope[moving]
            ends_b = (highs - base[moving]) / slope[moving]
        if np.any(np.isnan(ends_a)) or np.any(np.isnan(ends_b)):
            raise BackendCommandError("finite_pvat_numeric_range")
        lower = max(lower, float(np.max(np.minimum(ends_a, ends_b))))
        upper = min(upper, float(np.min(np.maximum(ends_a, ends_b))))
        if lower > upper:
            raise BackendCommandError("finite_pvat_no_feasible_window")
    return lower, upper


def build_finite_pvat_candidate(
    solution_q: object, actual_q: object, actual_qd: object, actual_qdd: object,
    limits: PvatLimits,
) -> FinitePvatCandidate:
    """Largest common target fraction certified by a fixed 80-ms-or-less hull.

    The duration is never extended, and observed velocity/acceleration are
    never clipped or replaced by a previous command. Bounds apply to the
    entire declared curve. A feasible interval can have a positive lower end.
    """
    invalid = "invalid_finite_pvat_vector"
    solution, actual, velocity, acceleration = (_six(x, invalid) for x in
                                              (solution_q, actual_q, actual_qd, actual_qdd))
    invalid_limits = "invalid_finite_pvat_limits"
    if not isinstance(limits, PvatLimits):
        raise BackendCommandError(invalid_limits)
    minimum = _six(limits.soft_joint_min_rad, invalid_limits)
    maximum = _six(limits.soft_joint_max_rad, invalid_limits)
    values = (limits.horizon_s, limits.max_joint_speed_radps,
              limits.max_joint_acceleration_radps2, limits.max_joint_tracking_error_rad)
    if (any(isinstance(v, (bool, np.bool_)) or not isinstance(v, Real) for v in values)
            or not np.all(np.isfinite(values)) or min(values) <= 0
            or not .01 <= limits.horizon_s <= .08 or not np.all(minimum < maximum)):
        raise BackendCommandError(invalid_limits)
    if np.any(actual < minimum) or np.any(actual > maximum):
        raise BackendCommandError("finite_pvat_actual_joint_limit")
    if np.any(solution < minimum) or np.any(solution > maximum):
        raise BackendCommandError("finite_pvat_target_joint_limit")
    speed_cap, acceleration_cap = values[1:3]
    if max(abs(velocity)) > speed_cap:
        raise BackendCommandError("finite_pvat_initial_speed_limit")
    if max(abs(acceleration)) > acceleration_cap:
        raise BackendCommandError("finite_pvat_initial_acceleration_limit")
    with np.errstate(over="ignore", invalid="ignore"):
        delta = solution - actual
        position_low, position_high = minimum - actual, maximum - actual
    if not np.all(np.isfinite((delta, position_low, position_high))):
        raise BackendCommandError("finite_pvat_numeric_range")
    if max(abs(delta)) > limits.max_joint_tracking_error_rad:
        raise BackendCommandError("ik_tracking_diverged")
    time_s = float(limits.horizon_s)
    base, slope = _controls(velocity, acceleration, delta, time_s)
    families = tuple(_families(base, slope, time_s))
    lows = (position_low, -speed_cap, -acceleration_cap)
    highs = (position_high, speed_cap, acceleration_cap)
    lower, fraction = _fraction_interval(families, lows, highs)
    endpoint = solution.copy() if fraction == 1 else actual + fraction * delta
    # Certify the rounded endpoint which will actually be placed in the packet.
    _, rounded_slope = _controls(velocity, acceleration, endpoint - actual, time_s)
    actual_hulls = [a + b for a, b in _families(base, rounded_slope, time_s)]
    if any(np.any(hull < low - _NUMERIC_TOLERANCE) or np.any(hull > high + _NUMERIC_TOLERANCE)
           for hull, low, high in zip(actual_hulls, lows, highs)):
        raise BackendCommandError("finite_pvat_numeric_range")
    point = PvatPoint(_tuple(endpoint), (0.,) * 6, (0.,) * 6, time_s,
                      _tuple(velocity), "observed_velocity")
    position, speed, accel = actual_hulls
    return FinitePvatCandidate(
        point, fraction, lower, _tuple(np.max(np.abs(speed), axis=0)),
        _tuple(np.max(np.abs(accel), axis=0)),
        _tuple(actual + np.min(position, axis=0)), _tuple(actual + np.max(position, axis=0)),
    )
