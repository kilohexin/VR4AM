"""Offline endpoint candidate only, never a robot command or stopping proof.

Constrains endpoint velocity and its horizon-averaged change. It does not
model LEBAI interpolation, interior acceleration, Cartesian straightness,
collision, jerk or physical feedback. Empty intervals reject; callers must
not use repeated rejection as a stationary hold or a verified stop.
"""
import numpy as np

from app.robots.lebai_pvat import PvatPoint


def build_directional_point(solution_q, actual_q, actual_qd, previous_qd, limits):
    def vector(value):
        try:
            result = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            raise ValueError('invalid_direction_vector') from None
        if result.shape != (6,) or not np.all(np.isfinite(result)):
            raise ValueError('invalid_direction_vector')
        return result

    solution, actual, observed = (vector(x) for x in (solution_q, actual_q, actual_qd))
    reference = observed if previous_qd is None else vector(previous_qd)
    minimum, maximum = vector(limits.soft_joint_min_rad), vector(limits.soft_joint_max_rad)
    values = (limits.horizon_s, limits.max_joint_speed_radps,
              limits.max_joint_acceleration_radps2, limits.max_joint_tracking_error_rad)
    if not np.all(np.isfinite(values)) or min(values) <= 0 or not np.all(minimum < maximum):
        raise ValueError('invalid_direction_limits')
    if np.any(solution < minimum) or np.any(solution > maximum):
        raise ValueError('direction_target_joint_limit')
    if np.any(actual < minimum) or np.any(actual > maximum):
        raise ValueError('direction_actual_joint_limit')
    if max(np.max(np.abs(observed)), np.max(np.abs(reference))) > limits.max_joint_speed_radps:
        raise ValueError('direction_speed_limit')
    delta = solution - actual
    if np.max(np.abs(delta)) > limits.max_joint_tracking_error_rad:
        raise ValueError('direction_tracking_limit')
    with np.errstate(over='ignore', invalid='ignore'):
        desired = delta / limits.horizon_s
    budget = limits.max_joint_acceleration_radps2 * limits.horizon_s
    if not np.all(np.isfinite(desired)) or not np.isfinite(budget) or budget <= 0:
        raise ValueError('direction_numeric_range')
    lower, upper = 0., 1.
    # v_i = fraction * desired_i; intersect all six acceleration intervals
    # with no reverse motion, no target overshoot and the original speed cap.
    for direction, speed in zip(desired, reference):
        if direction == 0:
            if abs(speed) > budget:
                raise ValueError('direction_acceleration_infeasible')
            continue
        ends = sorted(((speed - budget) / direction, (speed + budget) / direction))
        lower = max(lower, ends[0])
        upper = min(upper, ends[1], limits.max_joint_speed_radps / abs(direction))
    if lower > upper:
        raise ValueError('direction_acceleration_infeasible')
    velocity = desired * upper
    acceleration = (velocity - reference) / limits.horizon_s
    position = actual + upper * delta
    if not np.all(np.isfinite((position, velocity, acceleration))):
        raise ValueError('direction_numeric_range')
    return PvatPoint(
        q=tuple(float(x) for x in position),
        qd=tuple(float(x) for x in velocity),
        qdd=tuple(float(x) for x in acceleration),
        horizon_s=limits.horizon_s,
        reference_qd=tuple(float(x) for x in reference),
        reference_source='observed_velocity' if previous_qd is None else 'previous_command_velocity',
    )
