"""Necessary rest-to-rest parameter bounds; no SDK, model or motion admission.

With |a|<=A and zero initial/final Cartesian velocity,
|v(t)|<=min(V, A*t, A*(T-t)). Integrating this envelope bounds displacement.
Passing does not certify a quintic, a Cartesian path, controller interpolation,
actual execution, physical stopping or permission to move.
"""
import math


def assess_rest_to_rest_parameters(distance_m, duration_s, max_speed_mps, max_acceleration_mps2):
    values = (distance_m, duration_s, max_speed_mps, max_acceleration_mps2)
    try:
        if any(type(value) not in (int, float) for value in values):
            raise ValueError
        distance, duration, speed, acceleration = (float(value) for value in values)
        if any(not math.isfinite(value) or value <= 0 for value in (distance, duration, speed, acceleration)):
            raise ValueError
    except (ValueError, OverflowError):
        raise ValueError('finite_reference_invalid_parameters') from None
    if not .01 <= duration <= .08:
        raise ValueError('finite_reference_horizon_invalid')

    if speed >= acceleration * (duration / 2):
        bound = acceleration * (duration / 4) * duration
    else:
        bound = speed * (duration - speed / acceleration)
    return dict(
        assessment='necessary_parameters_only_not_trajectory_or_hardware_verification',
        requested_distance_m=distance, duration_s=duration,
        max_tcp_speed_mps=speed, max_tcp_acceleration_mps2=acceleration,
        max_rest_to_rest_distance_m=bound,
        necessary_conditions_satisfied=distance <= bound,
        trajectory_verified=False, hardware_verified=False, motion_authorized=False,
    )
