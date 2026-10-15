"""Theoretical LM3 region estimates, never hardware or motion admission.

Fixed-tool geometric Jacobian in the base frame. Rotation velocities are zero.
The derivative inequalities cover the model box; ordinary floating point is
not directed-rounding certification. Controller calibration, tracking, startup,
braking and stopping are outside this model. No SDK or network dependencies.
"""
import math

import numpy as np

from app.commissioning.native_jog_mdh import (
    MdhTool, _A, _D, _components, _finite, mdh_tcp_jacobian,
)


def assess_translation_region(q, tool: MdhTool, *, joint_radius_rad,
        initial_offset_rad, duration_s, cartesian_speed_mps,
        cartesian_acceleration_mps2, joint_speed_limit_radps,
        joint_acceleration_limit_radps2):
    """Estimate joint rates over a fixed theoretical joint box.

    initial_offset_rad is a per-axis absolute starting-offset cap from q,
    not a signed displacement. V*t checks only the stated reference interval,
    not delayed entry or braking. Even an eligible result carries no authority.
    See docs/native-cartesian-jog-region-derivation-2026-10-15.md for the
    derivative inequalities and componentwise Neumann-series argument.
    """
    q = _components(q, 6)
    radius = _components(joint_radius_rad, 6)
    offset = _components(initial_offset_rad, 6)
    if type(tool) is not MdhTool:
        raise ValueError('explicit immutable MDH tool required')
    if any(not 0 < r <= .01 or not 0 <= e < r for r, e in zip(radius, offset, strict=True)):
        raise ValueError('mdh_region_geometry_invalid')
    duration = _finite(duration_s)
    if not .01 <= duration <= .08:
        raise ValueError('explicit duration must be 10..80 ms')
    speed, acceleration, joint_speed, joint_acceleration = (
        _finite(v) for v in (cartesian_speed_mps, cartesian_acceleration_mps2,
                            joint_speed_limit_radps, joint_acceleration_limit_radps2))
    if any(v <= 0 for v in (speed, acceleration, joint_speed, joint_acceleration)):
        raise ValueError('positive diagnostic parameters required')

    try:
        with np.errstate(over='raise', invalid='raise', divide='raise'):
            _, j = mdh_tcp_jacobian(q, tool)
            if np.linalg.matrix_rank(j) < 6:
                raise ValueError('mdh_region_singular_reference')
            inverse = np.linalg.solve(j, np.eye(6))
            b = np.linalg.norm(inverse[:, :3], axis=1)
            c = np.linalg.norm(inverse[:, 3:], axis=1)
            lengths = [math.hypot(a, d) for a, d in zip(_A, _D, strict=True)]
            # Link ell translation precedes Rz(q_ell). A second derivative
            # with respect to j,k therefore uses links ell>max(j,k), plus
            # the explicit TCP offset after all six revolute joints.
            suffix = [sum(lengths[i+1:])+math.hypot(*tool.translation_m) for i in range(6)]
            h = np.array([[suffix[max(i, k)] for k in range(6)] for i in range(6)])
            linear_change = h @ np.array(radius)
            angular_change = np.array([sum(radius[:i]) for i in range(6)])
            m = b[:, None]*linear_change[None, :] + c[:, None]*angular_change[None, :]
            eta = float(np.max(m.sum(axis=1)))
            if not math.isfinite(eta) or eta >= 1-1e-12:
                raise ValueError('mdh_region_not_contractive')
            resolvent_system = np.eye(6)-m
            velocity = np.linalg.solve(resolvent_system, b*speed)
            linear_curvature = float(velocity @ h @ velocity)
            angular_curvature = sum(velocity[i]*velocity[k] for i in range(6) for k in range(i))
            accel = np.linalg.solve(resolvent_system,
                b*acceleration+b*linear_curvature+c*angular_curvature)
            excursion = np.array(offset)+velocity*duration
            if (not np.isfinite(velocity).all() or not np.isfinite(accel).all()
                    or not np.isfinite(excursion).all()
                    or (velocity < 0).any() or (accel < 0).any()):
                raise ValueError('mdh_region_nonfinite_or_negative_prediction')
    except (np.linalg.LinAlgError, FloatingPointError, OverflowError):
        raise ValueError('mdh_region_numerical_rejection') from None

    rates_ok = bool((velocity <= joint_speed).all() and (accel <= joint_acceleration).all())
    segment_ok = bool((excursion < np.array(radius)).all())
    return dict(
        scope='theoretical_region_only_not_controller_or_braking',
        model='manufacturer_theoretical_mdh_not_factory_calibration',
        perturbation_contraction=eta,
        joint_velocity_bound_radps=velocity.tolist(),
        joint_acceleration_bound_radps2=accel.tolist(),
        max_reference_excursion_rad=excursion.tolist(),
        model_rates_within_limits=rates_ok, reference_segment_within_region=segment_ok,
        model_reference_eligible=rates_ok and segment_ok,
        directed_rounding=False, factory_calibration_error_bounded=False,
        hardware_verified=False, joint_bounds_verified=False,
        braking_verified=False, continuous_control_admitted=False,
    )
