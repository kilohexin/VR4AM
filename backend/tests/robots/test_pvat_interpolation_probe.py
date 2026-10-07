"""Synthetic mathematical checks, not controller or physical validation."""
import math

import numpy as np
import pytest

from tests.robots.pvat_interpolation_probe import quintic_segment, reachable_endpoints
from app.robots.lebai_pvat import PvatLimits, build_pvat_point

ZERO = (0.,) * 6


def axis(value):
    return (value, *ZERO[1:])


def test_quintic_preserves_all_six_position_velocity_acceleration_boundaries():
    initial = ((.1, -.2, .3, .4, -.5, .6), (.01, -.02, .03, -.04, .05, -.06), (.1, -.2, .3, -.4, .5, -.6))
    terminal = ((.11, -.19, .31, .41, -.49, .61), (.02, -.01, .04, -.03, .06, -.05), (.2, -.1, .4, -.3, .6, -.5))
    segment = quintic_segment(*initial, *terminal, .08)
    np.testing.assert_allclose(segment.state(0.), initial, atol=1e-12, rtol=0)
    np.testing.assert_allclose(segment.state(.08), terminal, atol=1e-11, rtol=0)


@pytest.mark.parametrize('fraction', [0., .1, .5, .9, 1.])
def test_constant_acceleration_boundaries_reconstruct_the_quadratic(fraction):
    duration, velocity, acceleration = .08, .01, .2
    position = velocity*duration+.5*acceleration*duration**2
    segment = quintic_segment(ZERO, axis(velocity), axis(acceleration), axis(position), axis(velocity+acceleration*duration), axis(acceleration), duration)
    t = fraction*duration
    q, v, a = segment.state(t)
    assert q[0] == pytest.approx(velocity*t+.5*acceleration*t**2, abs=1e-14)
    assert v[0] == pytest.approx(velocity+acceleration*t, abs=1e-14)
    assert a[0] == pytest.approx(acceleration, abs=1e-12)


def test_derivative_roots_find_interior_peaks_with_zero_endpoint_derivatives():
    distance, duration = .001, .08
    segment = quintic_segment(ZERO, ZERO, ZERO, axis(distance), ZERO, ZERO, duration)
    speed, acceleration = segment.peak_abs_derivatives()
    assert speed[0] == pytest.approx(1.875*distance/duration)
    assert acceleration[0] == pytest.approx((10/math.sqrt(3))*distance/duration**2)
    assert speed[1:] == ZERO[1:] and acceleration[1:] == ZERO[1:]


@pytest.mark.parametrize('duration', [.01, .08, .5])
def test_closed_form_coefficients_match_an_independent_linear_system(duration):
    q0, v0, a0 = axis(.1), axis(.02), axis(-.15)
    q1, v1, a1 = axis(.103), axis(-.01), axis(.2)
    segment = quintic_segment(q0, v0, a0, q1, v1, a1, duration)
    equations = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 2, 0, 0, 0],
                          [1, 1, 1, 1, 1, 1], [0, 1, 2, 3, 4, 5], [0, 0, 2, 6, 12, 20]], dtype=float)
    right = np.array([q0, np.array(v0)*duration, np.array(a0)*duration**2,
                      q1, np.array(v1)*duration, np.array(a1)*duration**2])
    independent = np.linalg.solve(equations, right).T
    np.testing.assert_allclose(segment.coefficients, independent, rtol=0, atol=1e-12)


def test_analytic_derivative_peaks_cover_a_dense_independent_sample():
    segment = quintic_segment(ZERO, axis(-.01), axis(.2), axis(.002), axis(.04), axis(-.15), .08)
    speed, acceleration = segment.peak_abs_derivatives()
    states = np.array([segment.state(float(t)) for t in np.linspace(0., .08, 2001)])
    sample_speed = float(np.max(np.abs(states[:, 1, 0])))
    sample_acceleration = float(np.max(np.abs(states[:, 2, 0])))
    assert speed[0] >= sample_speed-1e-12
    assert acceleration[0] >= sample_acceleration-1e-12
    assert speed[0]-sample_speed < 1e-6
    assert acceleration[0]-sample_acceleration < 1e-5


@pytest.mark.parametrize('duration', [0., -.1, float('nan'), float('inf')])
def test_invalid_duration_is_rejected(duration):
    with pytest.raises(ValueError, match='invalid_duration'):
        quintic_segment(ZERO, ZERO, ZERO, ZERO, ZERO, ZERO, duration)
    with pytest.raises(ValueError, match='invalid_duration'):
        reachable_endpoints(ZERO, ZERO, ZERO, ZERO, duration, .05, .2)


@pytest.mark.parametrize('bad', [(0.,)*5, (float('nan'), *ZERO[1:]), (float('inf'), *ZERO[1:])])
def test_invalid_joint_vectors_are_rejected(bad):
    with pytest.raises(ValueError, match='invalid_vector'):
        quintic_segment(ZERO, ZERO, ZERO, bad, ZERO, ZERO, .08)
    with pytest.raises(ValueError, match='invalid_vector'):
        reachable_endpoints(ZERO, bad, ZERO, ZERO, .08, .05, .2)


@pytest.mark.parametrize('time', [-.001, .081, float('nan')])
def test_state_never_silently_extrapolates_outside_the_interval(time):
    segment = quintic_segment(ZERO, ZERO, ZERO, axis(.001), ZERO, ZERO, .08)
    with pytest.raises(ValueError, match='invalid_sample_time'):
        segment.state(time)


def test_position_and_velocity_feasibility_are_separate_conditions():
    bounds = reachable_endpoints(ZERO, ZERO, axis(.00128), axis(.016), .08, .05, .2)
    assert bounds.max_q[0] == pytest.approx(.00064)
    assert bounds.min_q[0] == pytest.approx(-.00064)
    assert bounds.position_compatible[0] is False
    assert bounds.velocity_compatible[0] is True
    assert bounds.compatible is False


def test_constant_acceleration_endpoint_is_inside_the_necessary_bounds():
    bounds = reachable_endpoints(ZERO, ZERO, axis(.00064), axis(.016), .08, .05, .2)
    assert bounds.compatible is True
    assert all(bounds.position_compatible) and all(bounds.velocity_compatible)


def test_terminal_velocity_can_fail_when_position_is_reachable():
    bounds = reachable_endpoints(ZERO, ZERO, axis(.0004), axis(.04), .08, .05, .2)
    assert bounds.position_compatible[0] is True
    assert bounds.velocity_compatible[0] is False
    assert bounds.compatible is False


@pytest.mark.parametrize('sign', [-1., 1.])
def test_position_bounds_include_velocity_saturation_and_sign_symmetry(sign):
    bounds = reachable_endpoints(ZERO, axis(sign*.04), ZERO, ZERO, 1., .05, .2)
    assert bounds.max_q[0] == pytest.approx(.04975 if sign == 1 else .02975)
    assert bounds.min_q[0] == pytest.approx(-.02975 if sign == 1 else -.04975)


@pytest.mark.parametrize('speed,acceleration', [(0., .2), (.05, 0.), (-.1, .2), (.05, float('nan'))])
def test_invalid_caps_are_rejected(speed, acceleration):
    with pytest.raises(ValueError, match='invalid_cap'):
        reachable_endpoints(ZERO, ZERO, ZERO, ZERO, .08, speed, acceleration)


def test_initial_speed_above_the_assumed_cap_is_rejected():
    with pytest.raises(ValueError, match='invalid_initial_speed'):
        reachable_endpoints(ZERO, axis(.051), ZERO, ZERO, .08, .05, .2)


def test_using_observed_velocity_alone_does_not_make_current_position_reachable():
    limits = PvatLimits(.08, .05, .2, .25, (-3.,)*6, (3.,)*6)
    point = build_pvat_point(axis(.1), ZERO, ZERO, None, limits)
    assert point.reference_source == 'observed_velocity'
    assert point.q[0] == pytest.approx(.00128)
    assert point.qd[0] == pytest.approx(.016)
    assert point.qdd[0] == pytest.approx(.2)
    bounds = reachable_endpoints(ZERO, ZERO, point.q, point.qd, .08, .05, .2)
    assert bounds.velocity_compatible[0] is True
    assert bounds.position_compatible[0] is False


def test_trapezoid_position_alone_does_not_limit_quintic_acceleration():
    segment = quintic_segment(ZERO, ZERO, ZERO, axis(.00064), axis(.016), axis(.2), .08)
    assert reachable_endpoints(ZERO, ZERO, axis(.00064), axis(.016), .08, .05, .2).compatible
    _, acceleration = segment.peak_abs_derivatives()
    assert acceleration[0] > .2


def test_declared_cubic_acceleration_ramp_gives_one_compatible_reference():
    # One illustrative interval from rest, no controller/plant/jerk guarantee.
    duration, acceleration_cap = .08, .2
    q1, v1 = axis(acceleration_cap*duration**2/6), axis(acceleration_cap*duration/2)
    segment = quintic_segment(ZERO, ZERO, ZERO, q1, v1, axis(acceleration_cap), duration)
    speed, acceleration = segment.peak_abs_derivatives()
    assert speed[0] == pytest.approx(.008)
    assert acceleration[0] == pytest.approx(.2)
    assert reachable_endpoints(ZERO, ZERO, q1, v1, duration, .05, .2).compatible
