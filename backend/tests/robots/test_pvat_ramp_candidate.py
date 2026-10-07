"""Analytic and synthetic trajectory tests, without SDK or robot connections."""
from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from tests.robots.pvat_ramp_candidate import RampLimits, build_ramp_candidate
from tests.robots.pvat_interpolation_probe import quintic_segment

ZERO = (0.,)*6


def axis(value):
    return (value, *ZERO[1:])


def assert_caps(candidate, limits=RampLimits()):
    velocity, acceleration = candidate.segment.peak_abs_derivatives()
    assert max(velocity) <= limits.speed_cap+1e-10
    assert max(acceleration) <= limits.acceleration_cap+1e-10
    for fraction in np.linspace(0., 1., 101):
        q, _, _ = candidate.segment.state(float(fraction)*limits.duration_s)
        assert np.all(np.array(q) >= np.array(limits.joint_min)-1e-10)
        assert np.all(np.array(q) <= np.array(limits.joint_max)+1e-10)


def test_from_rest_integrates_linear_acceleration_instead_of_terminal_speed():
    candidate = build_ramp_candidate(ZERO, ZERO, ZERO, axis(.1))
    q, v, a = candidate.segment.state(.08)
    assert q[0] == pytest.approx(.2*.08**2/6)
    assert v[0] == pytest.approx(.008)
    assert a[0] == pytest.approx(.2)
    assert candidate.scale == 1.
    assert_caps(candidate)


@pytest.mark.parametrize('fraction', [0., .25, .5, .75, 1.])
def test_ramp_states_follow_independently_integrated_cubic(fraction):
    candidate = build_ramp_candidate(ZERO, axis(.01), axis(.1), axis(.1))
    q1, v1, a1 = candidate.segment.state(.08)
    t = fraction*.08
    jerk = (a1[0]-.1)/.08
    q, v, a = candidate.segment.state(t)
    assert q[0] == pytest.approx(.01*t+.5*.1*t*t+jerk*t**3/6, abs=1e-12)
    assert v[0] == pytest.approx(.01+.1*t+.5*jerk*t*t, abs=1e-12)
    assert a[0] == pytest.approx(.1+jerk*t, abs=1e-12)
    assert_caps(candidate)


def test_speed_reserve_scales_before_the_next_neutral_acceleration_ramp():
    candidate = build_ramp_candidate(ZERO, axis(.032), axis(.2), axis(.1))
    _, velocity, acceleration = candidate.segment.state(.08)
    assert 0 < candidate.scale < 1
    assert velocity[0]+max(acceleration[0], 0)*.08/2 <= .05+1e-10
    assert_caps(candidate)


@pytest.mark.parametrize('sign', [-1., 1.])
def test_full_horizon_sequence_preserves_pva_and_caps_near_the_speed_cap(sign):
    state = (ZERO, ZERO, ZERO)
    previous = None
    for _ in range(25):
        candidate = build_ramp_candidate(*state, axis(sign*.5))
        np.testing.assert_allclose(candidate.segment.state(0.), state, atol=1e-10, rtol=0)
        if previous is not None:
            np.testing.assert_allclose(previous.segment.state(.08), candidate.segment.state(0.), atol=1e-10, rtol=0)
        assert_caps(candidate)
        state = candidate.segment.state(.08)
        previous = candidate
    assert state[0][0]*sign > .07
    # The conservative one-line search is not a maximum-speed optimizer.
    assert .048 <= state[1][0]*sign <= .05+1e-10


@pytest.mark.parametrize('interval', [.02, .047, .063, .078])
def test_overlapping_replans_use_elapsed_state_not_previous_endpoint(interval):
    state = (ZERO, ZERO, ZERO)
    for index in range(60):
        # A target reversal must not snap position, velocity or acceleration.
        target = axis(.5 if index < 30 else -.5)
        candidate = build_ramp_candidate(*state, target)
        np.testing.assert_allclose(candidate.segment.state(0.), state, atol=1e-10, rtol=0)
        assert_caps(candidate)
        state = candidate.segment.state(interval)
    assert state[1][0] < 0


def test_reversal_can_continue_in_the_old_direction_without_a_target_snap():
    candidate = build_ramp_candidate(ZERO, axis(.04), ZERO, axis(-.1))
    q, v, _ = candidate.segment.state(.08)
    assert q[0] > 0 and v[0] > 0
    assert_caps(candidate)


def test_interval_longer_than_horizon_is_rejected_instead_of_extrapolated():
    candidate = build_ramp_candidate(ZERO, ZERO, ZERO, axis(.1))
    with pytest.raises(ValueError, match='invalid_sample_time'):
        candidate.segment.state(.110)


def test_internal_position_extremum_outside_joint_bounds_is_rejected():
    # Initial/terminal q can both be in range while the interior exceeds it.
    limits = replace(RampLimits(), joint_max=axis(.000001))
    with pytest.raises(ValueError, match='no_feasible_ramp'):
        build_ramp_candidate(ZERO, axis(.001), axis(-.2), axis(-.1), limits)


def test_outward_acceleration_at_the_speed_cap_has_no_continuous_solution():
    with pytest.raises(ValueError, match='no_feasible_ramp'):
        build_ramp_candidate(ZERO, axis(.05), axis(.2), axis(-.1))


@pytest.mark.parametrize('q,v,a,target,reason', [
    (ZERO, axis(.051), ZERO, ZERO, 'initial_speed_limit'),
    (ZERO, ZERO, axis(.201), ZERO, 'initial_acceleration_limit'),
    (axis(3.01), ZERO, ZERO, ZERO, 'initial_joint_limit'),
    (ZERO, ZERO, ZERO, axis(3.01), 'target_joint_limit'),
    ((0.,)*5, ZERO, ZERO, ZERO, 'invalid_vector'),
    (ZERO, ZERO, ZERO, axis(float('nan')), 'invalid_vector'),
])
def test_invalid_or_out_of_bounds_states_are_explicitly_rejected(q, v, a, target, reason):
    with pytest.raises(ValueError, match=reason):
        build_ramp_candidate(q, v, a, target)


@pytest.mark.parametrize('changes', [
    {'duration_s': 0.}, {'duration_s': float('nan')}, {'speed_cap': 0.},
    {'acceleration_cap': float('inf')}, {'joint_min': (3.,)*6},
])
def test_invalid_limits_are_rejected(changes):
    with pytest.raises(ValueError, match='invalid_limits'):
        build_ramp_candidate(ZERO, ZERO, ZERO, ZERO, replace(RampLimits(), **changes))


def test_candidate_is_immutable_and_does_not_alias_input_arrays():
    initial = np.zeros(6)
    candidate = build_ramp_candidate(initial, initial, initial, axis(.1))
    initial[:] = 1
    assert candidate.segment.state(0.) == (ZERO, ZERO, ZERO)
    with pytest.raises(FrozenInstanceError):
        candidate.scale = 0.


def test_stationary_target_from_rest_produces_stationary_reference():
    candidate = build_ramp_candidate(ZERO, ZERO, ZERO, ZERO)
    assert candidate.segment.state(.08) == (ZERO, ZERO, ZERO)
    assert_caps(candidate)


@pytest.mark.parametrize('velocity,acceleration', [(0., 0.), (.032, .2)])
def test_neutral_acceleration_reserve_holds_at_every_sample_not_only_the_end(velocity, acceleration):
    candidate = build_ramp_candidate(ZERO, axis(velocity), axis(acceleration), axis(.1))
    for time in np.linspace(0., .08, 1001):
        _, v, a = candidate.segment.state(float(time))
        assert np.max(np.abs(np.array(v)+np.array(a)*.08/2)) <= .05+1e-10


def test_cubic_candidate_matches_independent_six_boundary_quintic_solution():
    candidate = build_ramp_candidate(ZERO, axis(.032), axis(.2), axis(.1))
    initial, terminal = candidate.segment.state(0.), candidate.segment.state(.08)
    independent = quintic_segment(*initial, *terminal, .08)
    np.testing.assert_allclose(candidate.segment.coefficients, independent.coefficients, atol=1e-12, rtol=0)


def test_six_joint_mixed_direction_replans_preserve_bounds_and_initial_pva():
    generator = np.random.default_rng(20261007)
    state = (ZERO, ZERO, ZERO)
    for index in range(40):
        target = tuple(generator.uniform(-.5, .5, 6))
        candidate = build_ramp_candidate(*state, target)
        np.testing.assert_allclose(candidate.segment.state(0.), state, atol=1e-10, rtol=0)
        assert_caps(candidate)
        state = candidate.segment.state((.02, .047, .078)[index % 3])


def test_near_target_crossings_remain_a_characterized_candidate_limitation():
    # Characterization only: caps and PVA continuity do not prove no overshoot.
    state = (ZERO, ZERO, ZERO)
    crossings = 0
    for _ in range(40):
        candidate = build_ramp_candidate(*state, axis(.001))
        following = candidate.segment.state(.08)
        crossings += (state[0][0]-.001)*(following[0][0]-.001) < 0
        state = following
    assert crossings > 0
