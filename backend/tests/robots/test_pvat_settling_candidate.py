"""Reference trajectory acceptance only, never a physical stop validation."""
from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest

from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import SettlingLimits, build_settling_plan, retarget_settling_plan

ZERO = (0.,)*6


def axis(value):
    return (value, *ZERO[1:])


def assert_limits(plan, limits=SettlingLimits()):
    speed, acceleration = plan.segment.peak_abs_derivatives()
    assert max(speed) <= limits.motion.speed_cap+1e-10
    assert max(acceleration) <= limits.motion.acceleration_cap+1e-10
    for t in np.linspace(0., plan.duration_s, 151):
        q, _, _ = plan.state(float(t))
        assert np.all(np.array(q) >= np.array(limits.motion.joint_min)-1e-10)
        assert np.all(np.array(q) <= np.array(limits.motion.joint_max)+1e-10)


@pytest.mark.parametrize('distance', [-.5, -.001, .001, .5])
def test_from_rest_arrives_without_crossing_and_with_zero_terminal_derivatives(distance):
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(distance))
    samples = np.array([plan.state(float(t)) for t in np.linspace(0., plan.duration_s, 301)])
    positions = samples[:, 0, 0]*np.sign(distance)
    assert np.min(positions) >= -1e-12
    assert np.max(positions) <= abs(distance)+1e-12
    assert np.min(np.diff(positions)) >= -1e-12
    assert plan.leaves_endpoint_interval == (False,)*6
    assert plan.state(plan.duration_s) == (axis(distance), ZERO, ZERO)
    assert_limits(plan)


def test_arrival_duration_from_rest_matches_independent_peak_formulas():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    expected = max(.08, 1.875*.001/.05, np.sqrt((10/np.sqrt(3))*.001/.2))
    assert plan.duration_s == pytest.approx(expected)


@pytest.mark.parametrize('interval', [.02, .047, .063, .078, .08])
def test_unchanged_target_reuses_full_plan_and_finishes_independent_of_cadence(interval):
    target = axis(.001)
    plan = build_settling_plan(ZERO, ZERO, ZERO, target)
    elapsed = 0.
    positions = []
    while elapsed < plan.duration_s:
        reused, cursor = retarget_settling_plan(plan, elapsed, target)
        assert reused is plan and cursor == elapsed
        window = plan.window(elapsed, .08)
        np.testing.assert_allclose(window.state(0.), plan.state(elapsed), atol=1e-12, rtol=0)
        elapsed += interval
        positions.append(plan.state(elapsed)[0][0])
    assert min(positions) >= 0 and max(positions) <= .001
    assert plan.state(elapsed) == (target, ZERO, ZERO)


def test_restricted_windows_exactly_match_the_full_quintic_not_a_refit_approximation():
    plan = build_settling_plan(ZERO, axis(.01), axis(.05), axis(.1))
    window = plan.window(.1, .08)
    for t in np.linspace(0., window.duration_s, 21):
        np.testing.assert_allclose(window.state(float(t)), plan.state(.1+float(t)), atol=1e-10, rtol=0)
    assert_limits(plan)


def test_last_window_shortens_to_arrival_then_reference_holds_without_extrapolation():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    window = plan.window(plan.duration_s-.01, .08)
    assert window.duration_s == pytest.approx(.01)
    np.testing.assert_allclose(window.state(.01), (axis(.001), ZERO, ZERO), atol=1e-9, rtol=0)
    assert plan.state(plan.duration_s+10.) == (axis(.001), ZERO, ZERO)
    with pytest.raises(ValueError, match='plan_complete'):
        plan.window(plan.duration_s, .08)


def test_moving_target_replan_preserves_position_velocity_and_acceleration():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.01))
    state = plan.state(.15)
    replacement, cursor = retarget_settling_plan(plan, .15, axis(-.01))
    assert replacement is not plan and cursor == 0.
    np.testing.assert_allclose(replacement.state(0.), state, atol=1e-12, rtol=0)
    assert replacement.state(replacement.duration_s) == (axis(-.01), ZERO, ZERO)
    assert_limits(replacement)


def test_retarget_to_current_position_while_moving_reports_target_crossing():
    plan = build_settling_plan(ZERO, axis(.04), ZERO, ZERO)
    assert plan.leaves_endpoint_interval[0] is True
    assert plan.state(plan.duration_s) == (ZERO, ZERO, ZERO)
    assert_limits(plan)


def test_all_six_joints_finish_at_one_synchronized_duration():
    target = (.01, -.005, .001, -.002, .003, -.004)
    plan = build_settling_plan(ZERO, ZERO, ZERO, target)
    assert plan.state(plan.duration_s) == (target, ZERO, ZERO)
    assert_limits(plan)


def test_immutable_plan_does_not_alias_input_arrays():
    initial = np.zeros(6)
    target = np.array(axis(.001))
    plan = build_settling_plan(initial, initial, initial, target)
    initial[:] = 1.; target[:] = 2.
    assert plan.state(0.) == (ZERO, ZERO, ZERO)
    assert plan.target == axis(.001)
    with pytest.raises(FrozenInstanceError):
        plan.target = ZERO


def test_stationary_same_position_plan_never_creates_motion():
    plan = build_settling_plan(ZERO, ZERO, ZERO, ZERO)
    assert plan.state(.04) == (ZERO, ZERO, ZERO)
    assert plan.state(1.) == (ZERO, ZERO, ZERO)


@pytest.mark.parametrize('q,v,a,target,reason', [
    (ZERO, axis(.051), ZERO, ZERO, 'initial_speed_limit'),
    (ZERO, ZERO, axis(.201), ZERO, 'initial_acceleration_limit'),
    (axis(3.01), ZERO, ZERO, ZERO, 'initial_joint_limit'),
    (ZERO, ZERO, ZERO, axis(3.01), 'target_joint_limit'),
    (ZERO, ZERO, ZERO, (0.,)*5, 'invalid_vector'),
    (ZERO, ZERO, axis(float('nan')), ZERO, 'invalid_vector'),
])
def test_invalid_states_are_rejected(q, v, a, target, reason):
    with pytest.raises(ValueError, match=reason):
        build_settling_plan(q, v, a, target)


@pytest.mark.parametrize('maximum', [0., .01, float('nan'), float('inf')])
def test_invalid_duration_search_bounds_are_rejected(maximum):
    with pytest.raises(ValueError, match='invalid_limits'):
        build_settling_plan(ZERO, ZERO, ZERO, ZERO, replace(SettlingLimits(), max_duration_s=maximum))


def test_bounded_search_rejects_a_target_that_cannot_meet_the_duration_budget():
    with pytest.raises(ValueError, match='no_feasible_settling_plan'):
        build_settling_plan(ZERO, ZERO, ZERO, axis(.5), replace(SettlingLimits(), max_duration_s=1.))


def test_initial_outward_motion_cannot_be_hidden_by_endpoint_joint_checks():
    limits = replace(SettlingLimits(), motion=replace(RampLimits(), joint_max=axis(.000001)))
    with pytest.raises(ValueError, match='no_feasible_settling_plan'):
        build_settling_plan(ZERO, axis(.001), axis(-.2), axis(-.1), limits)


@pytest.mark.parametrize('time', [-.001, float('nan'), float('inf')])
def test_invalid_elapsed_time_is_rejected(time):
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    with pytest.raises(ValueError, match='invalid_sample_time'):
        plan.state(time)


@pytest.mark.parametrize('horizon', [0., -.01, float('nan')])
def test_invalid_window_horizon_is_rejected(horizon):
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    with pytest.raises(ValueError, match='invalid_duration'):
        plan.window(0., horizon)


def test_changed_limits_cannot_reuse_a_plan_checked_with_old_limits():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    limits = replace(SettlingLimits(), motion=replace(RampLimits(), speed_cap=.01))
    replacement, cursor = retarget_settling_plan(plan, 0., axis(.001), limits)
    assert replacement is not plan and cursor == 0.
    assert_limits(replacement, limits)


def test_lowering_limits_below_current_reference_speed_explicitly_rejects():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    limits = replace(SettlingLimits(), motion=replace(RampLimits(), speed_cap=.001))
    with pytest.raises(ValueError, match='initial_speed_limit'):
        retarget_settling_plan(plan, .09, axis(.001), limits)


def test_underlying_polynomial_reaches_terminal_pva_before_hold_policy_applies():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    np.testing.assert_allclose(plan.segment.state(plan.duration_s), (axis(.001), ZERO, ZERO), atol=1e-11, rtol=0)


def test_far_target_duration_exposes_the_conservative_response_tradeoff():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.5))
    assert plan.duration_s == pytest.approx(18.75)
    assert plan.state(2.)[0][0] < .01


def test_large_initial_acceleration_is_a_characterized_duration_search_rejection():
    # A family/search limitation, not a proof that all physical trajectories fail.
    with pytest.raises(ValueError, match='no_feasible_settling_plan'):
        build_settling_plan(ZERO, axis(.01), axis(.1), axis(.1))


def test_full_plan_prediction_cannot_extend_the_last_issued_window_coverage():
    plan = build_settling_plan(ZERO, ZERO, ZERO, axis(.001))
    issued_window = plan.window(0., .08)
    assert plan.state(.110)[0][0] > 0
    with pytest.raises(ValueError, match='invalid_sample_time'):
        issued_window.state(.110)
