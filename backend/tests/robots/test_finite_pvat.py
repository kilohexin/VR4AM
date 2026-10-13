"""Offline curve checks; no SDK calls or physical stopping claims."""
from dataclasses import replace

import numpy as np
import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_finite_pvat import build_finite_pvat_candidate
from app.robots.lebai_pvat import PvatLimits
from tests.robots.pvat_interpolation_probe import quintic_segment
from tests.robots.pvat_settling_candidate import _position_range

ZERO = (0.,) * 6


def axis(value):
    return (value, *ZERO[1:])


def limits(**changes):
    return replace(PvatLimits(.08, .05, .2, .12, (-3.,) * 6, (3.,) * 6), **changes)


def build(target=axis(.01), actual=ZERO, velocity=ZERO, acceleration=ZERO, **changes):
    return build_finite_pvat_candidate(target, actual, velocity, acceleration, limits(**changes))


def assert_curve(candidate, actual, velocity, acceleration, bounds):
    point = candidate.point
    assert point.qd == point.qdd == ZERO
    assert point.reference_qd == tuple(velocity) and point.reference_source == "observed_velocity"
    curve = quintic_segment(actual, velocity, acceleration, point.q, point.qd, point.qdd, point.horizon_s)
    np.testing.assert_allclose(curve.state(0.), (actual, velocity, acceleration), atol=1e-12, rtol=0)
    np.testing.assert_allclose(curve.state(point.horizon_s), (point.q, ZERO, ZERO), atol=1e-10, rtol=0)
    speed, accel = curve.peak_abs_derivatives()
    assert max(speed) <= bounds.max_joint_speed_radps + 1e-10
    assert max(accel) <= bounds.max_joint_acceleration_radps2 + 1e-10
    assert all(v <= bound + 1e-10 for v, bound in zip(speed, candidate.peak_speed_bound))
    assert all(a <= bound + 1e-10 for a, bound in zip(accel, candidate.peak_acceleration_bound))
    minimum, maximum = _position_range(curve)
    assert np.all(minimum >= np.array(bounds.soft_joint_min_rad) - 1e-10)
    assert np.all(maximum <= np.array(bounds.soft_joint_max_rad) + 1e-10)
    assert np.all(minimum >= np.array(candidate.position_min_bound) - 1e-10)
    assert np.all(maximum <= np.array(candidate.position_max_bound) + 1e-10)


def test_rest_to_rest_window_reduces_progress_and_is_not_just_endpoint_clipping():
    candidate = build()
    assert 0 < candidate.progress_fraction < 1
    theoretical = .2 * .08 ** 2 / ((10 / np.sqrt(3)) * .01)
    assert .98 * theoretical <= candidate.progress_fraction <= theoretical + 1e-12
    assert_curve(candidate, ZERO, ZERO, ZERO, limits())


def test_all_six_endpoint_positions_share_one_fraction_and_inputs_are_immutable():
    actual = np.array([.5, -.5, .2, -.2, .3, -.3])
    delta = np.array([.01, -.02, .015, -.012, .004, -.008])
    target = actual + delta
    old = actual.copy(), target.copy()
    candidate = build_finite_pvat_candidate(target, actual, ZERO, ZERO, limits())
    np.testing.assert_allclose((np.array(candidate.point.q) - actual) / delta,
                               candidate.progress_fraction, atol=1e-12, rtol=0)
    assert_curve(candidate, actual, ZERO, ZERO, limits())
    np.testing.assert_array_equal(actual, old[0])
    np.testing.assert_array_equal(target, old[1])


def test_nonzero_velocity_feasible_interval_need_not_contain_zero_fraction():
    candidate = build(target=axis(.001), velocity=axis(.01))
    assert .1 < candidate.progress_fraction < 1
    assert candidate.feasible_fraction_min > 0
    assert_curve(candidate, ZERO, axis(.01), ZERO, limits())


def test_reversal_with_insufficient_time_rejects_without_faking_zero_velocity():
    with pytest.raises(BackendCommandError, match="finite_pvat_no_feasible_window"):
        build(target=axis(.001), velocity=axis(-.01))


def test_nonzero_acceleration_is_preserved_at_window_start():
    candidate = build(velocity=axis(.002), acceleration=axis(.04))
    assert_curve(candidate, ZERO, axis(.002), axis(.04), limits())


def test_at_target_and_at_rest_emits_a_constant_window():
    candidate = build(target=ZERO)
    assert candidate.point.q == ZERO
    assert candidate.progress_fraction == 1
    assert_curve(candidate, ZERO, ZERO, ZERO, limits())


@pytest.mark.parametrize("time", [.01, .02, .04, .08])
def test_duration_is_not_extended_to_make_window_feasible(time):
    candidate = build(horizon_s=time)
    assert candidate.point.horizon_s == time
    assert_curve(candidate, ZERO, ZERO, ZERO, limits(horizon_s=time))


@pytest.mark.parametrize("changes", [
    {"horizon_s": .009}, {"horizon_s": .081}, {"horizon_s": True}, {"horizon_s": float("nan")},
    {"max_joint_speed_radps": 0}, {"max_joint_acceleration_radps2": -1},
    {"max_joint_tracking_error_rad": float("inf")},
    {"soft_joint_min_rad": (3.,) * 6}, {"soft_joint_max_rad": (3.,) * 5},
])
def test_invalid_limits_are_rejected(changes):
    with pytest.raises(BackendCommandError, match="invalid_finite_pvat_limits"):
        build(**changes)


@pytest.mark.parametrize("changes,reason", [
    ({"target": axis(3.1)}, "finite_pvat_target_joint_limit"),
    ({"actual": axis(3.1), "target": axis(3.)}, "finite_pvat_actual_joint_limit"),
    ({"target": axis(.121)}, "ik_tracking_diverged"),
    ({"velocity": axis(.050001)}, "finite_pvat_initial_speed_limit"),
    ({"acceleration": axis(.200001)}, "finite_pvat_initial_acceleration_limit"),
    ({"acceleration": axis(.2876213977285576)}, "finite_pvat_initial_acceleration_limit"),
    ({"target": (0.,) * 5}, "invalid_finite_pvat_vector"),
    ({"actual": axis(float("nan"))}, "invalid_finite_pvat_vector"),
    ({"velocity": None}, "invalid_finite_pvat_vector"),
    ({"acceleration": (True, *ZERO[1:])}, "invalid_finite_pvat_vector"),
])
def test_invalid_input_is_rejected_without_clipping(changes, reason):
    with pytest.raises(BackendCommandError, match=reason):
        build(**changes)


def test_position_limit_applies_inside_curve_even_when_endpoint_is_inside():
    # Starting exactly at upper limit with outward velocity has no bounded curve.
    with pytest.raises(BackendCommandError, match="finite_pvat_no_feasible_window"):
        build(actual=axis(3.), target=axis(2.99), velocity=axis(.001))


def test_deterministic_random_curves_obey_bounds_at_independent_extrema():
    random = np.random.default_rng(20261013)
    for _ in range(128):
        actual = random.uniform(-1., 1., 6)
        target = actual + random.uniform(-.02, .02, 6)
        velocity = random.uniform(-.001, .001, 6)
        acceleration = random.uniform(-.01, .01, 6)
        candidate = build_finite_pvat_candidate(target, actual, velocity, acceleration, limits())
        assert 0 <= candidate.feasible_fraction_min <= candidate.progress_fraction <= 1
        assert_curve(candidate, actual, velocity, acceleration, limits())


def test_dispatch_reuses_precomputed_hulls(monkeypatch):
    import app.robots.lebai_finite_pvat as finite
    monkeypatch.setattr(finite, "_split", lambda _: pytest.fail("subdivision must be precomputed"))
    candidate = build()
    assert_curve(candidate, ZERO, ZERO, ZERO, limits())


@pytest.mark.parametrize("degree", [3, 4, 5])
def test_precomputed_hull_weights_are_nonnegative_and_sum_to_one(degree):
    from app.robots.lebai_finite_pvat import _HULL_MATRICES
    matrix = _HULL_MATRICES[degree]
    assert matrix.shape == (16 * (degree + 1), degree + 1)
    assert np.all(matrix >= 0)
    np.testing.assert_array_equal(np.sum(matrix, axis=1), np.ones(len(matrix)))
    assert not matrix.flags.writeable


def test_speed_change_alone_can_make_80ms_stop_impossible():
    # |v0-v1|=.02 exceeds A*T=.016; changing endpoint position cannot fix it.
    with pytest.raises(BackendCommandError, match="finite_pvat_no_feasible_window"):
        build(target=axis(.01), velocity=axis(.02))


def test_independent_completed_windows_never_require_an_unsent_tail():
    state = (ZERO, ZERO, ZERO)
    for target in [axis(.01)] * 8 + [axis(-.01)] * 8:
        candidate = build_finite_pvat_candidate(target, *state, limits())
        assert_curve(candidate, *state, limits())
        curve = quintic_segment(*state, candidate.point.q, ZERO, ZERO, candidate.point.horizon_s)
        endpoint = curve.state(candidate.point.horizon_s)
        np.testing.assert_allclose(endpoint[1:], (ZERO, ZERO), atol=1e-12, rtol=0)
        # A synthetic exactly completed reference, not measured robot feedback.
        state = (candidate.point.q, ZERO, ZERO)


def test_extreme_finite_input_reports_range_error_without_numeric_warnings():
    bounds = limits(max_joint_speed_radps=1e308, max_joint_acceleration_radps2=1e308,
                    max_joint_tracking_error_rad=1e308, soft_joint_min_rad=(-1e308,) * 6,
                    soft_joint_max_rad=(1e308,) * 6)
    with np.errstate(all="raise"):
        with pytest.raises(BackendCommandError, match="finite_pvat_numeric_range"):
            build_finite_pvat_candidate(ZERO, ZERO, axis(1e308), axis(1e308), bounds)
