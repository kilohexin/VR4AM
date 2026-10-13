"""Endpoint arithmetic only. No SDK, physical plant or controller interpolation."""
from dataclasses import replace

import numpy as np
import pytest

from app.robots.lebai_pvat import PvatLimits
from tests.robots.pvat_direction_candidate import build_directional_point

ZERO = (0.,) * 6
LIMITS = PvatLimits(.08, .05, .2, .25, (-3.,) * 6, (3.,) * 6)


def point(solution, reference=ZERO, actual=ZERO, limits=LIMITS, *, previous=None):
    return build_directional_point(solution, actual, reference, previous, limits)


def test_nonparallel_reference_chooses_maximal_feasible_shared_progress():
    # Desired=(.05,.025); accel budget=.016; second joint restricts fraction<=.64.
    # Old interpolation returns (.0436,.016), which rotates the joint step.
    p = point((.004, .002, 0, 0, 0, 0), (.04, 0, 0, 0, 0, 0))
    assert p.qd == pytest.approx((.032, .016, 0, 0, 0, 0))
    assert p.q == pytest.approx((.00256, .00128, 0, 0, 0, 0))
    assert p.qdd == pytest.approx((-.1, .2, 0, 0, 0, 0))


def test_conflicting_joint_acceleration_intervals_reject_instead_of_rotating():
    # First joint needs fraction>=.68; second requires fraction<=.64.
    with pytest.raises(ValueError, match='^direction_acceleration_infeasible$'):
        point((.004, .002, 0, 0, 0, 0), (.05, 0, 0, 0, 0, 0))


def test_transverse_speed_cannot_be_silently_removed_to_preserve_direction():
    with pytest.raises(ValueError, match='^direction_acceleration_infeasible$'):
        point((.004, 0, 0, 0, 0, 0), (0, .02, 0, 0, 0, 0))


def test_opposite_speed_infeasible_does_not_send_a_reverse_step():
    with pytest.raises(ValueError, match='^direction_acceleration_infeasible$'):
        point((.004, 0, 0, 0, 0, 0), (-.02, 0, 0, 0, 0, 0))


def test_arrived_target_with_large_reference_does_not_claim_instant_standstill():
    with pytest.raises(ValueError, match='^direction_acceleration_infeasible$'):
        point(ZERO, (.03, 0, 0, 0, 0, 0))


def test_previous_reference_is_used_instead_of_silently_zeroing_it():
    p = point((.004, .002, 0, 0, 0, 0), previous=(.04, 0, 0, 0, 0, 0))
    assert p.qd == pytest.approx((.032, .016, 0, 0, 0, 0))
    assert p.reference_source == 'previous_command_velocity'
    assert p.reference_qd == (.04, 0, 0, 0, 0, 0)


@pytest.mark.parametrize('sign', [-1, 1])
def test_zero_reference_preserves_all_joint_ratios_and_acceleration_cap(sign):
    d = np.array((.08, -.04, .016, 0, -.008, .002)) * sign
    p = point(tuple(d))
    assert p.qd == pytest.approx(tuple(d * .2))
    assert p.q == pytest.approx(tuple(d * .016))
    assert max(abs(x) for x in p.qdd) == pytest.approx(.2)


def test_maximal_progress_stops_at_target_when_it_fits_horizon():
    target = (.0008, -.0004, 0, 0, 0, 0)
    p = point(target)
    assert p.q == pytest.approx(target)
    assert p.qd == pytest.approx((.01, -.005, 0, 0, 0, 0))


def test_speed_cap_limits_shared_fraction():
    p = point((.008, -.004, 0, 0, 0, 0), limits=replace(LIMITS, max_joint_acceleration_radps2=10.))
    assert p.qd == pytest.approx((.05, -.025, 0, 0, 0, 0))
    assert p.q == pytest.approx((.004, -.002, 0, 0, 0, 0))


def test_zero_fraction_at_acceleration_boundary_is_valid_but_not_arrival_proof():
    p = point((.004, 0, 0, 0, 0, 0), (-.016, 0, 0, 0, 0, 0))
    assert p.q == pytest.approx(ZERO)
    assert p.qd == pytest.approx(ZERO)
    assert p.qdd[0] == pytest.approx(.2)


def test_zero_target_zero_reference_is_stationary_arithmetic():
    p = point(ZERO)
    assert p.q == p.qd == p.qdd == ZERO


@pytest.mark.parametrize('field,value', [
    ('horizon_s', 0.), ('max_joint_speed_radps', 0.),
    ('max_joint_acceleration_radps2', -.1), ('max_joint_speed_radps', float('nan')),
    ('max_joint_acceleration_radps2', float('inf')),
    ('max_joint_tracking_error_rad', 0.),
])
def test_invalid_limits_fail_without_a_point(field, value):
    with pytest.raises(ValueError, match='^invalid_direction_limits$'):
        point(ZERO, limits=replace(LIMITS, **{field: value}))


@pytest.mark.parametrize('target,actual,reference,previous,reason', [
    ((0.,)*5, ZERO, ZERO, None, 'invalid_direction_vector'),
    (ZERO, ZERO, (float('nan'),0,0,0,0,0), None, 'invalid_direction_vector'),
    (ZERO, ZERO, ZERO, (float('inf'),0,0,0,0,0), 'invalid_direction_vector'),
    ((3.1,0,0,0,0,0), ZERO, ZERO, None, 'direction_target_joint_limit'),
    (ZERO, (3.1,0,0,0,0,0), ZERO, None, 'direction_actual_joint_limit'),
    ((.251,0,0,0,0,0), ZERO, ZERO, None, 'direction_tracking_limit'),
    (ZERO, ZERO, (.051,0,0,0,0,0), None, 'direction_speed_limit'),
    (ZERO, ZERO, (.051,0,0,0,0,0), ZERO, 'direction_speed_limit'),
])
def test_invalid_or_unadmitted_inputs_reject(target, actual, reference, previous, reason):
    with pytest.raises(ValueError, match=f'^{reason}$'):
        point(target, reference, actual, previous=previous)


@pytest.mark.parametrize('changes', [
    {'horizon_s': 1e-320},
    {'horizon_s': 1e200, 'max_joint_acceleration_radps2': 1e200},
])
def test_nonfinite_derived_arithmetic_rejects_instead_of_returning_nan(changes):
    with pytest.raises(ValueError, match='^direction_numeric_range$'):
        point((.004, .002, 0, 0, 0, 0), limits=replace(LIMITS, **changes))


@pytest.mark.parametrize('reference', [
    (.01, -.005, 0, 0, 0, 0), (.04, 0, 0, 0, 0, 0),
    (.05, 0, 0, 0, 0, 0), (-.02, 0, 0, 0, 0, 0),
])
def test_scalar_inequality_grid_agrees_with_rejection_and_maximal_progress(reference):
    target = (.004, .002, 0, 0, 0, 0)
    # Separate finite grid checks the defining inequalities, not implementation helpers.
    fractions = np.linspace(0., 1., 10001)
    velocities = fractions[:, None] * np.array((.05, .025, 0, 0, 0, 0))
    mask = np.all(np.abs(velocities - np.array(reference)) <= .016 + 1e-14, axis=1)
    feasible = fractions[mask]
    if len(feasible) == 0:
        with pytest.raises(ValueError, match='^direction_acceleration_infeasible$'):
            point(target, reference)
    else:
        p = point(target, reference)
        assert feasible[-1] <= p.q[0] / .004 + 1e-14
        assert p.q[0] / .004 <= feasible[-1] + .0001 + 1e-14
        assert max(abs(v-r) for v, r in zip(p.qd, reference)) <= .016 + 1e-14


def test_negative_joint_directions_use_ordered_intersections():
    p = point((-.004, -.002, 0, 0, 0, 0), (-.04, 0, 0, 0, 0, 0))
    assert p.qd == pytest.approx((-.032, -.016, 0, 0, 0, 0))


def test_shared_fraction_preserves_affine_joint_relations():
    actual = (.2, -.3, .1, 0, 0, 0)
    target = (.204, -.298, .094, 0, 0, 0)
    p = point(target, (.02, 0, -.02, 0, 0, 0), actual)
    assert sum(p.q[:3]) == pytest.approx(sum(actual[:3]), abs=1e-14)


def test_input_arrays_are_not_modified_and_reference_is_copied():
    actual = np.zeros(6)
    target = np.array((.004, .002, 0, 0, 0, 0))
    reference = np.array((.04, 0, 0, 0, 0, 0))
    before = (actual.copy(), target.copy(), reference.copy())
    p = point(target, reference, actual)
    for current, original in zip((actual, target, reference), before):
        np.testing.assert_array_equal(current, original)
    reference[0] = 0
    assert p.reference_qd[0] == .04


@pytest.mark.parametrize('changes', [
    {'soft_joint_min_rad':(3.,)*6}, {'soft_joint_max_rad':(-3.,)*6},
])
def test_unordered_soft_bounds_reject(changes):
    with pytest.raises(ValueError, match='^invalid_direction_limits$'):
        point(ZERO, limits=replace(LIMITS, **changes))


def test_endpoint_acceleration_budget_does_not_prove_position_reachable():
    from tests.robots.pvat_interpolation_probe import reachable_endpoints

    p = point((.004, 0, 0, 0, 0, 0))
    assert p.q[0] == pytest.approx(.00128)
    # Any trajectory starting at v=0 with |a|<=.2 has displacement<=.00064
    # over .08s, regardless of controller interpolation. Endpoint v/a pass
    # alone must never be described as a full trajectory guarantee.
    bounds = reachable_endpoints(ZERO, ZERO, p.q, p.qd, .08, .05, .2)
    assert bounds.max_q[0] == pytest.approx(.00064)
    assert bounds.velocity_compatible[0] is True
    assert bounds.position_compatible[0] is False
