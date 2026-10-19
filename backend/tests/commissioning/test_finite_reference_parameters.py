"""Necessary parameter checks, never controller interpolation or stop evidence."""
import importlib
import math

import pytest


def assess(distance, duration=.08, speed=.005, acceleration=.02):
    try:
        module = importlib.import_module('app.commissioning.finite_reference_parameters')
    except ModuleNotFoundError:
        pytest.fail('finite reference parameter check is missing')
    return module.assess_rest_to_rest_parameters(distance, duration, speed, acceleration)


@pytest.mark.parametrize('distance,want', [(.00002, True), (.000032, True), (.000033, False), (.00005, False)])
def test_acceleration_limits_rest_to_rest_displacement(distance, want):
    result = assess(distance)
    assert result['necessary_conditions_satisfied'] is want
    assert result['max_rest_to_rest_distance_m'] == pytest.approx(.000032)
    assert result['assessment'] == 'necessary_parameters_only_not_trajectory_or_hardware_verification'


@pytest.mark.parametrize('distance,want', [(.0000168, True), (.000017, False)])
def test_combined_tcp_speed_and_acceleration_envelope_is_stricter_than_separate_bounds(distance, want):
    # V=.25 mm/s, A=20 mm/s2, T=80 ms: V*(T-V/A) = 16.875 um.
    result = assess(distance, speed=.00025)
    assert result['necessary_conditions_satisfied'] is want
    assert result['max_rest_to_rest_distance_m'] == pytest.approx(.000016875)


def test_passing_necessary_bound_does_not_claim_quintic_or_hardware_admission():
    # 30 um passes the shape-independent bound, but a straight quintic would
    # need about27 mm/s2. This check must not be called a trajectory certificate.
    result = assess(.00003)
    assert result['necessary_conditions_satisfied']
    assert result['trajectory_verified'] is False
    assert result['hardware_verified'] is False
    assert result['motion_authorized'] is False


@pytest.mark.parametrize('index', range(4))
@pytest.mark.parametrize('bad', [True, False, None, '0.02', 0., -1., math.nan, math.inf, -math.inf, 10**1000],
    ids=['true', 'false', 'none', 'string', 'zero', 'negative', 'nan', 'inf', 'negative_inf', 'huge_int'])
def test_invalid_or_nonfinite_inputs_are_rejected(index, bad):
    values = [.00005, .08, .005, .02]
    values[index] = bad
    with pytest.raises(ValueError, match='finite_reference_invalid_parameters'):
        assess(*values)


@pytest.mark.parametrize('duration', [.009, .081, .6])
def test_parameter_check_does_not_extend_existing_duration_limit(duration):
    with pytest.raises(ValueError, match='finite_reference_horizon_invalid'):
        assess(.00005, duration=duration)
