import pytest

from app.control.transactional_settings import PursuitSettings, SegmentedSettings, parse_target_strategy


def payload(**extra):
    return {'target_strategy': {'kind': 'segmented_experimental', 'settling': {
        'budget_ms': 500, 'max_observation_age_ms': 60, 'max_observation_gap_ms': 60,
        'position_tolerance_rad': .001, 'velocity_tolerance_radps': .002, 'settle_samples': 3,
    }, **extra}}


@pytest.mark.parametrize('extra,expected', [({}, False), ({'prepare_on_arm': True}, True),
    ({'prepare_on_arm': False, 'tool_spin': True}, False)])
def test_prepare_is_an_explicit_strict_segmented_selection(extra, expected):
    selection = parse_target_strategy(payload(**extra), backend='lebai', mode='control')
    assert selection.prepare_on_arm is expected


@pytest.mark.parametrize('value', [1, 0, 'true', None])
def test_prepare_selection_rejects_coercion(value):
    with pytest.raises(RuntimeError, match='invalid_config'):
        parse_target_strategy(payload(prepare_on_arm=value), backend='lebai', mode='control')
    with pytest.raises(ValueError, match='invalid_prepare_on_arm'):
        SegmentedSettings(PursuitSettings(500,60,60,.001,.002,3), prepare_on_arm=value)
