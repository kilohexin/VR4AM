"""Explicit experiment parameters. No default live-robot pursuit profile."""

from dataclasses import dataclass, fields
import math


@dataclass(frozen=True)
class PursuitSettings:
    budget_ms: int
    max_observation_age_ms: int
    max_observation_gap_ms: int
    position_tolerance_rad: float
    velocity_tolerance_radps: float
    settle_samples: int

    def __post_init__(self):
        for key in ('budget_ms', 'max_observation_age_ms', 'max_observation_gap_ms', 'settle_samples'):
            value = getattr(self, key)
            if type(value) is not int or value <= 0:
                raise ValueError(f'invalid_pursuit:{key}')
        for key in ('position_tolerance_rad', 'velocity_tolerance_radps'):
            value = getattr(self, key)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'invalid_pursuit:{key}')
        if (self.settle_samples < 2 or self.max_observation_age_ms > self.budget_ms
                or self.max_observation_gap_ms > self.budget_ms):
            raise ValueError('invalid_pursuit:bounds')


@dataclass(frozen=True)
class SegmentedSettings:
    settling: PursuitSettings
    tool_spin: bool = False

    def __post_init__(self):
        if type(self.tool_spin) is not bool:raise ValueError('invalid_tool_spin_selection')
        if (not isinstance(self.settling, PursuitSettings)
                or self.settling.settle_samples < 3
                or self.settling.position_tolerance_rad > .001
                or self.settling.velocity_tolerance_radps > .002
                or self.settling.budget_ms > 1000
                or self.settling.max_observation_age_ms > 100
                or self.settling.max_observation_gap_ms > 100):
            raise ValueError('invalid_segment_settling')


def parse_target_strategy(payload, *, backend, mode):
    if 'target_strategy' not in payload:
        return None
    selection = payload['target_strategy']
    if isinstance(selection, dict) and selection == {'kind': 'legacy'}:
        return None
    if isinstance(selection, dict) and selection.get('kind') == 'segmented_experimental':
        if set(selection) not in ({'kind','settling'},{'kind','settling','tool_spin'}):
            raise RuntimeError('invalid_config:target_strategy')
        if backend != 'lebai' or mode != 'control':
            raise RuntimeError('segmented_requires_lebai_control')
        params = selection['settling']
        if not isinstance(params, dict) or set(params) != {f.name for f in fields(PursuitSettings)}:
            raise RuntimeError('invalid_config:target_strategy:settling')
        try:
            return SegmentedSettings(PursuitSettings(**params),selection.get('tool_spin',False))
        except ValueError as error:
            raise RuntimeError('invalid_config:target_strategy:settling') from error
    if (not isinstance(selection, dict) or set(selection) != {'kind', 'pursuit'}
            or selection['kind'] != 'transactional_experimental'):
        raise RuntimeError('invalid_config:target_strategy')
    if backend != 'lebai' or mode != 'control':
        raise RuntimeError('transactional_requires_lebai_control')
    params = selection['pursuit']
    if not isinstance(params, dict) or set(params) != {f.name for f in fields(PursuitSettings)}:
        raise RuntimeError('invalid_config:target_strategy:pursuit')
    try:
        return PursuitSettings(**params)
    except ValueError as error:
        raise RuntimeError('invalid_config:target_strategy:pursuit') from error
