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


def parse_target_strategy(payload, *, backend, mode):
    if 'target_strategy' not in payload:
        return None
    selection = payload['target_strategy']
    if isinstance(selection, dict) and selection == {'kind': 'legacy'}:
        return None
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
