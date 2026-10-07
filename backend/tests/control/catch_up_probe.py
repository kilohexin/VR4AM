"""Test-only explicit parameter fixture for the runtime supervisor."""

from app.control.catch_up import CatchUpSupervisor
from app.control.transactional_settings import PursuitSettings

EXPERIMENT_PURSUIT = PursuitSettings(budget_ms=500, max_observation_age_ms=40,
    max_observation_gap_ms=40, position_tolerance_rad=.001,
    velocity_tolerance_radps=.002, settle_samples=3)


class CatchUpProbe(CatchUpSupervisor):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, settings=EXPERIMENT_PURSUIT, **kwargs)
