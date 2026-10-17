"""Fake-only wiring of the existing bounded observer window."""
from dataclasses import replace
from unittest.mock import AsyncMock
import pytest
from app.digital_twin.segmented_demo import demo_settings
from app.main import build_backend
from app.recording.noop import NoopRecorder

def test_fake_flag_enables_exactly_twenty_ms_but_ordinary_builds_stay_disabled():
    settings=demo_settings();factory=AsyncMock()
    candidate=build_backend(settings,NoopRecorder(),factory,'LEBAI_FAKE',offline_observer_priority=True)
    assert candidate._observer_window.max_wait_s==.020
    assert candidate.settings is settings.lebai
    assert candidate.settings.control.pvat_horizon_s==.08
    for runtime in ('LEBAI','LEBAI_FAKE'):
        ordinary=build_backend(settings,NoopRecorder(),factory,runtime)
        assert ordinary._observer_window.max_wait_s==0.
    factory.assert_not_called()

@pytest.mark.parametrize('value',[None,1,'true'])
def test_non_boolean_option_is_rejected_before_factory(value):
    factory=AsyncMock()
    with pytest.raises(ValueError,match='invalid_offline_observer_priority'):
        build_backend(demo_settings(),NoopRecorder(),factory,'LEBAI_FAKE',offline_observer_priority=value)
    factory.assert_not_called()

@pytest.mark.parametrize('case',['real','missing_identity','simulator_identity','simulator_backend','missing_segment','nonlocal','missing_lebai'])
def test_option_cannot_select_real_or_nonlocal_runtime(case):
    settings=demo_settings();runtime='LEBAI_FAKE';factory=AsyncMock()
    if case=='real':runtime='LEBAI'
    if case=='missing_identity':runtime=None
    if case=='simulator_identity':runtime='SIMULATOR'
    if case=='simulator_backend':settings=replace(settings,backend='simulator')
    if case=='missing_segment':settings=replace(settings,segmented_target=None)
    if case=='nonlocal':settings=replace(settings,lebai=replace(settings.lebai,ip='192.0.2.1'))
    if case=='missing_lebai':settings=replace(settings,lebai=None)
    with pytest.raises(ValueError,match='offline_observer_priority_offline_only'):
        build_backend(settings,NoopRecorder(),factory,runtime,offline_observer_priority=True)
    factory.assert_not_called()
