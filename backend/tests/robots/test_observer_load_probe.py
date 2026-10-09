import pytest

from tests.robots.pvat_observer_load_probe import summarize, run_case


def test_summary_keeps_missing_samples_missing_and_counts_strict_overruns():
    assert summarize([]) == {'count': 0, 'median_ms': None, 'p95_ms': None,
                             'max_ms': None, 'over_80ms': 0}
    assert summarize([79, 80, 81]) == {'count': 3, 'median_ms': 80,
                                     'p95_ms': 81, 'max_ms': 81, 'over_80ms': 1}


@pytest.mark.asyncio
async def test_probe_uses_fake_sdk_and_reports_complete_sampling():
    result = await run_case('baseline', 0, samples=3, warmup=0)
    assert result['offline_only'] is True
    assert result['hardware_verified'] is False
    assert result['sampling_error'] is None
    assert result['motion_samples'] == 3
    assert result['send_gaps']['count'] == 2
    assert result['observation_count'] > 0
    assert result['observation_errors'] == []
    assert result['cleanup_error'] is None
    assert result['comparison_eligible'] is True


@pytest.mark.asyncio
async def test_observer_failure_cannot_be_counted_as_eligible_comparison(monkeypatch):
    from tests.robots import pvat_observer_load_probe as probe
    original = probe._connected_control_adapter

    async def connected(**kwargs):
        adapter, client, clock = await original(**kwargs)

        async def failed_observation():
            raise RuntimeError('injected_observer_failure')

        adapter.get_state = failed_observation
        return adapter, client, clock

    monkeypatch.setattr(probe, '_connected_control_adapter', connected)
    result = await probe.run_case('baseline', 0, samples=3, warmup=0)
    assert result['sampling_complete'] is True
    assert result['observation_errors'] == ['injected_observer_failure']
    assert result['comparison_eligible'] is False


@pytest.mark.asyncio
async def test_one_send_is_complete_but_has_no_cadence_comparison():
    result = await run_case('baseline', 0, samples=1, warmup=0)
    assert result['sampling_complete'] is True
    assert result['send_gaps']['count'] == 0
    assert result['comparison_eligible'] is False


@pytest.mark.asyncio
async def test_failed_dispatch_is_reported_once_and_not_retried(monkeypatch):
    from tests.robots import pvat_observer_load_probe as probe
    original = probe._connected_control_adapter
    attempts = []

    async def connected(**kwargs):
        adapter, client, clock = await original(**kwargs)

        async def failed_dispatch(*args):
            attempts.append(args)
            raise RuntimeError('injected_send_failure')

        client.move_pvat = failed_dispatch
        return adapter, client, clock

    monkeypatch.setattr(probe, '_connected_control_adapter', connected)
    result = await probe.run_case('baseline', .005, samples=3, warmup=0)
    assert len(attempts) == 1
    assert result['sampling_error'] == 'sdk_call_failed:move_pvat'
    assert result['sampling_complete'] is False
    assert result['comparison_eligible'] is False


@pytest.mark.asyncio
@pytest.mark.parametrize('profile,budget,samples', [('unknown', 0, 3),
                                                  ('baseline', .021, 3),
                                                  ('baseline', 0, 0)])
async def test_invalid_probe_inputs_are_rejected_before_client_creation(profile, budget, samples):
    with pytest.raises(ValueError):
        await run_case(profile, budget, samples=samples)
