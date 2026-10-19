"""In-process Fake only; no controller or transport is used."""
import asyncio
import importlib

import pytest

from app.commissioning.finite_pause_window import run_pause_window
from tests.commissioning.test_finite_pause_start_period import TimedPauseClient, settings_for
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.test_lebai_adapter_control import FakeClock


def runner():
    try:
        return importlib.import_module('app.commissioning.finite_pause_sampling').run_sampling_pause_window
    except ModuleNotFoundError:
        pytest.fail('separate start-period pause runner missing')


async def exercise(*, finished_ms=160., drift=False, late_rpc=False):
    clock = FakeClock()
    client = TimedPauseClient(clock, finished_ms=finished_ms, drift=drift)
    if late_rpc:
        original = client.pause_move
        async def pause():
            await original()
            clock.advance_ms(200.)
        client.pause_move = pause
    recorder = Recorder()
    async def factory(ip):
        assert ip == '192.168.10.20'  # Synthetic label, never a transport target.
        return client
    result = await runner()(settings_for(client, ip='192.168.10.20'), recorder, factory,
        lambda: None, lambda: None, clock=clock.now_ns, deadline_clock=clock.now_ns,
        sleep=clock.sleep, write_check=lambda: None)
    assert client.network_calls == client.discovery_calls == 0
    return result, client, recorder


async def test_separate_runner_uses_start_period_and_original_limits():
    result, client, recorder = await exercise()
    assert result['complete'] and result['stop_confirmed']
    assert result['site_pause_start_period'] and not result['offline_pause_comparison']
    assert result['stop_sampling_policy'] == 'start_period_20ms'
    samples = next(e for e in recorder.events if e['kind'] == 'stop_diagnostics')['samples']
    assert samples[-1]['observed_ns'] - client.pause_ns == 494_000_000
    assert samples[-1]['observed_ns'] - samples[4]['observed_ns'] == 304_000_000
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    assert result['pvat_entries'] == result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert not result['hardware_verified'] and not result['motion_authorized']


@pytest.mark.parametrize('case', ['running', 'late_finished', 'drift', 'late_rpc'])
async def test_sampling_runner_keeps_failures_and_single_pause(case):
    result, client, _ = await exercise(finished_ms=10000. if case == 'running'
        else 260. if case == 'late_finished' else 0., drift=case == 'drift', late_rpc=case == 'late_rpc')
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == ('stop_unverified' if case == 'late_rpc' else 'stop_incomplete')
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']


async def test_legacy_site_runner_cannot_select_new_sampling_scope():
    clock = FakeClock()
    client = TimedPauseClient(clock)
    async def forbidden(_ip):
        pytest.fail('legacy scope gained new sampling before authorization')
    with pytest.raises(ValueError, match='sampling_scope_required'):
        await run_pause_window(settings_for(client, ip='192.168.10.20'), Recorder(), forbidden,
            lambda: None, lambda: None, clock=clock.now_ns, deadline_clock=clock.now_ns,
            write_check=lambda: None, _site_pause_start_period=True)


@pytest.mark.parametrize('missing', ['clock', 'write_check', 'client_factory', 'authorization_check', 'hot_check'])
async def test_sampling_runner_requires_explicit_dependencies_before_factory(missing):
    clock = FakeClock()
    client = TimedPauseClient(clock)
    def forbidden(*_args):
        pytest.fail('invalid dependencies entered factory')
    options = dict(clock=clock.now_ns, deadline_clock=clock.now_ns, write_check=lambda: None,
                   client_factory=forbidden, authorization_check=lambda: None, hot_check=lambda: None)
    options[missing] = None
    with pytest.raises(TypeError, match='explicit_checks_required'):
        await runner()(settings_for(client, ip='192.168.10.20'), Recorder(), **options)


async def test_sampling_runner_unknown_read_keeps_latch():
    clock, release = FakeClock(), asyncio.Event()
    client = TimedPauseClient(clock)
    original = client.get_kin_data
    async def pending():
        if client.pause_ns is not None:
            await release.wait()
        return await original()
    client.get_kin_data = pending
    async def factory(_ip):
        return client
    try:
        result = await runner()(settings_for(client, ip='192.168.10.20'), Recorder(), factory,
            lambda: None, lambda: None, clock=clock.now_ns, deadline_clock=clock.now_ns,
            sleep=clock.sleep, write_check=lambda: None)
        assert not result['complete'] and result['read_remote_result_unknown']
        assert result['latched_fault'] == 'stop_unverified'
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'pause_move']
    finally:
        release.set()
        for _ in range(5):
            await asyncio.sleep(0)
