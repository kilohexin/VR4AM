"""The separately admitted runner always receives a synthetic client here."""
import importlib

import pytest

from app.commissioning.finite_pause_sampling import run_sampling_pause_window
from tests.commissioning.test_finite_parallel_stop import ConcurrentClient, FIELDS
from tests.commissioning.test_finite_pause_start_period import settings_for
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.test_lebai_adapter_control import FakeClock


def runner():
    try:
        return importlib.import_module('app.commissioning.finite_pause_parallel').run_parallel_pause_window
    except ModuleNotFoundError:
        pytest.fail('separate parallel stop sampling runner missing')


async def exercise(*, mode='finished', fault=None):
    clock = FakeClock()
    client = ConcurrentClient(clock, mode=mode, fault=fault)
    recorder = Recorder()
    async def factory(ip):
        assert ip == '192.168.10.20'  # Label only, never used by a transport.
        return client
    try:
        result = await runner()(settings_for(client, ip='192.168.10.20'), recorder,
            factory, lambda: None, lambda: None, clock=clock.now_ns, deadline_clock=clock.now_ns,
            sleep=clock.sleep, write_check=lambda: None)
        return result, client, recorder
    finally:
        client.release.set()


async def test_site_candidate_has_separate_policy_and_repeated_parallel_reads():
    result, client, recorder = await exercise()
    assert result['complete'] and result['stop_confirmed'] and client.peak == 6
    assert result['site_parallel_stop_reads'] and result['site_pause_start_period']
    assert not result['offline_parallel_stop_reads'] and not result['offline_pause_comparison']
    assert result['stop_snapshot_read_policy'] == 'six_fields_parallel'
    assert result['pvat_entries'] == result['pause_entries'] == 1 and result['stop_entries'] == 0
    assert result['math_entries'] == dict(kinematics_inverse=1, kinematics_forward=4)
    assert [item[0] for item in client.write_calls] == ['move_pvat','pause_move']
    d = next(e for e in recorder.events if e['kind']=='stop_diagnostics')
    assert d['verification_deadline_ns']-d['verification_started_ns'] == 500_000_000
    assert d['samples'][-1]['observed_ns']-d['stable_since_ns'] >= 300_000_000
    assert len(d['samples']) >= 16
    assert not result['hardware_verified'] and not result['motion_authorized']


@pytest.mark.parametrize('case', ['running','late_finish','drift','late_effect','error','pending'])
async def test_site_candidate_keeps_failures_and_only_reserved_pause(case):
    result, client, _ = await exercise(mode=case if case not in ('error','pending') else 'finished',
                                     fault=case if case in ('error','pending') else None)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == ('stop_unverified' if case in ('error','pending') else 'stop_incomplete')
    assert [item[0] for item in client.write_calls] == ['move_pvat','pause_move']


async def test_prior_sampling_scope_cannot_enable_new_private_policy():
    clock=FakeClock()
    client=ConcurrentClient(clock)
    async def forbidden(ip):
        pytest.fail('old scope gained parallel stop reads')
    with pytest.raises(ValueError, match='parallel.*scope_required'):
        await run_sampling_pause_window(settings_for(client,ip='192.168.10.20'),Recorder(),forbidden,
            lambda:None,lambda:None,clock=clock.now_ns,deadline_clock=clock.now_ns,
            write_check=lambda:None,_site_parallel_stop_reads=True)


@pytest.mark.parametrize('missing', ['clock','write_check','client_factory','authorization_check','hot_check'])
async def test_runner_rejects_missing_explicit_dependency_before_factory(missing):
    clock=FakeClock()
    client=ConcurrentClient(clock)
    def forbidden(*args):
        pytest.fail('invalid dependency reached factory')
    kwargs=dict(clock=clock.now_ns,deadline_clock=clock.now_ns,write_check=lambda:None,
        client_factory=forbidden,authorization_check=lambda:None,hot_check=lambda:None)
    kwargs[missing]=None
    with pytest.raises(TypeError,match='explicit_checks_required'):
        await runner()(settings_for(client,ip='192.168.10.20'),Recorder(),**kwargs)


async def test_runner_requires_identical_clock_before_factory():
    clock=FakeClock()
    client=ConcurrentClient(clock)
    def forbidden(*args):
        pytest.fail('clock mismatch reached factory')
    with pytest.raises(ValueError,match='requires_one_clock'):
        await runner()(settings_for(client,ip='192.168.10.20'),Recorder(),forbidden,lambda:None,
            lambda:None,clock=clock.now_ns,deadline_clock=lambda:0,write_check=lambda:None)


@pytest.mark.parametrize('value',[None,0,1,'true',[],{}])
async def test_private_selector_is_boolean_before_any_factory(value):
    from app.commissioning.finite_window_run import _run_window
    client=ConcurrentClient(FakeClock())
    async def forbidden(ip):
        pytest.fail('invalid private selector entered factory')
    with pytest.raises(TypeError,match='site_parallel_stop_reads_requires_boolean'):
        await _run_window(settings_for(client,ip='192.168.10.20'),Recorder(),forbidden,
            lambda:None,lambda:None,_site_parallel_stop_reads=value)


@pytest.mark.parametrize('old_runner',['pause','sampling'])
async def test_prior_runner_rejects_parallel_keyword_even_when_false(old_runner):
    from app.commissioning.finite_pause_window import run_pause_window
    chosen=run_pause_window if old_runner=='pause' else run_sampling_pause_window
    clock=FakeClock()
    client=ConcurrentClient(clock)
    async def forbidden(ip):
        pytest.fail('legacy runner accepted new selector')
    with pytest.raises(ValueError,match='parallel_scope_required'):
        await chosen(settings_for(client,ip='192.168.10.20'),Recorder(),forbidden,lambda:None,
            lambda:None,clock=clock.now_ns,deadline_clock=clock.now_ns,write_check=lambda:None,
            _site_parallel_stop_reads=False)
