"""No transport: original cleanup lifecycle with separate write admission."""
import asyncio
import json

import pytest

from app.commissioning import paused_stop_cleanup as cleanup
from tests.commissioning.test_paused_stop_cleanup import paused_client
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


async def exercise(*, write_guard=None, separate=True, cold_cost_ms=83.):
    clock, client = FakeClock(), paused_client()
    original_read = client.get_kin_data
    async def read():
        clock.advance_ms(47.)
        return await original_read()
    client.get_kin_data = read
    original_stop = client.stop_move
    async def stop():
        await original_stop()
        client.robot_state = 'IDLE'
    client.stop_move = stop
    def cold():
        clock.advance_ms(cold_cost_ms)
    async def factory(_ip):
        return client
    kwargs = dict(stop_authorization_check=write_guard or (lambda: None)) if separate else {}
    report = await cleanup.run_cleanup(control_settings(), Recorder(), factory, cold,
        read_authorization_check=lambda: None, clock=clock.now_ns, sleep=clock.sleep, **kwargs)
    return report, client


async def test_separate_write_guard_avoids_cold_hash_after_final_snapshot():
    report, client = await exercise()
    assert report['complete'] and report['stop_confirmed']
    assert report['sdk_stop_entries'] == 1 and client.write_calls == [('stop_move',)]
    assert report['stop_entry_rejection'] is report['stop_entry_rejection_type'] is None


async def test_default_guard_stays_cold_and_reports_exact_sample_expiry():
    report, client = await exercise(separate=False)
    assert not report['complete'] and report['latched_fault'] == 'stop_unverified'
    assert report['stop_entry_rejection'] == 'paused_cleanup_sample_not_current'
    assert report['stop_entry_rejection_type'] == 'AdmissionRejected'
    assert report['sdk_stop_entries'] == 0 and client.write_calls == []


@pytest.mark.parametrize('kind', ['revoked', 'false', 'async', 'secret'])
async def test_write_guard_rejection_never_sends_or_clears_latch(kind):
    def denied():
        raise ValueError('paused_cleanup_authorization_expired' if kind == 'revoked' else 'secret_guard_details')
    async def asynchronous():
        return None
    guard = (lambda: False) if kind == 'false' else asynchronous if kind == 'async' else denied
    report, client = await exercise(write_guard=guard)
    assert not report['complete'] and report['latched_fault'] == 'stop_unverified'
    assert report['sdk_stop_entries'] == 0 and client.write_calls == []
    assert report['stop_entry_rejection'] == ('paused_cleanup_authorization_expired' if kind == 'revoked'
        else None if kind == 'secret' else 'paused_cleanup_invalid_authorization_result')
    assert 'secret_guard_details' not in json.dumps(report)


@pytest.mark.parametrize('value', [False, 0, 'yes'])
async def test_noncallable_write_guard_fails_before_factory(value):
    async def forbidden(_ip):
        pytest.fail('invalid write guard connected')
    with pytest.raises(TypeError, match='explicit_stop_authorization_required'):
        await cleanup.run_cleanup(control_settings(), Recorder(), forbidden, lambda: None,
            stop_authorization_check=value)


async def test_separate_write_guard_does_not_relax_sample_age(monkeypatch):
    clock, client = FakeClock(), paused_client()
    original = cleanup.RealLebaiAdapter._ensure_stop_transaction
    def transaction(adapter):
        original(adapter)
        adapter._stop_transaction.on_request_started = lambda _method: clock.advance_ms(100.)
    monkeypatch.setattr(cleanup.RealLebaiAdapter, '_ensure_stop_transaction', transaction)
    async def factory(_ip):
        return client
    report = await cleanup.run_cleanup(control_settings(), Recorder(), factory, lambda: None,
        read_authorization_check=lambda: None, stop_authorization_check=lambda: None,
        clock=clock.now_ns, sleep=clock.sleep)
    assert not report['complete'] and report['stop_entry_rejection'] == 'paused_cleanup_sample_not_current'
    assert report['sdk_stop_entries'] == 0 and client.write_calls == []


async def test_native_failure_is_not_mislabeled_as_write_guard_rejection():
    client, clock = paused_client(), FakeClock()
    async def stop():
        client.write_calls.append(('stop_move',))
        raise RuntimeError('private_sdk_error')
    client.stop_move = stop
    async def factory(_ip):
        return client
    report = await cleanup.run_cleanup(control_settings(), Recorder(), factory, lambda: None,
        stop_authorization_check=lambda: None, clock=clock.now_ns, sleep=clock.sleep)
    assert report['latched_fault'] == 'stop_unverified' and report['sdk_stop_entries'] == 1
    assert report['stop_entry_rejection'] is report['stop_entry_rejection_type'] is None
    assert client.write_calls == [('stop_move',)]
    assert 'private_sdk_error' not in json.dumps(report)
