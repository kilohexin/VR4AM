import asyncio
from unittest.mock import AsyncMock

import pytest

from app.commissioning import paused_stop_cleanup as cleanup
from tests.commissioning.test_idle_stop_baseline import Recorder
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


def paused_client():
    client = FakeLebaiClient.idle()
    client.robot_state = 'PAUSED'
    client.running_motion = 416
    return client


async def run(client, *, recorder=None, authorize=lambda: None, clock=None):
    clock = clock or FakeClock()
    factory = AsyncMock(return_value=client)
    result = await cleanup.run_cleanup(control_settings(), recorder or Recorder(), factory, authorize,
                                       clock=clock.now_ns, sleep=clock.sleep)
    return result


async def test_paused_cleanup_uses_one_original_stop_without_targets_or_resume():
    client = paused_client()
    original = client.stop_move
    async def stop():
        await original()
        client.robot_state = 'IDLE'
    client.stop_move = stop
    result = await run(client)
    assert result['complete'] and result['stop_confirmed'] and result['idle_tail_samples'] > 0
    assert result['tail_all_idle'] and not result['hardware_stop_verified']
    assert client.write_calls == [('stop_move',)] and result['sdk_stop_entries'] == 1
    assert result['local_resources_released'] and not result['native_connection_release_verified']


async def test_confirmed_stop_still_paused_is_not_cleanup_complete():
    client = paused_client()
    result = await run(client)
    assert not result['complete'] and result['stop_confirmed']
    assert not result['tail_all_idle'] and result['idle_tail_samples'] == 0
    assert client.write_calls == [('stop_move',)]


@pytest.mark.parametrize('change', ['idle', 'moving', 'hold', 'stop', 'estop', 'speed', 'running', 'tcp', 'geometry'])
async def test_ineligible_initial_state_has_zero_writes(change):
    client = paused_client()
    if change in ('idle', 'moving', 'hold', 'stop'):
        client.robot_state = change.upper()
    elif change == 'estop':
        client.estop_reason = 4
    elif change == 'speed':
        client.kin_data['actual_joint_speed'][0] = 1e-9
    elif change == 'running':
        client.motion_state = 'RUNNING'
    elif change == 'tcp':
        client.tcp['z'] += .01
    else:
        client.kin_data['actual_tcp_pose']['z'] = 3.
    result = await run(client)
    assert not result['complete'] and result['sdk_stop_entries'] == 0 and client.write_calls == []


@pytest.mark.parametrize('change', ['permission', 'drift', 'recording'])
async def test_change_after_admission_cannot_enter_native_stop(change):
    client, recorder = paused_client(), Recorder()
    original = recorder.write_critical_event
    revoked = False
    async def altered(kind, payload, timestamp):
        nonlocal revoked
        await original(kind, payload, timestamp)
        if kind == 'paused_stop_admission':
            if change == 'permission':
                revoked = True
            elif change == 'drift':
                client.kin_data['actual_joint_pose'][0] += .01
            else:
                recorder.fatal_error = 'synthetic_disk_failure'
    recorder.write_critical_event = altered
    def authorize():
        if revoked:
            raise ValueError('expired')
    result = await run(client, recorder=recorder, authorize=authorize)
    assert not result['complete'] and client.write_calls == []


@pytest.mark.parametrize('kind', ['timeout', 'error', 'moving'])
async def test_failed_stop_preserves_original_latch_and_never_retries(kind):
    client, clock = paused_client(), FakeClock()
    original = client.stop_move
    async def stop():
        await original()
        if kind == 'timeout':
            clock.advance_ms(610)
        elif kind == 'error':
            raise RuntimeError('secret_fake_error')
        else:
            client.robot_state = 'MOVING'
    client.stop_move = stop
    result = await run(client, clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == ('stop_incomplete' if kind == 'moving' else 'stop_unverified')
    assert client.write_calls == [('stop_move',)]
    assert 'secret_fake_error' not in str(result)


@pytest.mark.parametrize('change', ['state', 'drift'])
async def test_tail_change_invalidates_cleanup_without_another_stop(monkeypatch, change):
    client = paused_client()
    original_stop = client.stop_move
    async def stop():
        await original_stop()
        client.robot_state = 'IDLE'
    client.stop_move = stop
    original = cleanup.RealLebaiAdapter.read_stop_observation
    count = 0
    async def observe(adapter):
        nonlocal count
        count += 1
        if count == 3:
            if change == 'state':
                client.robot_state = 'MOVING'
            else:
                client.kin_data['actual_tcp_pose']['x'] += .002
        return await original(adapter)
    monkeypatch.setattr(cleanup.RealLebaiAdapter, 'read_stop_observation', observe)
    result = await run(client)
    assert not result['complete'] and result['tail_rejected']
    assert client.write_calls == [('stop_move',)]


async def test_false_authorization_result_never_connects():
    factory = AsyncMock()
    result = await cleanup.run_cleanup(control_settings(), Recorder(), factory, lambda: False)
    factory.assert_not_called()
    assert not result['complete']


async def test_cancellation_after_native_entry_keeps_unverified_and_no_extra_write():
    client, recorder = paused_client(), Recorder()
    entered, release = asyncio.Event(), asyncio.Event()
    original = client.stop_move
    async def stop():
        await original()
        entered.set()
        await release.wait()
    client.stop_move = stop
    task = asyncio.create_task(run(client, recorder=recorder))
    await entered.wait()
    task.cancel()
    try:
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 2.)
        result = next(e for e in recorder.events if e['kind'] == 'paused_stop_cleanup_result')
        assert result['latched_fault'] == 'stop_unverified' and result['sdk_stop_entries'] == 1
        assert client.write_calls == [('stop_move',)]
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)


async def test_even_ineligible_idle_connect_never_starts_motion_pump(monkeypatch):
    from app.robots.lebai_pump import PvatPump
    start = AsyncMock()
    monkeypatch.setattr(PvatPump, 'start', start)
    result = await run(FakeLebaiClient.idle())
    assert not result['complete']
    start.assert_not_called()


@pytest.mark.parametrize('change', ['age', 'raw_state', 'cached_read_fault'])
async def test_stop_child_entry_rechecks_latest_cache_and_age(monkeypatch, change):
    from dataclasses import replace
    client, clock = paused_client(), FakeClock()
    original = cleanup.RealLebaiAdapter._ensure_stop_transaction
    def transaction(adapter):
        original(adapter)
        def alter(_method):
            if change == 'age':
                clock.advance_ms(100)
            elif change == 'raw_state':
                adapter._snapshot = replace(adapter._snapshot, raw_robot_state='MOVING')
            else:
                adapter._snapshot_read_fault = 'synthetic_read_fault'
        adapter._stop_transaction.on_request_started = alter
    monkeypatch.setattr(cleanup.RealLebaiAdapter, '_ensure_stop_transaction', transaction)
    result = await run(client, clock=clock)
    assert not result['complete'] and client.write_calls == [] and result['sdk_stop_entries'] == 0


async def test_missing_stop_capability_has_zero_writes():
    client = paused_client()
    client.stop_move = None
    result = await run(client)
    assert not result['complete'] and result['sdk_stop_entries'] == 0 and client.write_calls == []


@pytest.mark.parametrize('raw', [None, False, 0., '', [], {}])
async def test_ambiguous_raw_estop_cannot_authorize_stop(raw):
    client = paused_client()
    client.estop_reason = raw
    result = await run(client)
    assert not result['complete'] and client.write_calls == []


async def test_final_event_failure_still_closes_recorder():
    client, recorder = paused_client(), Recorder()
    recorder.close = AsyncMock()
    original = recorder.write_critical_event
    async def events(kind, payload, timestamp):
        if kind == 'paused_stop_cleanup_result':
            raise RuntimeError('synthetic_final_event_failure')
        await original(kind, payload, timestamp)
    recorder.write_critical_event = events
    result = await run(client, recorder=recorder)
    assert not result['complete']
    recorder.close.assert_awaited_once()


@pytest.mark.parametrize('stage', ['connected', 'verification', 'tail'])
async def test_uncancellable_read_has_bounded_exit_and_no_further_reads(monkeypatch, stage):
    client, recorder = paused_client(), Recorder()
    release, finished = asyncio.Event(), asyncio.Event()
    original_state = client.get_robot_state
    async def stuck():
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        finished.set()
        return 'IDLE'
    if stage == 'connected':
        client.is_connected = stuck
    else:
        if stage == 'verification':
            original_stop = client.stop_move
            async def stop():
                await original_stop()
                client.get_robot_state = stuck
            client.stop_move = stop
        else:
            original = cleanup.RealLebaiAdapter.read_stop_observation
            count = 0
            async def tail(adapter):
                nonlocal count
                count += 1
                if count == 3:
                    client.get_robot_state = stuck
                return await original(adapter)
            monkeypatch.setattr(cleanup.RealLebaiAdapter, 'read_stop_observation', tail)
    task = asyncio.create_task(run(client, recorder=recorder))
    try:
        result = await asyncio.wait_for(asyncio.shield(task), .8)
        assert not result['complete'] and result['read_remote_result_unknown']
        assert result['read_gate_closed']
        assert client.write_calls == ([] if stage=='connected' else [('stop_move',)])
        frozen = str(result)
        release.set()
        await finished.wait()
        await asyncio.sleep(0)
        assert str(result) == frozen
    finally:
        release.set()
        if not task.done():
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('motion_id', [None, 0])
async def test_paused_without_explicit_finished_motion_is_not_eligible(motion_id):
    client = paused_client()
    client.running_motion = motion_id
    result = await run(client)
    assert not result['complete'] and client.write_calls == []


@pytest.mark.parametrize('raw', [0, '0', 'NONE', ' none '])
async def test_explicit_raw_clear_estop_allows_only_scoped_stop(raw):
    client = paused_client()
    client.estop_reason = raw
    result = await run(client)
    assert result['stop_confirmed'] and client.write_calls == [('stop_move',)]
    assert not result['complete']  # Still PAUSED, never an automatic resume.
    assert all(c['raw_estop'] == raw for c in result['read_calls'] if c['method'] == 'get_estop_reason')


async def test_native_read_future_is_supported():
    client = paused_client()
    def connected():
        future = asyncio.get_running_loop().create_future()
        future.set_result(True)
        return future
    client.is_connected = connected
    result = await run(client)
    assert result['stop_confirmed'] and client.write_calls == [('stop_move',)]


@pytest.mark.parametrize('boundary', ['entry_count', 'total_time', 'child_expiry'])
async def test_read_budget_blocks_native_entry(monkeypatch, boundary):
    client = paused_client()
    time_ns = [1_000_000_000]
    monkeypatch.setattr(cleanup.time, 'monotonic_ns', lambda: time_ns[0])
    owner = asyncio.current_task()
    def admit():
        if boundary == 'child_expiry':
            time_ns[0] += 200_000_000
    proxy = cleanup.BoundedStopOnlyClient(client, lambda: None, admit, owner)
    if boundary == 'entry_count':
        monkeypatch.setattr(cleanup, 'MAX_READ_ENTRIES', 0)
    elif boundary == 'total_time':
        time_ns[0] = proxy.read_deadline_ns
    with pytest.raises(cleanup.AdmissionRejected):
        await proxy.is_connected()
    assert client.read_calls == [] and client.write_calls == []


async def test_late_read_reply_never_restores_permission_or_changes_evidence():
    client = paused_client()
    entered, release, finished = asyncio.Event(), asyncio.Event(), asyncio.Event()
    async def stuck():
        entered.set()
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        finished.set()
        return True
    client.is_connected = stuck
    proxy = cleanup.BoundedStopOnlyClient(client, lambda: None, lambda: None, asyncio.current_task())
    try:
        with pytest.raises(cleanup.AdmissionRejected, match='timeout'):
            await proxy.is_connected()
        frozen = str(proxy.read_calls)
        release.set()
        await finished.wait()
        await asyncio.sleep(0)
        with pytest.raises(cleanup.AdmissionRejected):
            await proxy.get_robot_state()
        assert str(proxy.read_calls) == frozen and client.read_calls == []
    finally:
        release.set()
        if proxy.pending_read is not None:
            await asyncio.gather(proxy.pending_read, return_exceptions=True)
