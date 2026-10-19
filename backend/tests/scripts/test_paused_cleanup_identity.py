"""Loaded file identity at the one stop boundary; all clients are in-process Fake."""
import asyncio
from datetime import datetime
import json
from pathlib import Path
from types import SimpleNamespace
import os
from unittest.mock import AsyncMock

import pytest

from app.commissioning import paused_stop_cleanup as cleanup, readonly_kinematics
from tests.scripts.test_paused_stop_cleanup_entry import context
from tests.commissioning.test_paused_stop_cleanup import paused_client
from tests.robots.test_lebai_adapter_control import FakeClock


def at_stop_entry(monkeypatch, action):
    original = cleanup.PausedStopOnlyAdapter._ensure_stop_transaction
    def transaction(adapter):
        original(adapter)
        adapter._stop_transaction.on_request_started = lambda _method: action()
    monkeypatch.setattr(cleanup.PausedStopOnlyAdapter, '_ensure_stop_transaction', transaction)


def idle_after_stop(client):
    original = client.stop_move
    async def stop():
        await original()
        client.robot_state = 'IDLE'
    client.stop_move = stop


async def test_slow_full_hashes_stay_before_fresh_reads_and_cleanup_completes(context, monkeypatch):
    entry, plan, kw = context
    clock, client = FakeClock(), paused_client()
    monkeypatch.setattr(entry, 'monotonic_ns', clock.now_ns)
    monkeypatch.setattr(readonly_kinematics, 'monotonic_ns', clock.now_ns)
    monkeypatch.setattr(cleanup.time, 'monotonic_ns', clock.now_ns)
    original_run = cleanup.run_cleanup
    async def timed(*args, **kwargs):
        return await original_run(*args, **kwargs, clock=clock.now_ns, sleep=clock.sleep)
    monkeypatch.setattr(cleanup, 'run_cleanup', timed)
    reads = 0
    original_read = client.get_kin_data
    async def read():
        nonlocal reads
        reads += 1
        clock.advance_ms(47.)
        return await original_read()
    client.get_kin_data = read
    idle_after_stop(client)
    original_hashes = entry.runtime_hashes
    costs = []
    def slow_hashes():
        assert reads < 3 or client.write_calls, 'cold hashes after the final fresh read'
        clock.advance_ms(83.)
        costs.append(83.)
        return original_hashes()
    monkeypatch.setattr(entry, 'runtime_hashes', slow_hashes)
    result = await entry.execute(plan, **kw, client_factory=AsyncMock(return_value=client))
    assert result['complete'] and result['stop_confirmed'] and costs
    assert result['stop_entry_rejection'] is None
    assert result['sdk_stop_entries'] == 1 and client.write_calls == [('stop_move',)]
    assert json.loads((plan['output'] / 'result.json').read_text()) == result


@pytest.mark.parametrize('change,reason', [
    ('code', 'paused_cleanup_loaded_runtime_metadata_changed'),
    ('config', 'paused_cleanup_configuration_changed'),
    ('copied_config', 'paused_cleanup_copied_configuration_changed'),
    ('approval', 'paused_cleanup_scope_or_hash_changed'),
    ('expiry', 'paused_cleanup_expiry_invalid_or_expired'),
])
async def test_boundary_change_refuses_stop_and_records_reason(context, monkeypatch, change, reason):
    entry, plan, kw = context
    client = paused_client()
    def changed():
        if change == 'code':
            (entry.ROOT / 'synthetic').write_text('changed loaded file\n')
        elif change == 'config':
            plan['config'].write_text(plan['config'].read_text() + '\n# changed')
        elif change == 'copied_config':
            path = plan['output'] / 'config-control.yaml'
            path.write_text(path.read_text() + '\n# changed')
        elif change == 'approval':
            kw['approved'].write_text('{}')
        else:
            expired = datetime.fromisoformat(kw['expires'])
            monkeypatch.setattr(entry, 'datetime', SimpleNamespace(now=lambda _zone: expired,
                fromisoformat=datetime.fromisoformat))
    at_stop_entry(monkeypatch, changed)
    result = await entry.execute(plan, **kw, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['latched_fault'] == 'stop_unverified'
    assert result['sdk_stop_entries'] == 0 and client.write_calls == []
    assert result['stop_entry_rejection'] == reason


async def test_metadata_change_during_initial_hashing_cannot_pin_changed_file(context, monkeypatch):
    entry, plan, kw = context
    original = entry.runtime_hashes
    def mutate_after_hash():
        values = original()
        (entry.ROOT / 'synthetic').write_text('changed after byte hashing\n')
        return values
    monkeypatch.setattr(entry, 'runtime_hashes', mutate_after_hash)
    factory = AsyncMock(side_effect=AssertionError('untrusted loaded code connected'))
    with pytest.raises(ValueError, match='runtime_changed_during_hashes'):
        await entry.execute(plan, **kw, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


async def test_expiry_during_native_metadata_check_is_not_accepted(context, monkeypatch):
    entry, plan, kw = context
    client = paused_client()
    started = False
    def starting():
        nonlocal started
        started = True
    at_stop_entry(monkeypatch, starting)
    assert hasattr(entry, 'runtime_metadata'), 'metadata guard missing'
    original = entry.runtime_metadata
    def expire_during_stat(paths):
        values = original(paths)
        if started:
            expired = datetime.fromisoformat(kw['expires'])
            monkeypatch.setattr(entry, 'datetime', SimpleNamespace(now=lambda _zone: expired,
                fromisoformat=datetime.fromisoformat))
        return values
    monkeypatch.setattr(entry, 'runtime_metadata', expire_during_stat)
    result = await entry.execute(plan, **kw, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['sdk_stop_entries'] == 0 and client.write_calls == []
    assert result['stop_entry_rejection'] == 'paused_cleanup_expiry_invalid_or_expired'


async def test_expiry_during_initial_metadata_pin_cannot_reserve_output(context, monkeypatch):
    entry, plan, kw = context
    original = entry.runtime_metadata
    count = 0
    def expire_after_hash(paths):
        nonlocal count
        count += 1
        values = original(paths)
        if count == 2:
            expired = datetime.fromisoformat(kw['expires'])
            monkeypatch.setattr(entry, 'datetime', SimpleNamespace(now=lambda _zone: expired,
                fromisoformat=datetime.fromisoformat))
        return values
    monkeypatch.setattr(entry, 'runtime_metadata', expire_after_hash)
    factory = AsyncMock(side_effect=AssertionError('expired pin connected'))
    with pytest.raises(ValueError, match='expired'):
        await entry.execute(plan, **kw, client_factory=factory)
    assert not plan['output'].exists()
    factory.assert_not_called()


@pytest.mark.parametrize('change', ['removed', 'same_bytes_replaced'])
async def test_loaded_file_identity_cannot_disappear_or_be_replaced(context, monkeypatch, change):
    entry, plan, kw = context
    client = paused_client()
    def mutate():
        path = entry.ROOT / 'synthetic'
        if change == 'removed':
            path.unlink()
        else:
            stat = path.stat()
            replacement = entry.ROOT / 'synthetic-replacement'
            replacement.write_bytes(path.read_bytes())
            os.utime(replacement, ns=(stat.st_atime_ns, stat.st_mtime_ns))
            replacement.replace(path)
    at_stop_entry(monkeypatch, mutate)
    result = await entry.execute(plan, **kw, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['sdk_stop_entries'] == 0 and client.write_calls == []
    assert result['stop_entry_rejection'] == ('paused_cleanup_runtime_metadata_unavailable' if change == 'removed'
        else 'paused_cleanup_loaded_runtime_metadata_changed')


async def test_runtime_path_outside_repo_is_not_pinned(context):
    entry, plan, kw = context
    outside = entry.ROOT.parent / (entry.ROOT.name + '-external')
    outside.write_text('test-owned external code fixture\n')
    plan['runtime_hashes'] = {'../' + outside.name: entry.sha(outside)}
    kw['approved'].write_text(json.dumps(entry.preview(plan)))
    kw['expected_hash'] = entry.sha(kw['approved'])
    factory = AsyncMock(side_effect=AssertionError('external runtime path connected'))
    with pytest.raises(ValueError, match='runtime_path_invalid'):
        await entry.execute(plan, **kw, client_factory=factory)
    assert not plan['output'].exists()
    factory.assert_not_called()


async def test_native_late_reply_keeps_original_timeout_and_one_write(context):
    entry, plan, kw = context
    client = paused_client()
    idle_after_stop(client)
    original = client.stop_move
    async def late():
        await original()
        await asyncio.sleep(.25)
    client.stop_move = late
    result = await entry.execute(plan, **kw, client_factory=AsyncMock(return_value=client))
    assert not result['complete'] and result['latched_fault'] == 'stop_unverified'
    assert result['sdk_stop_entries'] == 1 and client.write_calls == [('stop_move',)]
    assert result['stop_entry_rejection'] is None


async def test_owner_cancellation_after_stop_does_not_repeat_write(context):
    entry, plan, kw = context
    client = paused_client()
    entered, release = asyncio.Event(), asyncio.Event()
    original = client.stop_move
    async def pending():
        await original()
        entered.set()
        await release.wait()
    client.stop_move = pending
    task = asyncio.create_task(entry.execute(plan, **kw, client_factory=AsyncMock(return_value=client)))
    try:
        await asyncio.wait_for(entered.wait(), 2.)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 2.)
        result = json.loads((plan['output'] / 'result.json').read_text())
        assert not result['complete'] and result['latched_fault'] == 'stop_unverified'
        assert result['sdk_stop_entries'] == 1 and client.write_calls == [('stop_move',)]
        assert result['stop_entry_rejection'] is None
    finally:
        release.set()
        if not task.done():task.cancel()
        await asyncio.gather(task, return_exceptions=True)
