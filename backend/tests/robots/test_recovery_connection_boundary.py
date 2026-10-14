"""Queued recovery after a valid fake preflight; no native SDK/network."""
import asyncio

import pytest

from app.robots.base import BackendCommandError
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, HOME_OPTIONS
from tests.robots.fake_lebai import FakeLebaiClient
from app.robots.lebai_snapshot_calls import OwnedSnapshotCalls


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_queued_recovery_cannot_write_saved_client_after_disconnect(monkeypatch, action):
    adapter, client, _clock = await _connected_control_adapter()
    snapshot = adapter._snapshot
    # Simulate a completed fresh preflight before another operation holds the
    # SDK lock. This mock cannot grant permission after connection replacement.
    async def completed_preflight(**kwargs):
        return snapshot
    monkeypatch.setattr(adapter, '_read_snapshot', completed_preflight)
    reached_dispatch = asyncio.Event()
    await adapter._sdk_lock.acquire()
    task = asyncio.create_task(getattr(adapter, action)(HOME_OPTIONS,
        lambda _phase: reached_dispatch.set()))
    released = False
    try:
        await asyncio.wait_for(reached_dispatch.wait(), .5)
        await adapter.disconnect()
        assert adapter._client is None
        adapter._sdk_lock.release()
        released = True
        done, _ = await asyncio.wait({task}, timeout=.5)
        assert done
        with pytest.raises(BackendCommandError, match='^recovery_request_revoked$'):
            task.result()
        assert client.write_calls == [], 'invalidated recovery must not move or create a new stop'
        assert not adapter._motion_accepted
    finally:
        if not released:
            adapter._sdk_lock.release()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_cancel_after_recovery_sdk_entry_keeps_original_stop(monkeypatch, action):
    adapter, client, _clock = await _connected_control_adapter()
    entered = asyncio.Event()
    original_movej = client.movej
    async def pending_movej(*args):
        await original_movej(*args)
        entered.set()
        await asyncio.Event().wait()
    monkeypatch.setattr(client, 'movej', pending_movej)
    task = asyncio.create_task(getattr(adapter, action)(HOME_OPTIONS, lambda _phase: None))
    try:
        await asyncio.wait_for(entered.wait(), .5)
        task.cancel()
        done, _ = await asyncio.wait({task}, timeout=.8)
        assert done
        with pytest.raises(asyncio.CancelledError):
            task.result()
        assert [c[0] for c in client.write_calls] == ['movej', 'stop_move']
    finally:
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_cleanup_rechecks_connection_under_stop_lock(monkeypatch, action):
    adapter, old_client, _clock = await _connected_control_adapter()
    entered = asyncio.Event()
    original_movej = old_client.movej
    async def pending_movej(*args):
        await original_movej(*args)
        entered.set()
        await asyncio.Event().wait()
    monkeypatch.setattr(old_client, 'movej', pending_movej)
    task = asyncio.create_task(getattr(adapter, action)(HOME_OPTIONS, lambda _phase: None))
    new_client = FakeLebaiClient.idle()
    held = False
    try:
        await asyncio.wait_for(entered.wait(), .5)
        await adapter._stop_lock.acquire()
        held = True
        task.cancel()
        # Drain the nested MoveJ cancellation and queue the shielded cleanup.
        for _ in range(12):
            await asyncio.sleep(0)
        assert not task.done()
        # Model replacement being published while cleanup waits for stop lock.
        adapter._snapshot_calls.close()
        adapter._snapshot_calls = OwnedSnapshotCalls()
        adapter._client = new_client
        adapter._motion_accepted = False
        adapter._stop_lock.release()
        held = False
        done, _ = await asyncio.wait({task}, timeout=.8)
        assert done
        with pytest.raises(asyncio.CancelledError):
            task.result()
        assert [c[0] for c in old_client.write_calls] == ['movej']
        assert new_client.write_calls == [], 'cleanup identity check must occur under stop lock'
    finally:
        if held:
            adapter._stop_lock.release()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_recovery_rechecks_stop_generation_at_actual_sdk_boundary(monkeypatch, action):
    adapter, client, _clock = await _connected_control_adapter()
    events = []
    async def record(event, timestamp):
        events.append(event)
    adapter._event_callback = record
    original = asyncio.wait_for
    async def wait(awaitable, timeout):
        if getattr(getattr(awaitable, 'cr_code', None), 'co_name', None) == 'send_recovery_if_current':
            adapter._pump.invalidate()
        return await original(awaitable, timeout)
    monkeypatch.setattr(asyncio, 'wait_for', wait)
    try:
        with pytest.raises(BackendCommandError, match='^recovery_request_revoked$'):
            await getattr(adapter, action)(HOME_OPTIONS, lambda _phase: None)
        assert client.write_calls == [] and not adapter._motion_accepted
        await adapter.disconnect()  # Bounded diagnostic flush.
        lifecycle = [e for e in events if e['kind'] == 'sdk_request_lifecycle' and e['method'] == 'movej']
        assert len(lifecycle) == 1 and lifecycle[0]['rpc_outcome'] == 'not_started'
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_late_recovery_verification_cannot_clear_replacement_motion(monkeypatch, action):
    adapter, old_client, _clock = await _connected_control_adapter()
    entered = asyncio.Event()
    release = asyncio.Event()
    async def pending_state(_motion_id):
        entered.set()
        await release.wait()
        return 'FINISHED'
    monkeypatch.setattr(old_client, 'get_motion_state', pending_state)
    task = asyncio.create_task(getattr(adapter, action)(HOME_OPTIONS, lambda _phase: None))
    new_client = FakeLebaiClient.idle()
    try:
        await asyncio.wait_for(entered.wait(), .5)
        # Model a stale verification reply after connection replacement. The
        # replacement has its own motion obligation, which this task cannot clear.
        adapter._snapshot_calls.close()
        adapter._snapshot_calls = OwnedSnapshotCalls()
        adapter._client = new_client
        adapter._motion_accepted = True
        release.set()
        done, _ = await asyncio.wait({task}, timeout=.5)
        assert done
        with pytest.raises(BackendCommandError, match='^recovery_request_revoked$'):
            task.result()
        assert adapter._motion_accepted
        assert old_client.read_calls.count('get_motion_state') == 0
        assert [c[0] for c in old_client.write_calls] == ['movej']
        assert new_client.write_calls == []
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)
        # This test injected the flag without sending a replacement move.
        adapter._motion_accepted = False
        await adapter.disconnect()


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_cancelled_queued_recovery_never_stops_replacement(monkeypatch, action):
    adapter, old_client, _clock = await _connected_control_adapter()
    snapshot = adapter._snapshot
    async def completed_preflight(**kwargs):
        return snapshot
    monkeypatch.setattr(adapter, '_read_snapshot', completed_preflight)
    reached_dispatch = asyncio.Event()
    await adapter._sdk_lock.acquire()
    task = asyncio.create_task(getattr(adapter, action)(HOME_OPTIONS,
        lambda _phase: reached_dispatch.set()))
    released = False
    new_client = FakeLebaiClient.idle()
    try:
        await asyncio.wait_for(reached_dispatch.wait(), .5)
        await adapter.disconnect()
        async def new_factory(*args, **kwargs):
            return new_client
        adapter._client_factory = new_factory
        await adapter.connect()
        assert adapter._client is new_client
        task.cancel()
        # Let cancellation cleanup queue while the SDK lock is still held.
        await asyncio.sleep(0)
        adapter._sdk_lock.release()
        released = True
        done, _ = await asyncio.wait({task}, timeout=.5)
        assert done
        with pytest.raises(asyncio.CancelledError):
            task.result()
        assert old_client.write_calls == []
        assert new_client.write_calls == [], 'old recovery must not stop a replacement'
        assert not adapter._motion_accepted
    finally:
        if not released:
            adapter._sdk_lock.release()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()
