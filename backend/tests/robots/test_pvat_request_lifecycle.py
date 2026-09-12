import asyncio

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_pump import PvatRequest
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target


@pytest.mark.asyncio
@pytest.mark.parametrize('outcome', ['returned', 'timeout', 'cancelled', 'error'])
async def test_inflight_pvat_is_audited_even_when_invalidated(outcome):
    adapter, client, clock = await _connected_control_adapter()
    events = []
    lock_states = []
    entered = asyncio.Event()
    release = asyncio.Event()

    async def record(event, timestamp):
        if event['kind'] == 'sdk_request_lifecycle':
            lock_states.append(adapter._sdk_lock.locked())
        events.append(event)

    async def send(*args):
        entered.set()
        await release.wait()
        if outcome == 'error':
            raise RuntimeError('test transport error')

    adapter._event_callback = record
    client.move_pvat = send
    await adapter._pump.start()
    request = PvatRequest(_target(), 41, adapter._pump._generation)
    task = asyncio.create_task(adapter._send_target(request))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        adapter._pump.invalidate()
        if outcome == 'cancelled':
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
        else:
            if outcome != 'timeout':
                release.set()
            await task
        records = [e for e in events if e['kind'] == 'sdk_request_lifecycle']
        assert len(records) == 1
        assert lock_states == [False]
        record = records[0]
        assert record['rpc_outcome'] == outcome
        assert record['invalidated'] is True
        assert record['command_id'] == 41
        assert record['generation'] == request.generation
        assert record['completed_ns'] >= record['started_ns']
        assert len(record['p']) == len(record['v']) == len(record['a']) == 6
        assert record['horizon_s'] == .08
        assert not any(e['kind'] == 'pvat_sent' for e in events)
    finally:
        release.set()
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_successful_requests_have_distinct_ids_and_keep_sent_events():
    adapter, client, clock = await _connected_control_adapter()
    events = []

    async def record(event, timestamp):
        events.append(event)

    adapter._event_callback = record
    await adapter._pump.start()
    try:
        for command_id in (51, 52):
            client.ik_results.append(list(client.kin_data['actual_joint_pose']))
            await adapter._send_target(
                PvatRequest(_target(), command_id, adapter._pump._generation)
            )
        await asyncio.sleep(0)
        records = [e for e in events if e['kind'] == 'sdk_request_lifecycle']
        assert len(records) == 2
        assert records[0]['request_id'] != records[1]['request_id']
        assert all(e['rpc_outcome'] == 'returned' and not e['invalidated'] for e in records)
        assert len([e for e in events if e['kind'] == 'pvat_sent']) == 2
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_blocked_diagnostic_does_not_hold_sdk_lock_or_hide_timeout():
    adapter, client, clock = await _connected_control_adapter()
    recording = asyncio.Event()

    async def send(*args):
        await asyncio.Event().wait()

    async def record(event, timestamp):
        if event['kind'] == 'sdk_request_lifecycle':
            recording.set()
            await asyncio.Event().wait()

    adapter._event_callback = record
    client.move_pvat = send
    await adapter._pump.start()
    task = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 61, adapter._pump._generation)
    ))
    try:
        await asyncio.wait_for(recording.wait(), 1)
        assert task.done(), 'diagnostic recording must not delay SDK error propagation'
        await asyncio.wait_for(adapter._sdk_lock.acquire(), .1)
        adapter._sdk_lock.release()
        with pytest.raises(BackendCommandError, match='sdk_timeout:move_pvat'):
            await asyncio.wait_for(task, .5)
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_diagnostic_tasks_are_bounded_even_if_callback_ignores_cancel():
    adapter, client, clock = await _connected_control_adapter()
    release = asyncio.Event()

    async def record(event, timestamp):
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                continue

    adapter._event_callback = record
    try:
        for request_id in range(20):
            adapter._schedule_sdk_diagnostic({'kind': 'sdk_request_lifecycle', 'request_id': request_id})
        await asyncio.sleep(.07)
        assert len(adapter._sdk_diagnostic_tasks) == 16
        assert adapter._sdk_diagnostics_dropped == 4
        adapter._schedule_sdk_diagnostic({'kind': 'sdk_request_lifecycle'})
        assert adapter._sdk_diagnostics_dropped == 5
    finally:
        release.set()
        await adapter.disconnect()
    assert not adapter._sdk_diagnostic_tasks
