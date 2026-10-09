import asyncio
from unittest.mock import AsyncMock

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_observer_window import ObserverWindow
from app.robots.lebai_pump import PvatRequest
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target


@pytest.mark.asyncio
async def test_observer_window_is_disabled_by_default():
    adapter, client, clock = await _connected_control_adapter()
    try:
        assert adapter._observer_window.max_wait_s == 0
        adapter._observer_window.wait = AsyncMock(side_effect=AssertionError('unexpected wait'))
        await adapter.get_state()
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_budget_expiry_allows_full_observation_without_finishing_motion():
    adapter, client, clock = await _connected_control_adapter()
    window = adapter._observer_window = ObserverWindow(.001)
    lease = window.begin()
    events = []

    async def record(event, timestamp):
        events.append(event)

    adapter._event_callback = record
    client.read_calls.clear()
    try:
        await asyncio.wait_for(adapter.get_state(), 1)
        assert window.active
        for name in ['get_robot_state', 'get_estop_reason', 'get_kin_data',
                     'get_tcp', 'get_claw', 'get_running_motion']:
            assert client.read_calls.count(name) == 1
        timing = events[0]['diagnostic_timing']['observer_window']
        assert timing['budget_s'] == .001
        assert timing['completed_ns'] >= timing['requested_ns']
        assert timing['alignment_verified'] is False
    finally:
        window.finish(lease)
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('purpose', ['motion', 'stop_verification', 'stop_observation'])
async def test_protected_snapshot_reads_never_wait_for_observer_window(purpose):
    adapter, client, clock = await _connected_control_adapter()
    window = adapter._observer_window = ObserverWindow(.02)
    lease = window.begin()
    window.wait = AsyncMock(side_effect=AssertionError('protected read delayed'))
    try:
        await adapter._read_snapshot(read_purpose=purpose, emit_kinematics=False)
        window.wait.assert_not_awaited()
    finally:
        window.finish(lease)
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('bypass', ['fault', 'stopping', 'disconnecting'])
async def test_observations_bypass_window_during_urgent_local_state(bypass):
    adapter, client, clock = await _connected_control_adapter()
    window = adapter._observer_window = ObserverWindow(.02)
    lease = window.begin()
    window.wait = AsyncMock(side_effect=AssertionError('urgent observation delayed'))
    if bypass == 'fault':
        adapter._latched_fault = 'stop_unverified'
    elif bypass == 'stopping':
        await adapter._stop_lock.acquire()
    else:
        adapter._disconnecting += 1
    try:
        await adapter.get_state()
        window.wait.assert_not_awaited()
    finally:
        if bypass == 'stopping':
            adapter._stop_lock.release()
        if bypass == 'disconnecting':
            adapter._disconnecting -= 1
        window.finish(lease)
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('entry', ['stop', 'stop_unverified', 'disconnect'])
async def test_urgent_entry_releases_all_observer_leases(entry):
    adapter, client, clock = await _connected_control_adapter()
    window = adapter._observer_window = ObserverWindow(.02)
    window.begin()
    window.begin()
    waiting = asyncio.create_task(window.wait())
    await asyncio.sleep(0)
    try:
        if entry == 'stop':
            adapter._stop_in_transaction = AsyncMock()
            await adapter.stop(StopReason.GRIP_RELEASED)
        elif entry == 'stop_unverified':
            adapter._latch_unverified_stop()
        else:
            await adapter.disconnect()
        await asyncio.wait_for(waiting, 1)
        assert not window.active
    finally:
        if not waiting.done():
            waiting.cancel()
        await asyncio.gather(waiting, return_exceptions=True)
        if adapter._client is not None:
            await adapter.disconnect()


@pytest.mark.asyncio
async def test_enabled_window_coordinates_full_observer_after_motion():
    """Control the unexpired-budget case; separate tests cover expiry."""
    adapter, client, clock = await _connected_control_adapter()
    window = adapter._observer_window = ObserverWindow(.02)
    motion_read, release_motion, observer_wait = asyncio.Event(), asyncio.Event(), asyncio.Event()
    original_read = client.get_kin_data
    reads = 0

    async def read():
        nonlocal reads
        reads += 1
        if reads == 1:
            motion_read.set()
            await release_motion.wait()
        else:
            assert [x[0] for x in client.write_calls] == ['move_pvat']
        return await original_read()

    async def unexpired_wait():
        observer_wait.set()
        await window._idle.wait()

    window.wait = unexpired_wait
    client.get_kin_data = read
    client.read_calls.clear()
    await adapter._pump.start()
    sending = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    observer = None
    try:
        await asyncio.wait_for(motion_read.wait(), 1)
        observer = asyncio.create_task(adapter.get_state())
        await asyncio.wait_for(observer_wait.wait(), 1)
        release_motion.set()
        await asyncio.wait_for(sending, 1)
        await asyncio.wait_for(observer, 1)
        assert not window.active
        for name in ['get_robot_state', 'get_estop_reason', 'get_kin_data',
                     'get_tcp', 'get_claw', 'get_running_motion']:
            assert client.read_calls.count(name) == 2
    finally:
        release_motion.set()
        window.release_all()
        for task in [sending, observer]:
            if task is not None and not task.done():
                task.cancel()
        await asyncio.gather(*[x for x in [sending, observer] if x is not None], return_exceptions=True)
        client.get_kin_data = original_read
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_pump_fault_appearing_during_wait_is_rechecked():
    adapter, client, clock = await _connected_control_adapter()
    window = adapter._observer_window = ObserverWindow(.02)

    async def fail_during_wait():
        adapter._pump._fault = BackendCommandError('injected_pump_fault')

    window.wait = fail_during_wait
    client.read_calls.clear()
    try:
        with pytest.raises(BackendCommandError, match='injected_pump_fault'):
            await adapter.get_state()
        assert client.read_calls == []
    finally:
        await adapter.disconnect()
