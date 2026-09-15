import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.control.robot_control import RobotControl, LatestVRFrame
from app.recording.noop import NoopRecorder
from tests.robots.test_lebai_adapter_control import (
    _connected_control_adapter, _target, _wait_for_pvat_count,
)


async def test_production_stop_reuses_pending_requests_and_preserves_late_evidence():
    adapter, client, _ = await _connected_control_adapter()
    release = asyncio.Event()
    cancelled = []
    events = []

    async def record(event, timestamp):
        events.append(event)

    async def blocked(method):
        client.write_calls.append((method,))
        try:
            await release.wait()
        except asyncio.CancelledError:
            cancelled.append(method)
            raise

    client.stop_move = lambda: blocked('stop_move')
    client.stop_sys = lambda: blocked('stop_sys')
    adapter._event_callback = record
    try:
        with pytest.raises(BackendCommandError, match='sdk_timeout:stop_move'):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert not cancelled
        with pytest.raises(BackendCommandError, match='sdk_timeout:stop_move'):
            await asyncio.wait_for(adapter.stop(StopReason.SHUTDOWN), .1)
        assert client.write_calls == [('stop_move',), ('stop_sys',)]
        release.set()
        for _ in range(20):
            await asyncio.sleep(0)
        lifecycle = [e for e in events if e['kind'] == 'stop_rpc_lifecycle']
        assert {e['method'] for e in lifecycle} == {'stop_move', 'stop_sys'}
        assert all(e['outcome'] == 'returned_late' for e in lifecycle)
        assert (await adapter.get_state()).fault == 'stop_unverified'
        diagnostics = [e for e in events if e['kind'] == 'stop_diagnostics']
        assert diagnostics[0]['episode_id'] == diagnostics[1]['episode_id']
        calls = [[c for c in d['rpc_calls'] if c['method'] == 'stop_move'][0] for d in diagnostics]
        assert calls[0]['request_id'] == calls[1]['request_id']
        assert calls[0]['reused'] is False and calls[1]['reused'] is True
    finally:
        release.set()
        await adapter.disconnect()


async def test_repeated_confirmed_stop_does_not_resend_motion_stop():
    adapter, client, _ = await _connected_control_adapter()
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        await adapter.stop(StopReason.SHUTDOWN)
        assert client.write_calls == [('stop_move',)]
    finally:
        await adapter.disconnect()


async def test_new_motion_gets_a_new_stop_episode():
    adapter, client, _ = await _connected_control_adapter()
    events = []

    async def record(event, timestamp):
        events.append(event)

    adapter._event_callback = record
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert (await adapter.preflight()).ready
        await adapter.command_tcp(_target(), 1)
        await _wait_for_pvat_count(client, 1)
        await adapter.stop(StopReason.GRIP_RELEASED)
        diagnostics = [e for e in events if e['kind'] == 'stop_diagnostics']
        assert diagnostics[0]['episode_id'] != diagnostics[1]['episode_id']
        assert [c[0] for c in client.write_calls].count('stop_move') == 2
    finally:
        await adapter.disconnect()


async def test_drift_after_confirmed_stop_is_not_hidden_by_reuse():
    adapter, client, clock = await _connected_control_adapter()
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        clock.advance_ms(1000)
        client.kin_data['actual_tcp_pose']['x'] += .01
        with pytest.raises(BackendCommandError, match='stop_incomplete'):
            await adapter.stop(StopReason.SHUTDOWN)
        assert client.write_calls == [('stop_move',), ('stop_sys',)]
        assert (await adapter.get_state()).fault == 'stop_incomplete'
    finally:
        await adapter.disconnect()


async def test_disconnect_releases_local_resources_even_if_stop_fails():
    adapter, client, _ = await _connected_control_adapter()

    async def failed_stop():
        client.write_calls.append(('stop_move',))
        raise RuntimeError('failed')

    client.stop_move = failed_stop
    await adapter.command_tcp(_target(), 1)
    await _wait_for_pvat_count(client, 1)
    try:
        with pytest.raises(BackendCommandError, match='sdk_call_failed:stop_move'):
            await adapter.disconnect()
        assert not adapter._pump.running
        assert adapter._client is None
    finally:
        await adapter._pump.stop()  # Test cleanup even if the regression returns.


async def test_slow_verification_cannot_confirm_after_its_fixed_deadline():
    adapter, client, clock = await _connected_control_adapter()
    original = client.get_kin_data
    delays = iter([200, 280])

    async def slow_read():
        clock.advance_ms(next(delays, 280))
        return await original()

    client.get_kin_data = slow_read
    try:
        with pytest.raises(BackendCommandError, match='stop_incomplete'):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == [('stop_move',), ('stop_sys',)]
    finally:
        await adapter.disconnect()


async def test_disconnect_waits_for_in_progress_escalation_before_closing():
    adapter, client, _ = await _connected_control_adapter()
    checking = asyncio.Event()
    release = asyncio.Event()
    original_connected = client.is_connected

    async def failed_stop():
        client.write_calls.append(('stop_move',))
        raise RuntimeError('failed')

    async def connected_during_escalation():
        checking.set()
        await release.wait()
        return await original_connected()

    client.stop_move = failed_stop
    client.is_connected = connected_during_escalation
    stopping = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    closing = None
    try:
        await asyncio.wait_for(checking.wait(), 1)
        closing = asyncio.create_task(adapter.disconnect())
        # Give disconnect sufficient scheduling turns to expose the race.
        for _ in range(10):
            await asyncio.sleep(0)
        assert not closing.done()
        with pytest.raises(BackendCommandError, match='disconnect_in_progress'):
            await adapter.command_tcp(_target(), 10)
        with pytest.raises(BackendCommandError, match='disconnect_in_progress'):
            await adapter.connect()
        release.set()
        with pytest.raises(BackendCommandError, match='sdk_call_failed:stop_move'):
            await stopping
        await closing
        assert client.write_calls == [('stop_move',), ('stop_sys',)]
        assert adapter._client is None
        assert all(r.task.done() for r in adapter._stop_transaction.requests.values())
    finally:
        release.set()
        await asyncio.gather(stopping, *( [closing] if closing else [] ), return_exceptions=True)
        await adapter.disconnect()


async def test_reconnect_does_not_reuse_closed_connection_stop_requests():
    adapter, client, _ = await _connected_control_adapter()
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        await adapter.disconnect()
        await adapter.connect()
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == [('stop_move',), ('stop_move',)]
    finally:
        await adapter.disconnect()


async def test_control_shutdown_does_not_cancel_a_stop_already_in_progress():
    adapter, client, clock = await _connected_control_adapter()
    started = asyncio.Event()
    release = asyncio.Event()
    original = client.stop_move

    async def held_stop():
        started.set()
        await release.wait()
        await original()

    client.stop_move = held_stop
    control = RobotControl(backend=adapter, latest=LatestVRFrame(), clock=clock,
                           recorder=NoopRecorder())
    worker = asyncio.create_task(control._stop_backend(StopReason.DISCONNECT))
    control._task = worker
    shutdown = None
    try:
        await asyncio.wait_for(started.wait(), 1)
        shutdown = asyncio.create_task(control.stop())
        for _ in range(10):
            await asyncio.sleep(0)
        assert not worker.done()
        release.set()
        await shutdown
        assert not worker.cancelled()
        assert (await adapter.get_state()).fault is None
        assert client.write_calls == [('stop_move',)]
    finally:
        release.set()
        await asyncio.gather(worker, *([shutdown] if shutdown else []), return_exceptions=True)
        await adapter.disconnect()
