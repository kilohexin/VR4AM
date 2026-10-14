"""Explicit fake reads only; no native SDK or robot connections."""
import asyncio
from unittest.mock import AsyncMock

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock, _target

METHODS = ['is_connected', 'get_robot_state', 'get_estop_reason', 'get_kin_data',
    'get_tcp', 'get_claw', 'get_running_motion', 'get_motion_state']


async def setup(method):
    client, clock = FakeLebaiClient(), FakeClock()
    factory = AsyncMock(return_value=client)
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, sleep=clock.sleep, pvat_clock=clock.now_ns)
    await adapter.connect()
    entered, release, finished = asyncio.Event(), asyncio.Event(), asyncio.Event()
    original = getattr(client, method)
    calls = []
    async def stubborn(*args):
        calls.append(method)
        value = await original(*args)
        entered.set()
        try:
            try:
                await release.wait()
            except asyncio.CancelledError:
                await release.wait()
            return value
        finally:
            finished.set()
    if method == 'get_motion_state':
        client.running_motion = 416
    setattr(client, method, stubborn)
    before = adapter._snapshot
    task = asyncio.create_task(adapter.get_state())
    await asyncio.wait_for(entered.wait(), .5)
    return adapter, client, task, entered, release, finished, calls, before, factory


async def cleanup(adapter, task, release, finished):
    release.set()
    await finished.wait()
    await asyncio.gather(task, return_exceptions=True)
    try:
        await adapter.disconnect()
    except BackendCommandError:
        pass


@pytest.mark.parametrize('method', METHODS)
async def test_snapshot_budget_does_not_wait_for_native_cancel_ack(method):
    adapter, client, task, _, release, finished, calls, before, factory = await setup(method)
    try:
        done, _ = await asyncio.wait({task}, timeout=.45)
        assert done, '300 ms snapshot budget must not await native cancellation acknowledgment'
        with pytest.raises(BackendCommandError, match='robot_state_stale|sdk_timeout:get_motion_state'):
            task.result()
        assert not adapter._sdk_lock.locked() and adapter._snapshot is before
        previous = len(client.read_calls)
        with pytest.raises(BackendCommandError, match='snapshot_rpc_result_unknown'):
            await adapter.preflight()
        with pytest.raises(BackendCommandError, match='snapshot_rpc_result_unknown'):
            await adapter.submit_tcp(_target(.301), 1)
        assert len(client.read_calls) == previous and len(calls) == 1
        release.set()
        await finished.wait()
        await asyncio.sleep(0)
        assert adapter._snapshot is before and adapter.read_motion_view().accepted is None
        assert client.write_calls == []
    finally:
        await cleanup(adapter, task, release, finished)
        factory.assert_awaited_once()
        assert client.network_calls == client.discovery_calls == 0


async def test_stop_behind_uncooperative_snapshot_enters_once_but_is_not_confirmed():
    adapter, client, read, _, release, finished, _calls, _before, _factory = await setup('get_kin_data')
    stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        done, _ = await asyncio.wait({stop}, timeout=.5)
        assert done, 'stop must not remain blocked on a native read cancellation acknowledgment'
        with pytest.raises(BackendCommandError):
            stop.result()
        assert client.write_calls == [('stop_move',)]
        assert adapter.read_motion_view().latched_fault == 'stop_unverified'
        assert not adapter._stop_transaction.confirmed
        assert not release.is_set()
    finally:
        await cleanup(adapter, read, release, finished)
        await asyncio.gather(stop, return_exceptions=True)


async def test_cancelled_snapshot_is_bounded_and_late_result_cannot_publish():
    adapter, client, task, _, release, finished, _calls, before, _factory = await setup('get_kin_data')
    try:
        task.cancel()
        done, _ = await asyncio.wait({task}, timeout=.15)
        assert done, 'caller cancellation must not wait for native read cancellation acknowledgment'
        with pytest.raises(asyncio.CancelledError):
            task.result()
        assert not adapter._sdk_lock.locked()
        await adapter.disconnect()
        assert adapter._snapshot is None and adapter._client is None
        release.set()
        await finished.wait()
        await asyncio.sleep(0)
        assert adapter._snapshot is None and adapter.read_motion_view().sample is None
        with pytest.raises(BackendCommandError, match='snapshot_rpc_result_unknown'):
            await adapter.connect()
        assert client.write_calls == [] and before is not None
    finally:
        await cleanup(adapter, task, release, finished)


async def test_terminal_read_error_is_not_a_pending_native_result_and_keeps_fault_fence():
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    original = client.get_tcp
    async def error_response():
        await original()
        raise RuntimeError('terminal_fake_read_error')
    client.get_tcp = error_response
    try:
        with pytest.raises(BackendCommandError, match='^sdk_call_failed:get_tcp$'):
            await adapter.get_state()
        assert not adapter._snapshot_calls.unknown_methods
        client.get_tcp = original
        await adapter.get_state()
        assert adapter._snapshot_read_fault[1] == 'sdk_call_failed:get_tcp'
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('boundary', ['factory', 'first_connected'])
async def test_disconnect_during_connection_cannot_be_undone_by_late_return(boundary):
    client, clock = FakeLebaiClient(), FakeClock()
    entered, release, finished = asyncio.Event(), asyncio.Event(), asyncio.Event()
    async def pending(value):
        entered.set()
        try:
            try:
                await release.wait()
            except asyncio.CancelledError:
                await release.wait()
            return value
        finally:
            finished.set()
    async def factory(_ip):
        return await pending(client) if boundary == 'factory' else client
    if boundary == 'first_connected':
        async def connected():
            return await pending(True)
        client.is_connected = connected
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, sleep=clock.sleep)
    task = asyncio.create_task(adapter.connect())
    await entered.wait()
    try:
        await asyncio.wait_for(adapter.disconnect(), .15)
        done, _ = await asyncio.wait({task}, timeout=.15)
        assert done, 'disconnect must revoke the pending connection operation'
        assert adapter._client is None and not adapter._pump.running
        release.set()
        await finished.wait()
        await asyncio.sleep(0)
        assert adapter._client is None and adapter._snapshot is None
        assert not adapter._pump.running
        with pytest.raises(BackendCommandError, match='snapshot_rpc_result_unknown'):
            await adapter.connect()
        assert client.write_calls == []
    finally:
        release.set()
        await finished.wait()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


async def test_blocked_kinematics_recording_cannot_publish_old_preflight_after_disconnect():
    client, clock = FakeLebaiClient(), FakeClock()
    factory = AsyncMock(return_value=client)
    entered, release = asyncio.Event(), asyncio.Event()
    blocked = False
    async def record(event, timestamp):
        if (blocked and event['kind'] == 'robot_kinematics'
                and event.get('snapshot_read_purpose') == 'preflight'):
            entered.set()
            await release.wait()
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, sleep=clock.sleep, event_callback=record)
    await adapter.connect()
    blocked = True
    task = asyncio.create_task(adapter.preflight())
    await entered.wait()
    try:
        await adapter.disconnect()
        with pytest.raises(BackendCommandError, match='snapshot_read_pending'):
            await adapter.connect()
        factory.assert_awaited_once()
        release.set()
        with pytest.raises(BackendCommandError, match='snapshot_read_client_changed'):
            await task
        assert adapter._client is None and adapter._snapshot is None
        assert not adapter._preflight_ready and not adapter._pump.running
        blocked = False
        await adapter.connect()
        assert adapter._preflight_ready
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


async def test_proxy_reply_validation_failure_is_not_native_result_unknown():
    client, clock = FakeLebaiClient(), FakeClock()
    client.estop_reason = None
    proxy = BoundedStopOnlyClient(client, lambda: None, lambda: None, asyncio.current_task())
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=proxy),
        clock=clock.now_ns, sleep=clock.sleep)
    try:
        with pytest.raises(BackendCommandError, match='paused_cleanup_ambiguous_estop'):
            await adapter.connect()
        record = adapter._snapshot_calls.last['get_estop_reason']
        assert record['entered'] and not record['remote_result_unknown']
        assert not adapter._snapshot_calls.unknown_methods
        assert not proxy.read_remote_result_unknown
        assert proxy.read_calls[-1]['native_completed_ns'] is not None
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_proxy_timeout_keeps_underlying_native_result_unknown():
    client, clock = FakeLebaiClient(), FakeClock()
    entered, release, finished = asyncio.Event(), asyncio.Event(), asyncio.Event()
    original = client.get_kin_data
    async def pending():
        entered.set()
        try:
            try:
                await release.wait()
            except asyncio.CancelledError:
                await release.wait()
            return await original()
        finally:
            finished.set()
    client.get_kin_data = pending
    proxy = BoundedStopOnlyClient(client, lambda: None, lambda: None, asyncio.current_task())
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=proxy),
        clock=clock.now_ns, sleep=clock.sleep)
    try:
        with pytest.raises(BackendCommandError, match='paused_cleanup_read_timeout'):
            await adapter.connect()
        assert entered.is_set() and proxy.read_remote_result_unknown
        assert adapter._snapshot_calls.last['get_kin_data']['remote_result_unknown']
        assert adapter._snapshot_calls.unknown_methods == {'get_kin_data'}
        assert client.write_calls == []
    finally:
        release.set()
        await finished.wait()
        await adapter.disconnect()


@pytest.mark.parametrize('method', METHODS)
async def test_snapshot_native_future_is_supported(method):
    client, clock = FakeLebaiClient(), FakeClock()
    if method == 'get_motion_state':
        client.running_motion = 416
    original = getattr(client, method)
    def future(*args):
        return asyncio.ensure_future(original(*args))
    setattr(client, method, future)
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    try:
        state = await adapter.get_state()
        assert state.actual_tcp.p == (.3, 0., .4)
        assert not adapter._snapshot_calls.unknown_methods
        assert adapter._snapshot_calls.last[method]['outcome'] == 'returned'
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_connection_revoked_before_factory_native_task_never_calls_factory(monkeypatch):
    client, clock = FakeLebaiClient(), FakeClock()
    factory = AsyncMock(return_value=client)
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, sleep=clock.sleep)
    original = asyncio.create_task
    def schedule(coro, *args, **kwargs):
        if kwargs.get('name') == 'owned-motion-connect':
            adapter._snapshot_calls.close()
        return original(coro, *args, **kwargs)
    monkeypatch.setattr(asyncio, 'create_task', schedule)
    with pytest.raises(BackendCommandError, match='sdk_timeout:connect'):
        await adapter.connect()
    factory.assert_not_awaited()
    record = adapter._snapshot_calls.last['connect']
    assert not record['entered'] and not record['remote_result_unknown']
    assert record['abandon_reason'] == 'authority_revoked'
    assert adapter._client is None and not adapter._pump.running
    await adapter.disconnect()


async def test_first_connected_timeout_is_bounded_without_disconnection():
    client, clock = FakeLebaiClient(), FakeClock()
    entered, release, finished = asyncio.Event(), asyncio.Event(), asyncio.Event()
    async def connected():
        entered.set()
        try:
            try:
                await release.wait()
            except asyncio.CancelledError:
                await release.wait()
            return True
        finally:
            finished.set()
    client.is_connected = connected
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, sleep=clock.sleep)
    task = asyncio.create_task(adapter.connect())
    await entered.wait()
    try:
        done, _ = await asyncio.wait({task}, timeout=.35)
        assert done
        with pytest.raises(BackendCommandError, match='sdk_timeout:is_connected'):
            task.result()
        assert adapter._snapshot_calls.unknown_methods == {'initial_is_connected'}
        assert adapter._client is None
        release.set()
        await finished.wait()
        await asyncio.sleep(0)
        assert adapter._client is None and not adapter._pump.running
    finally:
        release.set()
        await finished.wait()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


async def test_proxy_abandoned_native_late_finish_cannot_clear_outer_unknown():
    client, clock = FakeLebaiClient(), FakeClock()
    release, finished = asyncio.Event(), asyncio.Event()
    original = client.get_kin_data
    async def pending():
        try:
            await release.wait()
        except asyncio.CancelledError:
            await release.wait()
        finished.set()
        return await original()
    client.get_kin_data = pending
    class LateProxy(BoundedStopOnlyClient):
        async def read(self, name, invoke):
            try:
                return await super().read(name, invoke)
            except BackendCommandError:
                if name == 'get_kin_data':
                    release.set()
                    await finished.wait()
                    await asyncio.sleep(0)
                    assert self.pending_read.done()
                raise
    proxy = LateProxy(client, lambda: None, lambda: None, asyncio.current_task())
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=proxy),
        clock=clock.now_ns, sleep=clock.sleep)
    try:
        with pytest.raises(BackendCommandError, match='paused_cleanup_read_timeout'):
            await adapter.connect()
        assert proxy.read_calls[-1]['remote_result_unknown']
        assert adapter._snapshot_calls.last['get_kin_data']['remote_result_unknown']
        assert adapter._snapshot_calls.unknown_methods == {'get_kin_data'}
    finally:
        release.set()
        await finished.wait()
        await adapter.disconnect()
