"""Normal adapter with explicit fake SDK; no network or native imports."""
import asyncio
from unittest.mock import AsyncMock

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock, HOME_OPTIONS, _frame, _target


class StubbornClient(FakeLebaiClient):
    def __init__(self, method):
        super().__init__()
        self.method = method
        self.entered, self.release, self.finished = asyncio.Event(), asyncio.Event(), asyncio.Event()

    async def stubborn(self):
        self.entered.set()
        try:
            try:
                await self.release.wait()
            except asyncio.CancelledError:
                await self.release.wait()
        finally:
            self.finished.set()

    async def kinematics_inverse(self, pose, joints):
        if self.method == 'ik':
            self.ik_calls.append((pose.copy(), list(joints)))
            await self.stubborn()
            return [IDLE_Q[0] + .01, *IDLE_Q[1:]]
        return await super().kinematics_inverse(pose, joints)

    async def move_pvat(self, *args):
        result = await super().move_pvat(*args)
        if self.method == 'pvat':
            await self.stubborn()
        return result


async def setup(method):
    client, clock = StubbornClient(method), FakeClock()
    factory = AsyncMock(return_value=client)
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    receipt = await adapter.submit_tcp(_target(.301), 1)
    await asyncio.wait_for(client.entered.wait(), .5)
    return adapter, client, receipt, factory


async def cleanup(adapter, client, task=None):
    client.release.set()
    if client.entered.is_set():
        await client.finished.wait()
    if task is not None:
        await asyncio.gather(task, return_exceptions=True)
    try:
        await adapter.disconnect()
    except BackendCommandError:
        pass


@pytest.mark.parametrize('method', ['ik', 'pvat'])
async def test_uncooperative_motion_call_timeout_releases_sdk_lock(method):
    adapter, client, receipt, factory = await setup(method)
    timely = False
    try:
        try:
            await asyncio.wait_for(receipt.completion.wait(), .35 if method == 'ik' else .18)
            timely = True
        except TimeoutError:
            pass
        assert timely, 'native cancellation acknowledgment must not extend the local wait'
        assert not adapter._sdk_lock.locked()
        assert not (await adapter.preflight()).ready
        with pytest.raises(BackendCommandError):
            await adapter.submit_tcp(_target(.302), 2)
        assert len(client.ik_calls) == 1
    finally:
        await cleanup(adapter, client)
        factory.assert_awaited_once()
        assert client.network_calls == client.discovery_calls == 0


@pytest.mark.parametrize('method', ['ik', 'pvat'])
async def test_stop_during_stubborn_call_enters_once_without_waiting_for_release(method):
    adapter, client, receipt, _factory = await setup(method)
    task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        try:
            await asyncio.wait_for(asyncio.shield(task), .45)
        except (TimeoutError, BackendCommandError):
            pass
        assert task.done(), 'the original stop transaction must have bounded local completion'
        assert [c[0] for c in client.write_calls].count('stop_move') == 1
        assert not client.release.is_set()
        await receipt.completion.wait()
        if method == 'pvat':
            assert adapter.read_motion_view().latched_fault == 'stop_unverified'
            assert not adapter._stop_transaction.confirmed
        assert not (await adapter.preflight()).ready
    finally:
        await cleanup(adapter, client, task)


@pytest.mark.parametrize('action', ['home', 'prepare'])
async def test_unknown_ik_while_recovery_waits_for_lock_cannot_enter_movej(monkeypatch, action):
    adapter, client, _receipt, _factory = await setup('ik')
    snapshot = adapter._snapshot
    async def cached_preflight(**kwargs):
        return snapshot
    monkeypatch.setattr(adapter, '_read_snapshot', cached_preflight)
    try:
        with pytest.raises(BackendCommandError):
            await asyncio.wait_for(getattr(adapter, action)(HOME_OPTIONS, lambda _phase: None), .45)
        assert 'movej' not in [c[0] for c in client.write_calls]
        assert adapter._motion_calls.unknown_methods == {'kinematics_inverse'}
    finally:
        await cleanup(adapter, client)


@pytest.mark.parametrize('method', ['ik', 'pvat'])
async def test_disconnect_during_first_native_call_is_bounded_and_never_accepts_late_result(method):
    adapter, client, receipt, factory = await setup(method)
    task = asyncio.create_task(adapter.disconnect())
    try:
        try:
            await asyncio.wait_for(asyncio.shield(task), .45)
        except (TimeoutError, BackendCommandError):
            pass
        assert task.done(), 'local disconnect must not await uncooperative native cancellation'
        assert adapter._client is None and not adapter._pump.running
        if method == 'pvat':
            assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
        else:
            assert client.write_calls == []
        frozen = receipt.completion.result
        client.release.set()
        await client.finished.wait()
        await asyncio.sleep(0)
        assert receipt.completion.result == frozen
        assert adapter.read_motion_view().accepted is None
        with pytest.raises(BackendCommandError, match='motion_rpc_result_unknown'):
            await adapter.connect()
        factory.assert_awaited_once()
    finally:
        await cleanup(adapter, client, task)


@pytest.mark.parametrize('method', ['ik', 'pvat'])
async def test_real_vr_tick_grip_release_can_stop_without_native_cancel_ack(method):
    client, clock, latest = StubbornClient(method), FakeClock(), LatestVRFrame()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    control = RobotControl(backend=adapter, latest=latest, clock=clock, recorder=NoopRecorder())
    await adapter.connect()
    await control.connect()
    try:
        latest.publish(_frame(1, grip=False), clock.now_ns())
        await control.tick()
        await control.arm()
        latest.publish(_frame(2, grip=True), clock.now_ns())
        await control.tick()
        latest.publish(_frame(3, grip=True, x=.01), clock.now_ns())
        await control.tick()
        await asyncio.wait_for(client.entered.wait(), .5)
        latest.publish(_frame(4, grip=False, x=.01), clock.now_ns())
        await asyncio.wait_for(control.tick(), .45)
        assert [c[0] for c in client.write_calls].count('stop_move') == 1
        assert not client.release.is_set()
        assert not adapter._sdk_lock.locked()
        record = adapter._motion_calls.snapshot()['calls']['kinematics_inverse' if method == 'ik' else 'move_pvat']
        assert record['abandon_reason'] == 'authority_revoked'
        assert record['completed_monotonic_s'] >= record['started_monotonic_s']
        if method == 'pvat':
            assert (await control.state_message()).fault == 'stop_unverified'
            assert not adapter._stop_transaction.confirmed
        with pytest.raises(RuntimeError):
            await control.arm()
        assert not any(c[0] in {'stop_sys', 'movej'} for c in client.write_calls)
    finally:
        await cleanup(adapter, client)


@pytest.mark.parametrize('method', ['kinematics_inverse', 'move_pvat'])
async def test_owned_motion_supports_native_future_return(method):
    client, clock = FakeLebaiClient(), FakeClock()
    original = getattr(client, method)
    def future(*args):
        return asyncio.ensure_future(original(*args))
    setattr(client, method, future)
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    try:
        receipt = await adapter.submit_tcp(_target(.301), 1)
        result = await asyncio.wait_for(receipt.completion.wait(), .5)
        assert result.sdk_outcome == 'returned'
        assert not adapter._motion_calls.unknown_methods
        assert len(client.ik_calls) == 1
        assert [c[0] for c in client.write_calls] == ['move_pvat']
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('method', ['kinematics_inverse', 'move_pvat'])
async def test_owned_native_child_rechecks_revocation_before_entry(monkeypatch, method):
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    original = asyncio.create_task
    def schedule(coro, *args, **kwargs):
        if kwargs.get('name') == 'owned-motion-' + method:
            adapter._pump.invalidate()
        return original(coro, *args, **kwargs)
    monkeypatch.setattr(asyncio, 'create_task', schedule)
    try:
        receipt = await adapter.submit_tcp(_target(.301), 1)
        await asyncio.wait_for(receipt.completion.wait(), .5)
        assert client.write_calls == []
        assert not adapter._motion_calls.unknown_methods
        record = adapter._motion_calls.snapshot()['calls'][method]
        assert not record['entered'] and not record['remote_result_unknown']
        assert record['abandon_reason'] == 'authority_revoked'
        if method == 'kinematics_inverse':
            assert client.ik_calls == []
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('method', ['ik', 'pvat'])
async def test_owner_cancellation_keeps_native_task_owned_and_disconnect_is_bounded(method):
    adapter, client, receipt, _factory = await setup(method)
    try:
        await asyncio.wait_for(adapter._pump.stop(), .45)
        record = adapter._motion_calls.snapshot()['calls']['kinematics_inverse' if method == 'ik' else 'move_pvat']
        assert record['abandon_reason'] == 'owner_cancelled' and record['remote_result_unknown']
        assert adapter._motion_calls.tasks
        frozen = receipt.completion.result
        try:
            await asyncio.wait_for(adapter.disconnect(), .45)
        except BackendCommandError:
            pass
        assert adapter._client is None
        assert [c[0] for c in client.write_calls].count('stop_move') == (1 if method == 'pvat' else 0)
        client.release.set()
        await client.finished.wait()
        await asyncio.sleep(0)
        assert receipt.completion.result == frozen
        assert adapter.read_motion_view().accepted is None
    finally:
        await cleanup(adapter, client)


async def test_failed_stop_obligation_does_not_transfer_to_reconnected_client():
    client, clock = FakeLebaiClient(), FakeClock()
    factory = AsyncMock(return_value=client)
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    receipt = await adapter.submit_tcp(_target(.301), 1)
    await receipt.completion.wait()
    original = client.stop_move
    async def late_stop():
        await original()
        clock.advance_ms(621)
    client.stop_move = late_stop
    with pytest.raises(BackendCommandError, match='^sdk_timeout:stop_move$'):
        await adapter.stop(StopReason.GRIP_RELEASED)
    await adapter.disconnect()
    assert not adapter._motion_write_stop_required
    await adapter.connect()
    await adapter.disconnect()
    assert [c[0] for c in client.write_calls] == ['move_pvat', 'stop_move']
    assert adapter.read_motion_view().latched_fault == 'stop_unverified'
    assert factory.await_count == 2
