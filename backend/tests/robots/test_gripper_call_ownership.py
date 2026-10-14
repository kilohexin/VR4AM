"""Explicit fake SDK writes only; no native imports or robot network."""
import asyncio
from unittest.mock import AsyncMock

import pytest

from app.robots.base import BackendCommandError, GripperCommandRevoked, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock, _frame, _target, HOME_OPTIONS


class StubbornClaw(FakeLebaiClient):
    def __init__(self):
        super().__init__()
        self.entered, self.release, self.finished = asyncio.Event(), asyncio.Event(), asyncio.Event()
        self.fail_late = False

    async def set_claw(self, force, amplitude):
        await super().set_claw(force, amplitude)
        self.entered.set()
        try:
            while not self.release.is_set():
                try:
                    await self.release.wait()
                except asyncio.CancelledError:
                    pass  # Model native cancellation without acknowledgment.
        finally:
            self.finished.set()
        if self.fail_late:
            raise RuntimeError('late_fake_claw_failure')


async def setup():
    client, clock = StubbornClaw(), FakeClock()
    factory = AsyncMock(return_value=client)
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    task = asyncio.create_task(adapter.set_gripper(.5))
    await asyncio.wait_for(client.entered.wait(), .5)
    return adapter, client, task, factory


async def cleanup(adapter, client, *tasks):
    client.release.set()
    if client.entered.is_set():
        await asyncio.wait_for(client.finished.wait(), .5)
    await asyncio.gather(*tasks, return_exceptions=True)
    try:
        await adapter.disconnect()
    except BackendCommandError:
        pass


@pytest.mark.parametrize('late_error', [False, True])
async def test_claw_timeout_exits_without_native_cancel_ack_and_never_heals(late_error):
    adapter, client, task, factory = await setup()
    client.fail_late = late_error
    try:
        done, _ = await asyncio.wait({task}, timeout=.35)
        assert done, '200 ms local claw budget must not await native cancellation acknowledgment'
        with pytest.raises(BackendCommandError, match='^sdk_timeout:set_claw$'):
            task.result()
        assert not adapter._sdk_lock.locked()
        assert not (await adapter.preflight()).ready
        assert adapter._motion_calls.unknown_methods == {'set_claw'}
        for operation in [lambda: adapter.set_gripper(.2), lambda: adapter.submit_tcp(_target(), 2),
                lambda: adapter.home(HOME_OPTIONS, lambda _phase: None),
                lambda: adapter.prepare(HOME_OPTIONS, lambda _phase: None)]:
            with pytest.raises(BackendCommandError):
                await operation()
        frozen = adapter._motion_calls.snapshot()['calls']['set_claw']
        client.release.set()
        await asyncio.wait_for(client.finished.wait(), .5)
        await asyncio.sleep(0)
        assert adapter._motion_calls.snapshot()['calls']['set_claw'] == frozen
        assert not (await adapter.preflight()).ready
        await adapter.disconnect()
        with pytest.raises(BackendCommandError, match='motion_rpc_result_unknown'):
            await adapter.connect()
        factory.assert_awaited_once()
        assert [c[0] for c in client.write_calls] == ['set_claw']
    finally:
        await cleanup(adapter, client, task)


async def test_stop_can_enter_while_claw_ignores_cancel_and_cannot_confirm_claw():
    adapter, client, claw, _factory = await setup()
    stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        done, _ = await asyncio.wait({stop}, timeout=.45)
        assert done, 'Grip release stop must not await claw cancellation acknowledgment'
        with pytest.raises(BackendCommandError, match='^stop_unverified$'):
            stop.result()
        assert not client.release.is_set()
        assert not adapter._sdk_lock.locked()
        assert [c[0] for c in client.write_calls] == ['set_claw', 'stop_move']
        assert adapter._motion_calls.unknown_methods == {'set_claw'}
        assert adapter.read_motion_view().latched_fault == 'stop_unverified'
        assert not adapter._stop_transaction.confirmed
    finally:
        await cleanup(adapter, client, claw, stop)


async def test_caller_cancel_releases_lock_without_native_ack():
    adapter, client, claw, _factory = await setup()
    try:
        claw.cancel()
        done, _ = await asyncio.wait({claw}, timeout=.1)
        assert done, 'caller cancellation must not await native acknowledgment'
        with pytest.raises(asyncio.CancelledError):
            claw.result()
        assert not adapter._sdk_lock.locked()
        assert not (await adapter.preflight()).ready
        assert adapter._motion_calls.unknown_methods == {'set_claw'}
        assert client.write_calls == [('set_claw', 30, 50)]
    finally:
        await cleanup(adapter, client, claw)


async def test_disconnect_closes_claw_owner_and_blocks_replacement_factory():
    adapter, client, claw, factory = await setup()
    task = asyncio.create_task(adapter.disconnect())
    try:
        done, _ = await asyncio.wait({task}, timeout=.25)
        assert done
        task.result()
        assert adapter._client is None
        with pytest.raises(BackendCommandError):
            await adapter.connect()
        factory.assert_awaited_once()
        done, _ = await asyncio.wait({claw}, timeout=.1)
        assert done
        assert not adapter._sdk_lock.locked(), 'local disconnect must revoke the claw wait'
        assert adapter._motion_calls.unknown_methods == {'set_claw'}
        assert client.write_calls == [('set_claw', 30, 50)]
    finally:
        await cleanup(adapter, client, claw, task)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), float('-inf')])
async def test_nonfinite_claw_input_never_becomes_an_endpoint(value):
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    try:
        with pytest.raises(BackendCommandError, match='^invalid_gripper_command$'):
            await adapter.set_gripper(value)
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_gripper_revocation_at_owned_child_entry_is_not_sdk_timeout(monkeypatch):
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    authority = True
    adapter.bind_gripper_authority(lambda: authority)
    original = adapter._motion_calls.call
    async def revoke(method, *args, **kwargs):
        nonlocal authority
        if method == 'set_claw':
            authority = False
        return await original(method, *args, **kwargs)
    monkeypatch.setattr(adapter._motion_calls, 'call', revoke)
    try:
        with pytest.raises(GripperCommandRevoked):
            await adapter.set_gripper(.5)
        assert client.write_calls == []
        assert not adapter._motion_calls.unknown_methods
        assert adapter._latched_fault is None
        assert adapter._gripper_active == 0
    finally:
        await adapter.disconnect()


async def test_claw_supports_sdk_future_return(monkeypatch):
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    original = client.set_claw
    monkeypatch.setattr(client, 'set_claw', lambda *args: asyncio.ensure_future(original(*args)))
    await adapter.connect()
    try:
        await adapter.set_gripper(.25)
        assert client.write_calls == [('set_claw', 30, 75)]
        assert not adapter._motion_calls.unknown_methods
        record = adapter._motion_calls.snapshot()['calls']['set_claw']
        assert record['entered'] and record['outcome'] == 'returned'
        assert not record['remote_result_unknown']
    finally:
        await adapter.disconnect()


async def test_cancel_before_claw_sdk_entry_is_known_no_write(monkeypatch):
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    snapshot, read = adapter._snapshot, asyncio.Event()
    async def preflight(**kwargs):
        read.set()
        return snapshot
    monkeypatch.setattr(adapter, '_read_snapshot', preflight)
    await adapter._sdk_lock.acquire()
    task = asyncio.create_task(adapter.set_gripper(.5))
    try:
        await asyncio.wait_for(read.wait(), .5)
        task.cancel()
        done, _ = await asyncio.wait({task}, timeout=.1)
        assert done
        with pytest.raises(asyncio.CancelledError):
            task.result()
        assert adapter._gripper_active == 0
        assert not adapter._motion_calls.unknown_methods and adapter._latched_fault is None
        assert client.write_calls == []
    finally:
        adapter._sdk_lock.release()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


async def test_reconnect_waits_for_whole_old_claw_pipeline(monkeypatch):
    client, replacement, clock = FakeLebaiClient(), FakeLebaiClient(), FakeClock()
    factory = AsyncMock(side_effect=[client, replacement])
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    snapshot, read = adapter._snapshot, asyncio.Event()
    async def preflight(**kwargs):
        read.set()
        return snapshot
    monkeypatch.setattr(adapter, '_read_snapshot', preflight)
    await adapter._sdk_lock.acquire()
    task = asyncio.create_task(adapter.set_gripper(.5))
    held = True
    try:
        await asyncio.wait_for(read.wait(), .5)
        await adapter.disconnect()
        with pytest.raises(BackendCommandError, match='^gripper_request_pending$'):
            await adapter.connect()
        factory.assert_awaited_once()
        adapter._sdk_lock.release()
        held = False
        done, _ = await asyncio.wait({task}, timeout=.1)
        assert done
        with pytest.raises(BackendCommandError):
            task.result()
        assert adapter._gripper_active == 0
        await adapter.connect()
        assert adapter._client is replacement
        assert client.write_calls == replacement.write_calls == []
    finally:
        if held:
            adapter._sdk_lock.release()
        await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.parametrize('active', [False, True])
async def test_real_control_trigger_wait_cannot_block_grip_release(active):
    client, clock, latest = StubbornClaw(), FakeClock(), LatestVRFrame()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    control = RobotControl(backend=adapter, latest=latest, clock=clock, recorder=NoopRecorder())
    await adapter.connect()
    await control.connect()
    tick = None
    try:
        latest.publish(_frame(1, grip=False), clock.now_ns())
        await control.tick()
        await control.arm()
        latest.publish(_frame(2, grip=active), clock.now_ns())
        await control.tick()
        trigger = _frame(3, grip=True)
        trigger = trigger.model_copy(update={'right': trigger.right.model_copy(update={'trigger': .5})})
        latest.publish(trigger, clock.now_ns())
        tick = asyncio.create_task(control.tick())
        await asyncio.wait_for(client.entered.wait(), .5)
        latest.publish(_frame(4, grip=False), clock.now_ns())
        done, _ = await asyncio.wait({tick}, timeout=.45)
        assert done, 'VR tick must escape a claw SDK that ignores cancellation'
        tick.result()
        assert not client.release.is_set() and not adapter._sdk_lock.locked()
        assert [c[0] for c in client.write_calls].count('set_claw') == 1
        assert [c[0] for c in client.write_calls].count('stop_move') == 1
        assert control._last_gripper_sent != .5
        assert adapter.read_motion_view().latched_fault == 'stop_unverified'
        assert not adapter._stop_transaction.confirmed
        assert not control.mapper.has_anchor
        with pytest.raises(RuntimeError):
            await control.arm()
        assert not any(c[0] in {'stop_sys', 'movej'} for c in client.write_calls)
    finally:
        await cleanup(adapter, client, *([tick] if tick is not None else []))


async def test_unknown_claw_revokes_previously_confirmed_stop_without_new_rpc():
    client, clock = StubbornClaw(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    claw = None
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        transaction = adapter._stop_transaction
        assert transaction.confirmed
        assert (await adapter.preflight()).ready
        claw = asyncio.create_task(adapter.set_gripper(.5))
        await asyncio.wait_for(client.entered.wait(), .5)
        claw.cancel()
        done, _ = await asyncio.wait({claw}, timeout=.1)
        assert done
        with pytest.raises(asyncio.CancelledError):
            claw.result()
        assert not transaction.confirmed, 'claw unknown must revoke a prior confirmed bit'
        with pytest.raises(BackendCommandError, match='^stop_unverified$'):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert adapter._stop_transaction is transaction
        assert not transaction.confirmed
        assert [c[0] for c in client.write_calls] == ['stop_move', 'set_claw']
    finally:
        await cleanup(adapter, client, *([claw] if claw is not None else []))


@pytest.mark.parametrize('error', [RuntimeError, TimeoutError])
async def test_entered_claw_error_stays_unknown_and_is_not_retried(monkeypatch, error):
    client, clock = FakeLebaiClient(), FakeClock()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    original = client.set_claw
    async def fail(*args):
        await original(*args)
        raise error('fake_claw_rpc_error')
    monkeypatch.setattr(client, 'set_claw', fail)
    await adapter.connect()
    try:
        expected = 'sdk_timeout:set_claw' if error is TimeoutError else 'sdk_call_failed:set_claw'
        with pytest.raises(BackendCommandError, match='^' + expected + '$'):
            await adapter.set_gripper(.5)
        assert adapter._motion_calls.unknown_methods == {'set_claw'}
        assert adapter._latched_fault == 'stop_unverified'
        assert not adapter._sdk_lock.locked()
        assert not (await adapter.preflight()).ready
        with pytest.raises(BackendCommandError, match='motion_rpc_result_unknown'):
            await adapter.set_gripper(.2)
        assert client.write_calls == [('set_claw', 30, 50)]
    finally:
        await adapter.disconnect()


async def test_unknown_claw_keeps_existing_arm_stop_obligation():
    client, clock = StubbornClaw(), FakeClock()
    q = list(client.kin_data['actual_joint_pose'])
    q[0] += .005
    client.ik_results.clear()
    client.ik_results.append(q)
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, pvat_clock=clock.now_ns, sleep=clock.sleep)
    await adapter.connect()
    claw = None
    try:
        receipt = await adapter.submit_tcp(_target(.301), 1)
        await asyncio.wait_for(receipt.completion.wait(), .5)
        assert [c[0] for c in client.write_calls] == ['move_pvat']
        assert adapter._motion_accepted and adapter._motion_write_stop_required
        claw = asyncio.create_task(adapter.set_gripper(.5))
        await asyncio.wait_for(client.entered.wait(), .5)
        claw.cancel()
        done, _ = await asyncio.wait({claw}, timeout=.1)
        assert done
        with pytest.raises(asyncio.CancelledError):
            claw.result()
        assert adapter._motion_accepted and adapter._motion_write_stop_required
        with pytest.raises(BackendCommandError, match='^stop_unverified$'):
            await adapter.disconnect()
        assert adapter._client is None
        assert [c[0] for c in client.write_calls] == ['move_pvat', 'set_claw', 'stop_move']
    finally:
        await cleanup(adapter, client, *([claw] if claw is not None else []))
