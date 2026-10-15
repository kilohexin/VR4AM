"""Pending claw authorization using production wire/control/adapter and fake SDK only."""
import asyncio
from contextlib import suppress

import pytest

from app.schemas.messages import TeleopMode
from tests.api.test_xr_wire_replay import activate, rig


async def send_frame(r, **changes):
    message = r.latest.snapshot().frame.model_dump(mode='json')
    message['seq'] += 1
    message['right']['trigger'] = changes.pop('trigger', message['right']['trigger'])
    message['right']['grip'] = changes.pop('grip', message['right']['grip'])
    message.update(changes)
    r.steps['synthetic_claw_input'] = {'messages': [message]}
    await r.feed('synthetic_claw_input')


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
@pytest.mark.parametrize('revocation', ['release', 'expired', 'tracking', 'hidden', 'new_session'])
async def test_claw_read_cannot_send_after_vr_revocation(monkeypatch, strategy, revocation):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        before = list(r.fake.write_calls)
        sent = r.control._last_gripper_sent
        sent_ns = r.control._last_gripper_sent_ns
        read_started, finish_read = asyncio.Event(), asyncio.Event()
        original = r.fake.get_kin_data
        blocked = False

        async def blocked_read():
            nonlocal blocked
            if not blocked:
                blocked = True
                read_started.set()
                await finish_read.wait()
            return await original()

        monkeypatch.setattr(r.fake, 'get_kin_data', blocked_read)
        r.clock.advance_ms(100)
        await send_frame(r, trigger=0.8)
        tick = asyncio.create_task(r.control.tick())
        try:
            await asyncio.wait_for(read_started.wait(), 1)
            if revocation == 'release':
                await send_frame(r, grip=False)
                await send_frame(r, grip=True)
                assert r.latest.grip_release_count == 1
            elif revocation == 'expired':
                r.clock.advance_ms(100)
            elif revocation == 'tracking':
                await send_frame(r, tracking_valid=False)
            elif revocation == 'hidden':
                await send_frame(r, visibility='hidden')
            else:
                await send_frame(r, session_id='replacement-claw-session')
            finish_read.set()
            await asyncio.wait_for(tick, 1)
            assert r.fake.write_calls == before
            assert r.control._last_gripper_sent == sent
            assert r.control._last_gripper_sent_ns == sent_ns
            assert r.control._trigger_changed_since_arm  # Preserve prior successful intent.
            assert r.control.mode is TeleopMode.ACTIVE
            assert (await r.state())['fault'] is None
        finally:
            finish_read.set()
            if not tick.done():
                tick.cancel()
            with suppress(asyncio.CancelledError):
                await tick


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_direct_claw_call_reports_local_revocation_without_false_send(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        before = list(r.fake.write_calls)
        started, finish = asyncio.Event(), asyncio.Event()
        original = r.fake.get_kin_data
        reads = 0

        async def blocked_adapter_read():
            nonlocal reads
            reads += 1
            if reads == 2:  # Direct call preflight completes; then set_gripper reads.
                started.set()
                await finish.wait()
            return await original()

        monkeypatch.setattr(r.fake, 'get_kin_data', blocked_adapter_read)
        await send_frame(r, trigger=0.8)
        request = asyncio.create_task(r.control.command_gripper(0.8))
        try:
            await asyncio.wait_for(started.wait(), 1)
            await send_frame(r, grip=False)
            await send_frame(r, grip=True)
            finish.set()
            with pytest.raises(RuntimeError, match='^gripper_command_authority_changed$'):
                await asyncio.wait_for(request, 1)
            assert r.fake.write_calls == before
            assert r.control._last_gripper_sent == 0.5
            assert (await r.state())['fault'] is None
        finally:
            finish.set()
            if not request.done():
                request.cancel()
            with suppress(asyncio.CancelledError, RuntimeError):
                await request


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_armed_trigger_keeps_claw_control_with_grip_released(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await r.feed('connect', 'enter', 'release')
        await r.control.tick()
        await r.feed('arm')
        await r.control.tick()  # Released Trigger establishes the baseline.
        assert r.control.mode is TeleopMode.ARMED
        await send_frame(r, grip=False, trigger=0.5)
        await r.control.tick()
        assert r.control.mode is TeleopMode.ARMED
        assert r.fake.write_calls == [('stop_move',), ('set_claw', 30, 50)]


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_claw_sdk_task_rechecks_release_before_entry(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        before = list(r.fake.write_calls)
        sent, sent_ns = r.control._last_gripper_sent, r.control._last_gripper_sent_ns
        original_wait_for = asyncio.wait_for
        original_call = r.adapter._motion_calls.call
        intercepted = False

        async def revoke_before_entry(method, operation, timeout, **kwargs):
            nonlocal intercepted
            if method == 'set_claw' and not intercepted:
                intercepted = True
                original = operation
                async def operation():
                    await send_frame(r, grip=False)
                    await send_frame(r, grip=True)
                    return await original()
            return await original_call(method, operation, timeout, **kwargs)

        monkeypatch.setattr(r.adapter._motion_calls, 'call', revoke_before_entry)
        r.clock.advance_ms(100)
        await send_frame(r, trigger=0.8)
        await original_wait_for(r.control.tick(), 1)
        assert intercepted
        assert r.fake.write_calls == before
        assert r.control._last_gripper_sent == sent
        assert r.control._last_gripper_sent_ns == sent_ns
        assert (await r.state())['fault'] is None


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_claw_waiting_for_sdk_lock_rechecks_vr_release(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        before = list(r.fake.write_calls)
        held = asyncio.Event()
        blocked = False

        async def hold_after_snapshot(event, _timestamp):
            nonlocal blocked
            if event.get('kind') == 'robot_kinematics' and not blocked:
                blocked = True
                await r.adapter._sdk_lock.acquire()
                held.set()

        monkeypatch.setattr(r.adapter, '_event_callback', hold_after_snapshot)
        r.clock.advance_ms(100)
        await send_frame(r, trigger=0.8)
        tick = asyncio.create_task(r.control.tick())
        try:
            await asyncio.wait_for(held.wait(), 1)
            await send_frame(r, grip=False)
            await send_frame(r, grip=True)
            r.adapter._sdk_lock.release()
            await asyncio.wait_for(tick, 1)
            assert r.fake.write_calls == before
            assert r.control._last_gripper_sent == 0.5
            assert (await r.state())['fault'] is None
        finally:
            if r.adapter._sdk_lock.locked():
                r.adapter._sdk_lock.release()
            if not tick.done():
                tick.cancel()
            with suppress(asyncio.CancelledError):
                await tick


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_claw_sdk_failure_is_not_swallowed_as_authority_revocation(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        entered = asyncio.Event()

        async def failing_claw(_force, _amplitude):
            entered.set()
            raise RuntimeError('synthetic SDK failure')

        monkeypatch.setattr(r.fake, 'set_claw', failing_claw)
        r.clock.advance_ms(100)
        await send_frame(r, trigger=0.8)
        await r.control.tick()
        assert entered.is_set()
        assert (await r.state())['fault'] == 'stop_unverified'
        assert r.adapter._motion_calls.unknown_methods == {'set_claw'}
        assert r.control._last_gripper_sent == 0.5


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
@pytest.mark.parametrize('revocation', ['expired', 'tracking', 'hidden'])
async def test_revoked_first_trigger_press_does_not_authorize_later_release(monkeypatch, strategy, revocation):
    async with rig(monkeypatch, strategy, initial_claw_amplitude=60) as r:
        await r.feed('connect', 'enter', 'release')
        await r.control.tick()
        await r.feed('arm')
        await r.control.tick()
        assert r.control._last_gripper_sent == pytest.approx(0.4)
        assert not r.control._trigger_changed_since_arm
        before = list(r.fake.write_calls)
        started, finish = asyncio.Event(), asyncio.Event()
        original = r.fake.get_kin_data
        blocked = False

        async def blocked_read():
            nonlocal blocked
            if not blocked:
                blocked = True
                started.set()
                await finish.wait()
            return await original()

        monkeypatch.setattr(r.fake, 'get_kin_data', blocked_read)
        await send_frame(r, trigger=0.8)
        tick = asyncio.create_task(r.control.tick())
        try:
            await asyncio.wait_for(started.wait(), 1)
            if revocation == 'expired':
                r.clock.advance_ms(100)
            elif revocation == 'tracking':
                await send_frame(r, tracking_valid=False)
            else:
                await send_frame(r, visibility='hidden')
            finish.set()
            await asyncio.wait_for(tick, 1)
            assert r.fake.write_calls == before
            assert not r.control._trigger_changed_since_arm
            await send_frame(r, tracking_valid=True, visibility='visible', trigger=0)
            await r.control.tick()
            assert r.fake.write_calls == before
            await send_frame(r, trigger=0.8)
            await r.control.tick()
            assert r.fake.write_calls == before + [('set_claw', 30, 20)]
        finally:
            finish.set()
            if not tick.done():
                tick.cancel()
            with suppress(asyncio.CancelledError):
                await tick
