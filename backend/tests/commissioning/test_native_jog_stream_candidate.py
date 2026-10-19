"""Protocol/state ownership only; Fake does not simulate physical motion."""
import asyncio
import importlib
from dataclasses import replace

import pytest

from app.commissioning.native_jog_model_guard import ModelJogRegionGuard
from app.commissioning.native_jog_packet import plan_jog
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_codec import pose_to_lebai
from tests.commissioning.test_native_jog_region_binding import fixture, SHA
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.test_lebai_adapter_control import FakeClock


def cls():
    try:
        return importlib.import_module('app.commissioning.native_jog_stream_candidate').LoopbackJogStreamAdapter
    except ModuleNotFoundError:
        pytest.fail('offline held-input adapter missing')


class StreamClient(FakeLebaiClient):
    async def speedl(self, *values):
        self.write_calls.append(('speedl', *values))
        self.robot_state = 'MOVING'
        self.running_motion = sum(c[0] == 'speedl' for c in self.write_calls) + 6
        self.motion_state = 'RUNNING'
        return self.running_motion

    async def stop_move(self):
        await super().stop_move()
        self.robot_state, self.running_motion, self.motion_state = 'IDLE', None, 'FINISHED'


async def connected():
    args, pose = fixture()
    client, clock = StreamClient(), FakeClock()
    client.kin_data['actual_tcp_pose'] = pose_to_lebai(pose)
    client.kin_data['target_tcp_pose'] = pose_to_lebai(pose)
    async def factory(host):
        assert host == '127.0.0.1'
        return client
    options = dict(limits=args['limits'], config_sha256=SHA, client_factory=factory,
        clock=clock.now_ns, sleep=clock.sleep, event_callback=None,
        model_region_guard=ModelJogRegionGuard(**args))
    adapter = cls()(args['settings'], **options)
    await adapter.connect()
    return adapter, client, clock, options


def offer(a, clock, seq):
    now = clock.now_ns()
    p = plan_jog((0, 0, 1), 1, limits=a._stream_limits,
        actual_p=a._snapshot.actual_tcp.p, snapshot_ns=now, now_ns=now,
        generation=a._pump.generation, sequence=seq, input_ns=now,
        input_expires_ns=now+100_000_000, duration_s=.04, config_sha256=SHA)
    a.offer_jog(p)
    return p


async def close(a):
    try:
        await a.disconnect()
    except BackendCommandError:
        pass


async def test_own_moving_id_continues_and_release_uses_one_original_stop():
    a, c, clock, _ = await connected()
    try:
        for seq in range(1, 4):
            offer(a, clock, seq)
            assert await a.dispatch_latest() == seq+6
            clock.advance_ms(10)
        assert [x[0] for x in c.write_calls] == ['speedl']*3
        await a.revoke_stream(StopReason.GRIP_RELEASED)
        assert a._stop_transaction.confirmed and a._latched_fault is None
        assert [x[0] for x in c.write_calls] == ['speedl']*3+['stop_move']
        with pytest.raises(BackendCommandError):
            offer(a, clock, 4)
        assert c.network_calls == c.discovery_calls == 0
    finally:
        await close(a)
    assert a._watchdog.done()


async def test_latest_input_only_and_no_implicit_heartbeat():
    a, c, clock, _ = await connected()
    try:
        for seq in (1, 2, 3):
            offer(a, clock, seq)
        await a.dispatch_latest()
        assert await a.dispatch_latest() is None
        assert len(c.write_calls) == 1
        assert a._gate.entries == 1
    finally:
        await close(a)


@pytest.mark.parametrize('damage', ['foreign', 'paused', 'estop', 'tool', 'joint', 'region', 'rate'])
async def test_continuation_never_relabels_foreign_or_unsafe_feedback(damage):
    a, c, clock, _ = await connected()
    try:
        offer(a, clock, 1)
        await a.dispatch_latest()
        if damage == 'foreign':
            c.running_motion = 999
        elif damage == 'paused':
            c.robot_state = 'PAUSED'
        elif damage == 'estop':
            c.estop_reason = 3
        elif damage == 'tool':
            c.tcp = {**c.tcp, 'z': .18}
        elif damage == 'joint':
            c.kin_data['actual_joint_pose'][0] = 4.
        elif damage == 'region':
            c.kin_data['actual_joint_pose'][0] += .001
        else:
            c.kin_data['actual_joint_speed'][0] = .001
        offer(a, clock, 2)
        with pytest.raises((BackendCommandError, ValueError)):
            await a.dispatch_latest()
        assert sum(x[0] == 'speedl' for x in c.write_calls) == 1
        assert sum(x[0] == 'stop_move' for x in c.write_calls) == 1
    finally:
        await close(a)


async def test_expired_input_stops_even_without_further_api_calls():
    a, c, clock, _ = await connected()
    try:
        offer(a, clock, 1)
        await a.dispatch_latest()
        clock.advance_ms(100)
        await c.wait_for_write('stop_move', timeout=.5)
        assert [x[0] for x in c.write_calls] == ['speedl', 'stop_move']
        assert a._gate.revoked
    finally:
        await close(a)


async def test_waiting_lock_replacement_writes_only_new_packet():
    a, c, clock, _ = await connected()
    try:
        offer(a, clock, 1)
        # Keep fresh snapshot acquisition real; pause exactly after it.
        old_read = a._read_snapshot
        ready, resume = asyncio.Event(), asyncio.Event()
        async def read(**kw):
            s = await old_read(**kw)
            if kw.get('read_purpose') == 'motion':
                ready.set()
                await resume.wait()
            return s
        a._read_snapshot = read
        task = asyncio.create_task(a.dispatch_latest())
        await ready.wait()
        offer(a, clock, 2)
        resume.set()
        assert await task is None
        assert c.write_calls == []
        assert await a.dispatch_latest() == 7
        assert len(c.write_calls) == 1
    finally:
        await close(a)


async def test_new_input_during_native_await_does_not_revoke_previous_result():
    a, c, clock, _ = await connected()
    started, release = asyncio.Event(), asyncio.Event()
    original = c.speedl
    async def pending(*args):
        started.set()
        await release.wait()
        return await original(*args)
    c.speedl = pending
    try:
        offer(a, clock, 1)
        task = asyncio.create_task(a.dispatch_latest())
        await started.wait()
        offer(a, clock, 2)
        assert await a.dispatch_latest() is None
        release.set()
        assert await task == 7
        assert await a.dispatch_latest() == 8
    finally:
        release.set()
        await close(a)


async def test_tenth_result_is_known_then_session_stops():
    a, c, clock, _ = await connected()
    try:
        for seq in range(1, 11):
            offer(a, clock, seq)
            assert await a.dispatch_latest() == seq+6
        assert a._gate.entries == 10 and not a._motion_calls.unknown_methods
        assert a._gate.revoked and c.write_calls[-1] == ('stop_move',)
    finally:
        await close(a)


async def test_cancellation_resistant_late_native_result_never_reopens_stream():
    a, c, clock, _ = await connected()
    started, release = asyncio.Event(), asyncio.Event()
    original = c.speedl
    async def pending(*args):
        started.set()
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                pass
        return await original(*args)
    c.speedl = pending
    try:
        offer(a, clock, 1)
        task = asyncio.create_task(a.dispatch_latest())
        await started.wait()
        with pytest.raises(BackendCommandError):
            await task
        assert a._latched_fault == 'stop_unverified'
        release.set()
        await asyncio.sleep(.01)
        with pytest.raises(BackendCommandError):
            offer(a, clock, 2)
        assert a._gate.revoked
        assert sum(x[0] == 'stop_move' for x in c.write_calls) == 1
    finally:
        release.set()
        await close(a)


@pytest.mark.parametrize('damage', ['ip', 'factory', 'label', 'model'])
async def test_constructor_refuses_nonloopback_or_unbound_inputs(damage):
    a, c, _, options = await connected()
    try:
        settings = a.settings
        if damage == 'ip':
            settings = replace(settings, ip='192.168.10.20')
        elif damage == 'factory':
            options['client_factory'] = None
        elif damage == 'label':
            options['backend_label'] = 'LEBAI'
        else:
            options['model_region_guard'] = None
        with pytest.raises(ValueError):
            cls()(settings, **options)
        assert c.write_calls == []
    finally:
        await close(a)


@pytest.mark.parametrize('first_written', [False, True])
async def test_revocation_at_owned_native_task_has_no_following_speedl(monkeypatch, first_written):
    a, c, clock, _ = await connected()
    try:
        if first_written:
            offer(a, clock, 1)
            await a.dispatch_latest()
        offer(a, clock, 2)
        create = asyncio.create_task
        def schedule(coro, **kwargs):
            if kwargs.get('name') == 'owned-motion-speedl':
                a._request_stop(StopReason.GRIP_RELEASED)
            return create(coro, **kwargs)
        monkeypatch.setattr(asyncio, 'create_task', schedule)
        with pytest.raises(BackendCommandError):
            await a.dispatch_latest()
        assert sum(x[0] == 'speedl' for x in c.write_calls) == int(first_written)
        assert sum(x[0] == 'stop_move' for x in c.write_calls) == int(first_written)
        assert not a._motion_calls.unknown_methods
    finally:
        await close(a)


@pytest.mark.parametrize('result', [True, 0, '7', 7])
async def test_invalid_or_reused_motion_id_is_permanently_unknown(result):
    a, c, clock, _ = await connected()
    try:
        if result == 7 and type(result) is int:
            offer(a, clock, 1)
            await a.dispatch_latest()
        async def invalid(*args):
            c.write_calls.append(('speedl', *args))
            return result
        c.speedl = invalid
        offer(a, clock, 2)
        with pytest.raises(BackendCommandError):
            await a.dispatch_latest()
        assert a._gate.revoked and a._latched_fault == 'stop_unverified'
        assert a._motion_calls.unknown_methods == {'speedl'}
        assert sum(x[0] == 'stop_move' for x in c.write_calls) == 1
    finally:
        await close(a)


async def test_owned_motion_state_timeout_cannot_install_late_read_or_continue():
    a, c, clock, _ = await connected()
    started, release = asyncio.Event(), asyncio.Event()
    try:
        offer(a, clock, 1)
        await a.dispatch_latest()
        async def pending(_id):
            started.set()
            while not release.is_set():
                try:
                    await release.wait()
                except asyncio.CancelledError:
                    pass
            return 'RUNNING'
        c.get_motion_state = pending
        offer(a, clock, 2)
        with pytest.raises(BackendCommandError):
            await a.dispatch_latest()
        assert started.is_set() and a._snapshot_calls.unknown_methods
        assert a._latched_fault == 'stop_unverified'
        release.set()
        await asyncio.sleep(.01)
        assert a._gate.revoked and a.native_jog_entries == 1
    finally:
        release.set()
        await close(a)


async def test_in_place_configuration_change_stops_without_new_packet():
    a, c, clock, _ = await connected()
    try:
        offer(a, clock, 1)
        await a.dispatch_latest()
        object.__setattr__(a.settings.control, 'max_tcp_speed_mps', .1)
        with pytest.raises(BackendCommandError):
            offer(a, clock, 2)
        await a.revoke_stream(StopReason.GRIP_RELEASED)
        assert a.native_jog_entries == 1
        assert sum(x[0] == 'stop_move' for x in c.write_calls) == 1
    finally:
        await close(a)


async def test_first_packet_does_not_adopt_already_moving_robot():
    a, c, clock, _ = await connected()
    try:
        c.robot_state, c.running_motion, c.motion_state = 'MOVING', 77, 'RUNNING'
        offer(a, clock, 1)
        with pytest.raises(BackendCommandError):
            await a.dispatch_latest()
        assert c.write_calls == []
    finally:
        await close(a)


async def test_input_gap_cannot_reopen_stream_ahead_of_delayed_watchdog():
    a, c, clock, _ = await connected()
    try:
        offer(a, clock, 1)
        await a.dispatch_latest()
        clock.advance_ms(100)
        # No event-loop yield: the newly arriving API call beats the watchdog.
        with pytest.raises(BackendCommandError):
            offer(a, clock, 2)
        await a.revoke_stream(StopReason.GRIP_RELEASED)
        assert a.native_jog_entries == 1 and a._gate.revoked
    finally:
        await close(a)
