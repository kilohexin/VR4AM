"""Scripted changing model feedback; no controller or stopping simulation."""
import asyncio

import pytest

from app.commissioning.native_jog_moving_guard import MovingModelJogRegionGuard
from app.commissioning.native_jog_packet import plan_jog
from app.commissioning.native_jog_stream_candidate import LoopbackJogStreamAdapter
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_codec import pose_to_lebai
from tests.commissioning.test_native_jog_moving_guard import state
from tests.commissioning.test_native_jog_region_binding import fixture, SHA
from tests.commissioning.test_native_jog_stream_candidate import StreamClient, close
from tests.robots.test_lebai_adapter_control import FakeClock


class MovingClient(StreamClient):
    def __init__(self, args, clock):
        super().__init__()
        self.args, self.clock = args, clock
        self.speed = 0.
        self.trace = []
        self.install(args['reference_q'], 0., 0.)

    def install(self, q, speed, acceleration):
        q, qd, qdd, pose = state(q, self.args['tool'], speed, acceleration)
        for prefix in ('actual', 'target'):
            for name, value in [('pose',q), ('speed',qd), ('acc',qdd)]:
                self.kin_data[prefix+'_joint_'+name] = list(value)
            self.kin_data[prefix+'_tcp_pose'] = pose_to_lebai(pose)
        self.trace.append((q, qd, qdd, tuple(pose.p)))

    async def speedl(self, acceleration, v, duration, frame):
        value = await super().speedl(acceleration, v, duration, frame)
        dt = .02
        # Stay below the exact configured boundary; do not relax the guard
        # to accommodate floating-point residuals at that boundary.
        change = max(-.8*acceleration*dt, min(.8*acceleration*dt, v['z']-self.speed))
        start, qd, qdd, _ = state(self.kin_data['actual_joint_pose'], self.args['tool'],
            self.speed, change/dt)
        q = [p+d*dt+.5*a*dt*dt for p,d,a in zip(start, qd, qdd, strict=True)]
        self.speed += change
        self.install(q, self.speed, change/dt)
        self.clock.advance_ms(20)
        return value

    async def stop_move(self):
        await super().stop_move()
        self.speed = 0.
        self.install(self.kin_data['actual_joint_pose'], 0., 0.)


async def moving_connected():
    args, _ = fixture()
    clock = FakeClock()
    c = MovingClient(args, clock)
    async def factory(host):
        assert host == '127.0.0.1'
        return c
    a = LoopbackJogStreamAdapter(args['settings'], limits=args['limits'], config_sha256=SHA,
        client_factory=factory, clock=clock.now_ns, sleep=clock.sleep, event_callback=None,
        model_region_guard=MovingModelJogRegionGuard(**args))
    await a.connect()
    return a, c, clock


def offer(a, clock, seq, sign=1):
    now = clock.now_ns()
    packet = plan_jog((0,0,sign), 1, limits=a._stream_limits, actual_p=a._snapshot.actual_tcp.p,
        snapshot_ns=a._snapshot.captured_ns, now_ns=now, generation=a._pump.generation,
        sequence=seq, input_ns=now, input_expires_ns=now+100_000_000,
        duration_s=.04, config_sha256=SHA)
    a.offer_jog(packet)


@pytest.mark.parametrize('signs', [(1,1,1), (1,-1,-1)])
async def test_changing_nonzero_feedback_continues_and_reversal_does_not_teleport(signs):
    a,c,clock = await moving_connected()
    try:
        for seq,sign in enumerate(signs,1):
            offer(a,clock,seq,sign)
            assert await a.dispatch_latest() == seq+6
        moving = c.trace[1:]
        assert any(any(v != 0 for v in row[1]) for row in moving)
        assert moving[-1][0] != c.trace[0][0] and moving[-1][3] != c.trace[0][3]
        assert [call[2]['z'] for call in c.write_calls] == [sign*.003 for sign in signs]
        await a.revoke_stream(StopReason.GRIP_RELEASED)
        assert a._stop_transaction.confirmed and a._latched_fault is None
        assert [call[0] for call in c.write_calls] == ['speedl']*3+['stop_move']
        assert a._gate.revoked and c.network_calls == c.discovery_calls == 0
    finally:
        await close(a)


async def test_nonzero_feedback_input_expiry_stops_without_another_offer():
    a,c,clock = await moving_connected()
    try:
        offer(a,clock,1)
        await a.dispatch_latest()
        clock.advance_ms(80)  # First input reaches its original 100ms deadline.
        await c.wait_for_write('stop_move', timeout=.5)
        await a.revoke_stream(StopReason.GRIP_RELEASED)
        assert a._gate.revoked and a.native_jog_entries == 1
        assert [x[0] for x in c.write_calls] == ['speedl','stop_move']
    finally:
        await close(a)


@pytest.mark.parametrize('damage', ['region','rotation','acceleration','foreign','tool'])
async def test_damaged_moving_feedback_has_no_second_native_entry(damage):
    a,c,clock = await moving_connected()
    try:
        offer(a,clock,1)
        await a.dispatch_latest()
        if damage == 'region':
            q = list(c.kin_data['actual_joint_pose']); q[2] += .0029
            c.install(q,c.speed,.01)
        elif damage == 'rotation':
            c.kin_data['actual_joint_speed'] = [0.,0.,0.,0.,0.,.00001]
        elif damage == 'acceleration':
            c.kin_data['target_joint_acc'] = [.3,0.,0.,0.,0.,0.]
        elif damage == 'foreign':
            c.running_motion = 99
        else:
            c.tcp['z'] = .18
        offer(a,clock,2)
        with pytest.raises((BackendCommandError,ValueError)):
            await a.dispatch_latest()
        assert a._gate.revoked and a.native_jog_entries == 1
        assert sum(x[0]=='stop_move' for x in c.write_calls) == 1
    finally:
        await close(a)


async def test_late_moving_reply_keeps_unknown_and_stop_unverified():
    a,c,clock = await moving_connected()
    release = asyncio.Event()
    original = c.speedl
    async def late(*args):
        while not release.is_set():
            try:
                await release.wait()
            except asyncio.CancelledError:
                pass
        return await original(*args)
    try:
        offer(a,clock,1)
        await a.dispatch_latest()
        c.speedl = late
        offer(a,clock,2)
        with pytest.raises(BackendCommandError):
            await a.dispatch_latest()
        assert a._motion_calls.unknown_methods == {'speedl'}
        assert a._latched_fault == 'stop_unverified' and a._gate.revoked
        release.set()
        await asyncio.sleep(.01)
        with pytest.raises(BackendCommandError):
            offer(a,clock,3)
        assert sum(x[0]=='stop_move' for x in c.write_calls) == 1
    finally:
        release.set()
        await close(a)


def test_single_candidate_does_not_adopt_moving_guard():
    from app.commissioning.native_jog_candidate import NativeJogCandidateAdapter
    args,_ = fixture()
    with pytest.raises(ValueError):
        NativeJogCandidateAdapter(args['settings'], limits=args['limits'], config_sha256=SHA,
            client_factory=lambda _:None, model_region_guard=MovingModelJogRegionGuard(**args),
            clock=lambda:0, sleep=asyncio.sleep, event_callback=None)


@pytest.mark.parametrize('own_packet', [False,True])
async def test_idle_label_cannot_adopt_unattributed_nonzero_rates(own_packet):
    a,c,clock = await moving_connected()
    try:
        if own_packet:
            offer(a,clock,1)
            await a.dispatch_latest()
        else:
            c.install(c.kin_data['actual_joint_pose'], .001, .004)
        c.robot_state, c.running_motion, c.motion_state = 'IDLE',None,'FINISHED'
        offer(a,clock,2)
        with pytest.raises(BackendCommandError,match='initial_motion_reported'):
            await a.dispatch_latest()
        assert a.native_jog_entries == int(own_packet)
        assert sum(x[0]=='stop_move' for x in c.write_calls) == int(own_packet)
    finally:
        await close(a)
