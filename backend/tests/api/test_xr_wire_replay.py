"""Synthetic shared frontend wire -> production receiver/control/adapter, with no network.

Transport and native SDK are in-memory substitutes. This is not headset or hardware acceptance.
"""
import asyncio
import json
from collections import deque
from contextlib import asynccontextmanager, suppress
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from app.api import teleop_ws
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.transactional_settings import PursuitSettings
from app.recording.noop import NoopRecorder
from app.robots.lebai_adapter import RealLebaiAdapter
from app.schemas.messages import RobotStateMessage, TeleopMode
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock

ROOT = Path(__file__).resolve().parents[3]


class MemoryWebSocket:
    def __init__(self, latest):
        self.app = SimpleNamespace(state=SimpleNamespace(latest=latest))
        self.incoming = asyncio.Queue()
        self.outgoing = asyncio.Queue()
        self.messages = []

    async def receive_json(self):
        # Exercise JSON decoding without using a TCP socket.
        return json.loads(await self.incoming.get())

    async def send_json(self, value):
        message = json.loads(json.dumps(value))
        self.messages.append(message)
        await self.outgoing.put(message)

    async def receive_type(self, kind, request_id=None):
        async def receive():
            while True:
                message = await self.outgoing.get()
                if message['type'] == kind and (request_id is None or message.get('request_id') == request_id):
                    return message
        return await asyncio.wait_for(receive(), 2)


@asynccontextmanager
async def rig(monkeypatch, strategy='legacy', event_callback=None):
    fixture = json.loads((ROOT / 'schemas/fixtures/xr-grip-release-wire.json').read_text())
    latest, clock = LatestVRFrame(), FakeClock()
    fake = FakeLebaiClient.idle()
    fake.ik_results = deque([list(IDLE_Q) for _ in range(20)])
    factory = AsyncMock(return_value=fake)  # Never imports or connects the native SDK.
    adapter = RealLebaiAdapter(control_settings(), client_factory=factory,
        # Exercise the real selection policy with an injected fake SDK.
        clock=clock.now_ns, sleep=clock.sleep, backend_label='LEBAI', event_callback=event_callback)
    await adapter.connect()
    pursuit = PursuitSettings(budget_ms=500, max_observation_age_ms=40,
        max_observation_gap_ms=40, position_tolerance_rad=.001,
        velocity_tolerance_radps=.002, settle_samples=3) if strategy == 'transactional' else None
    control = RobotControl(backend=adapter, latest=latest, clock=clock,
        recorder=NoopRecorder(), transactional_settings=pursuit)
    admissions = []
    original_submit = adapter.submit_tcp

    async def observe_submit(*args, **kwargs):
        # Passive receipt capture: run the unchanged production admission method.
        admission = await original_submit(*args, **kwargs)
        admissions.append(admission)
        return admission

    monkeypatch.setattr(adapter, 'submit_tcp', observe_submit)
    await control.connect()
    socket = MemoryWebSocket(latest)
    lock = asyncio.Lock()
    monkeypatch.setattr(teleop_ws, 'monotonic_ns', clock.now_ns)
    receiver = asyncio.create_task(teleop_ws._receive_messages(socket, control, asyncio.Event(), lock))
    steps = {step['name']: step for step in fixture['steps']}
    barrier = 0

    async def feed(*names):
        nonlocal barrier
        for name in names:
            clock.advance_ms(20)
            for message in steps[name]['messages']:
                await socket.incoming.put(json.dumps(message))
        barrier += 1
        request_id = f'barrier-{barrier}'
        await socket.incoming.put(json.dumps({'v': 1, 'type': 'ping', 'request_id': request_id}))
        await socket.receive_type('pong', request_id)
        assert not receiver.done()

    async def state():
        sender = asyncio.create_task(teleop_ws.state_sender(socket, control, lock))
        try:
            message = await socket.receive_type('robot_state')
            RobotStateMessage.model_validate(message)
            return message
        finally:
            sender.cancel()
            with suppress(asyncio.CancelledError):
                await sender

    try:
        yield SimpleNamespace(control=control, latest=latest, adapter=adapter, fake=fake,
            clock=clock, socket=socket, feed=feed, state=state, steps=steps, admissions=admissions)
    finally:
        receiver.cancel()
        with suppress(asyncio.CancelledError):
            await receiver
        await adapter.disconnect()
        assert fake.network_calls == fake.discovery_calls == 0
        assert not any(call[0] in {'stop_sys', 'movej'} for call in fake.write_calls)
        assert factory.await_count == 1


async def activate(r, *, pending=False):
    await r.feed('connect', 'enter', 'release')
    await r.control.tick()
    assert (await r.state())['mode'] == 'READY'
    await r.feed('arm')
    assert any(message['type'] == 'arm_ack' and message['request_id'] == 'xr-arm_request-1'
        for message in r.socket.messages)
    assert (await r.state())['mode'] == 'ARMED'
    await r.feed('active')
    await r.control.tick()
    assert (await r.state())['mode'] == 'ACTIVE'
    r.clock.advance_ms(100)
    r.fake.block_ik = pending
    await r.feed('trigger')
    await r.control.tick()
    # XR entry intentionally requests Disarm before any input is accepted.
    assert r.fake.write_calls == [('stop_move',), ('set_claw', 30, 50)]
    # command_tcp is an asynchronous admission, not an SDK send receipt.
    # Finish the already admitted pre-release candidate before testing new authority.
    assert len(r.admissions) == 1
    if pending:
        await asyncio.wait_for(r.fake.ik_started.wait(), 1)
    else:
        result = await asyncio.wait_for(r.admissions[-1].completion.wait(), 1)
        assert result.outcome == 'sdk_returned' and result.sdk_outcome == 'returned'
        assert result.command.command_id == 3
        assert [call[0] for call in r.fake.write_calls] == ['stop_move', 'set_claw', 'move_pvat']
    r.before_release = list(r.fake.write_calls)


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_shared_wire_release_coalescing_verified_stop_and_fresh_trigger(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        await r.feed('quick_release', 'throttled_press', 'regrip')
        assert r.latest.depth == 1 and r.latest.grip_release_count == 1
        assert r.latest.snapshot().frame.right.grip is True
        await r.control.tick()
        assert (await r.state())['mode'] == 'HOLD'
        assert not r.control.mapper.has_anchor
        assert r.fake.write_calls == r.before_release + [('stop_move',)]
        # Stop verification consumed synthetic time: new receipt is required for regrip.
        await r.feed('baseline')
        await r.control.tick()
        assert (await r.state())['mode'] == 'ACTIVE'
        await r.control.tick()  # Repeated receipt cannot emit a new command.
        await r.feed('trigger_release')
        await r.control.tick()
        assert r.fake.write_calls == r.before_release + [('stop_move',)]
        await r.feed('trigger_repress')
        await r.control.tick()
        assert [call for call in r.fake.write_calls[len(r.before_release):] if call[0] != 'move_pvat'] == [
            ('stop_move',), ('set_claw', 30, 40)]
        await r.feed('tracking_loss')
        await r.control.tick()
        assert (await r.state())['mode'] == 'STALE'
        # state_message returns the observation before applying completion.
        assert (await r.state())['mode'] == 'DISARMED'
        writes = list(r.fake.write_calls)
        await r.feed('tracking_recover', 'exit')
        await r.control.tick()
        assert (await r.state())['mode'] == 'DISARMED'
        assert r.fake.write_calls == writes


@pytest.mark.parametrize('fault', ['stop_unverified', 'stop_incomplete'])
@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_shared_wire_stop_failure_latches_after_late_reply_and_idle(monkeypatch, fault, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r)
        release, completed = asyncio.Event(), asyncio.Event()

        async def delayed_stop():
            r.fake.write_calls.append(('stop_move',))
            await release.wait()
            completed.set()

        if fault == 'stop_unverified':
            r.fake.stop_move = delayed_stop
        else:
            r.fake.robot_state = 'MOVING'
            r.fake.kin_data['actual_joint_speed'] = [0.1] * 6
        try:
            await r.feed('quick_release', 'regrip')
            await asyncio.wait_for(r.control.tick(), 2)
            state = await r.state()
            assert state['mode'] == 'FAULT' and state['fault'] == fault
            assert r.fake.write_calls == r.before_release + [('stop_move',)]
            release.set()
            if fault == 'stop_unverified':
                await asyncio.wait_for(completed.wait(), 1)
            r.fake.robot_state = 'IDLE'
            r.fake.kin_data['actual_joint_speed'] = [0.0] * 6
            await r.feed('baseline', 'trigger_release', 'trigger_repress')
            await r.control.tick()
            state = await r.state()
            assert state['mode'] == 'FAULT' and state['fault'] == fault
            assert not r.control.mapper.has_anchor
            assert r.fake.write_calls == r.before_release + [('stop_move',)]
            # A fresh arm request is rejected even after raw SDK IDLE.
            await r.feed('arm')
            rejection = [message for message in r.socket.messages if message['type'] == 'arm_rejected']
            assert rejection[-1]['request_id'] == 'xr-arm_request-1'
            assert r.control.mode is TeleopMode.FAULT
        finally:
            release.set()


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_release_stop_revokes_blocked_ik_before_any_pvat_entry(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r, pending=True)
        receipt = r.admissions[-1]
        assert receipt.command.command_id == 3
        await r.feed('quick_release', 'regrip')
        assert r.latest.grip_release_count == 1
        stopping = asyncio.create_task(r.control.tick())

        async def revoked():
            while r.adapter.read_motion_view().generation == receipt.command.generation:
                await asyncio.sleep(0)

        try:
            await asyncio.wait_for(revoked(), 1)
            assert r.control.mode is TeleopMode.HOLD and not r.control.mapper.has_anchor
            assert r.fake.write_calls == r.before_release
            r.fake.release_ik.set()
            await asyncio.wait_for(stopping, 2)
            result = await asyncio.wait_for(receipt.completion.wait(), 1)
            assert result.command == receipt.command
            assert result.invalidated and result.sdk_outcome == 'not_started'
            # No PVAT ever started, so the confirmed entry stop transaction is
            # reused and reverified instead of sending a duplicate stop RPC.
            assert r.fake.write_calls == r.before_release
            assert (await r.state())['mode'] == 'HOLD'
        finally:
            r.fake.release_ik.set()
            if not stopping.done():
                stopping.cancel()
            with suppress(asyncio.CancelledError):
                await stopping


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_pending_ik_accepts_newer_healthy_held_frame(monkeypatch, strategy):
    async with rig(monkeypatch, strategy) as r:
        await activate(r, pending=True)
        receipt = r.admissions[-1]
        await r.feed('baseline')
        assert r.latest.grip_release_count == 0
        r.fake.release_ik.set()
        result = await asyncio.wait_for(receipt.completion.wait(), 1)
        assert result.outcome == 'sdk_returned' and not result.invalidated
        assert result.sdk_outcome == 'returned'
        assert [call[0] for call in r.fake.write_calls] == ['stop_move', 'set_claw', 'move_pvat']


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
async def test_sdk_scheduled_task_rechecks_release_before_entry(monkeypatch, strategy):
    events, recorded = [], asyncio.Event()

    async def record(event, _timestamp):
        if event.get('kind') == 'sdk_request_lifecycle' and event.get('command_id') == 3:
            events.append(event)
            recorded.set()

    async with rig(monkeypatch, strategy, event_callback=record) as r:
        await activate(r, pending=True)
        receipt = r.admissions[-1]
        original_wait_for = asyncio.wait_for
        intercepted = asyncio.Event()

        async def revoke_before_scheduled_send(awaitable, timeout):
            if timeout == 0.06 and not intercepted.is_set():
                intercepted.set()
                # This is after the outer IK/authority check, before SDK task execution.
                released = r.latest.snapshot().frame.model_copy(deep=True)
                released.seq += 1
                released.right.grip = False
                r.latest.publish(released, r.clock.now_ns())
            return await original_wait_for(awaitable, timeout)

        monkeypatch.setattr(asyncio, 'wait_for', revoke_before_scheduled_send)
        r.fake.release_ik.set()
        result = await original_wait_for(receipt.completion.wait(), 1)
        await original_wait_for(recorded.wait(), 1)
        assert intercepted.is_set()
        assert result.invalidated and result.sdk_outcome == 'not_started'
        assert r.fake.write_calls == r.before_release
        assert r.control.mode is TeleopMode.ACTIVE  # No control tick or automatic stop.
        assert events[-1]['rpc_outcome'] == 'not_started'
        assert events[-1]['invalidated'] is True
        assert events[-1]['revocation_reason'] == 'teleop_motion_authority_revoked'


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
@pytest.mark.parametrize('revocation', ['release', 'expired', 'tracking', 'hidden', 'new_session'])
async def test_pending_ik_checks_live_vr_authority_without_waiting_for_tick(monkeypatch, strategy, revocation):
    async with rig(monkeypatch, strategy) as r:
        await activate(r, pending=True)
        receipt = r.admissions[-1]
        if revocation == 'release':
            await r.feed('quick_release', 'regrip')
            assert r.latest.grip_release_count == 1
        elif revocation == 'expired':
            r.clock.advance_ms(100)
        else:
            message = r.steps['baseline']['messages'][0]
            if revocation == 'tracking':
                message['tracking_valid'] = False
            elif revocation == 'hidden':
                message['visibility'] = 'hidden'
            else:
                message['session_id'] = 'replacement-synthetic-session'
            await r.feed('baseline')
        # Intentionally do not call control.tick: SDK dispatch has its own awaits.
        assert r.control.mode is TeleopMode.ACTIVE
        try:
            r.fake.release_ik.set()
            result = await asyncio.wait_for(receipt.completion.wait(), 1)
            assert result.invalidated and result.sdk_outcome == 'not_started'
            assert result.outcome == 'invalidated'
            assert r.fake.write_calls == r.before_release
        finally:
            r.fake.release_ik.set()
