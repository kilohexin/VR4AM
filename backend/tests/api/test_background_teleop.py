"""Production background tasks with real local timers; memory wire and fake SDK.

No socket, native SDK, site configuration, headset, robot or physics validation.
"""
import asyncio
import json
import time
from collections import deque
from contextlib import asynccontextmanager, suppress
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from fastapi import WebSocketDisconnect

from app.api.teleop_ws import teleop_websocket
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.transactional_settings import PursuitSettings
from app.diagnostics.store import DiagnosticsStore
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.base import BackendCommandError
from app.schemas.messages import RobotStateMessage
from app.timebase import MonotonicClock
from tests.control.test_robot_control import RecordingRecorder
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q
from tests.robots.real_settings import control_settings


class LoopClient(FakeLebaiClient):
    def __init__(self, timeline):
        super().__init__()
        self.timeline = timeline
        self.ik_results = deque([list(IDLE_Q) for _ in range(1000)])
        self.stop_mode = 'normal'
        self.release_stop = asyncio.Event()
        self.stop_started = asyncio.Event()
        self.stop_returned = asyncio.Event()
        self.entries = []

    async def move_pvat(self, p, v, a, t):
        self.timeline.append(('sdk', 'move_pvat'))
        self.entries.append((time.monotonic_ns(), 'move_pvat'))
        return await super().move_pvat(p, v, a, t)

    async def set_claw(self, force, amplitude):
        self.timeline.append(('sdk', 'set_claw'))
        self.entries.append((time.monotonic_ns(), 'set_claw'))
        await super().set_claw(force, amplitude)

    async def stop_move(self):
        self.timeline.append(('sdk', 'stop_move'))
        self.entries.append((time.monotonic_ns(), 'stop_move'))
        await super().stop_move()
        self.stop_started.set()
        if self.stop_mode == 'late':
            await self.release_stop.wait()
        elif self.stop_mode == 'moving':
            self.robot_state = 'MOVING'
            self.kin_data['actual_joint_speed'] = [0.1] * 6
        self.stop_returned.set()


class MemoryWire:
    def __init__(self, state):
        self.app = SimpleNamespace(state=state)
        self.incoming = asyncio.Queue()
        self.messages = []
        self.changed = asyncio.Condition()
        self.accepted = asyncio.Event()

    async def accept(self):
        self.accepted.set()

    async def receive_json(self):
        value = await self.incoming.get()
        if value is None:
            raise WebSocketDisconnect(1000)
        if isinstance(value, BaseException):
            raise value  # Test-only transport failure injection.
        return json.loads(value)

    async def send_json(self, value):
        message = json.loads(json.dumps(value))
        if message.get('type') == 'robot_state':
            RobotStateMessage.model_validate(message)
        async with self.changed:
            self.messages.append(message)
            self.changed.notify_all()

    async def close(self, **_kwargs):
        self.disconnect()

    def send(self, message):
        self.incoming.put_nowait(json.dumps(message))

    def disconnect(self):
        self.incoming.put_nowait(None)

    async def wait(self, predicate, *, after=0, timeout=3):
        async def matching():
            async with self.changed:
                while True:
                    for index in range(after, len(self.messages)):
                        if predicate(self.messages[index]):
                            return index, self.messages[index]
                    await self.changed.wait()
        return await asyncio.wait_for(matching(), timeout)

    async def mode(self, mode, *, after=0):
        return await self.wait(lambda message: message.get('type') == 'robot_state'
                               and message['mode'] == mode, after=after)


class Frames:
    def __init__(self, wire):
        self.wire, self.seq = wire, 0
        self.grip, self.trigger = False, 0.0
        self.task = None

    def emit(self, *, grip=None, trigger=None):
        self.seq += 1
        self.wire.send({'v': 1, 'type': 'vr_frame', 'session_id': 'background-synthetic',
            'seq': self.seq, 'client_mono_ms': time.monotonic_ns() / 1e6,
            'tracking_valid': True, 'visibility': 'visible', 'right': {
                'p': [0.4, 0.5, 0.6], 'q': [0, 0, 0, 1],
                'grip': self.grip if grip is None else grip,
                'trigger': self.trigger if trigger is None else trigger}})

    async def run(self):
        while True:
            self.emit()
            await asyncio.sleep(1 / 90)  # Requested cadence, not a realtime guarantee.

    async def stop(self):
        if self.task is not None:
            self.task.cancel()
            with suppress(asyncio.CancelledError):
                await self.task
            self.task = None

    def release_and_regrip(self):
        # Queue both on one event-loop turn, retaining the accepted release edge.
        self.trigger, self.grip = 1.0, True
        self.emit(grip=False)
        self.emit(grip=True)


async def wait_until(predicate, timeout=3):
    async def ready():
        while not predicate():
            await asyncio.sleep(0.005)
    await asyncio.wait_for(ready(), timeout)


class TimelineRecorder(RecordingRecorder):
    def __init__(self, timeline):
        super().__init__()
        self.timeline = timeline

    async def write_critical_event(self, kind, payload, server_mono_ns):
        self.timeline.append(('event', kind))
        await super().write_critical_event(kind, payload, server_mono_ns)


async def finish_cleanup(awaitable, control):
    try:
        await asyncio.wait_for(awaitable, 3)
    except BackendCommandError as error:
        # Production disconnect/shutdown reuses the failed stop transaction.
        # Accept only those two known failures; do not hide unrelated exceptions.
        assert control._fault in {'stop_unverified', 'stop_incomplete'}
        assert str(error) in {'sdk_timeout:stop_move', 'stop_incomplete'}


@asynccontextmanager
async def background(strategy):
    timeline = []
    fake, clock, latest = LoopClient(timeline), MonotonicClock(), LatestVRFrame()
    recorder, sdk_events = TimelineRecorder(timeline), []

    async def record_sdk(event, _timestamp):
        sdk_events.append(event)

    factory = AsyncMock(return_value=fake)
    settings = control_settings()  # Invented test settings, never a local config file.
    adapter = RealLebaiAdapter(settings, client_factory=factory, event_callback=record_sdk)
    await adapter.connect()
    pursuit = PursuitSettings(budget_ms=500, max_observation_age_ms=40,
        max_observation_gap_ms=40, position_tolerance_rad=.001,
        velocity_tolerance_radps=.002, settle_samples=3) if strategy == 'transactional' else None
    control = RobotControl(backend=adapter, latest=latest, clock=clock,
        recorder=recorder, transactional_settings=pursuit)
    await control.connect()
    state = SimpleNamespace(control=control, latest=latest, teleop_owner=None,
        teleop_owner_lock=asyncio.Lock(), teleop_sender_tasks=set(),
        diagnostics=DiagnosticsStore(), runtime_backend='LEBAI',
        settings=SimpleNamespace(backend='lebai', lebai=settings))
    wire, frames = MemoryWire(state), None
    frames = Frames(wire)
    session = asyncio.create_task(teleop_websocket(wire))
    await control.start()  # Real run(), real asyncio sleeps, no overridden tick or clock.
    try:
        await asyncio.wait_for(wire.accepted.wait(), 1)
        wire.send({'v': 1, 'type': 'hello', 'request_id': 'background-hello'})
        frames.task = asyncio.create_task(frames.run())
        await wait_until(lambda: control.machine.can_arm)
        wire.send({'v': 1, 'type': 'arm_request', 'request_id': 'background-arm'})
        await wire.wait(lambda message: message.get('type') == 'arm_ack')
        await wire.mode('ARMED')
        frames.grip = True
        await wire.mode('ACTIVE')
        frames.trigger = 0.5
        await fake.wait_for_write('set_claw', timeout=2)
        await fake.wait_for_write('move_pvat', timeout=2)
        yield SimpleNamespace(control=control, adapter=adapter, fake=fake, state=state,
            wire=wire, frames=frames, session=session, recorder=recorder, sdk_events=sdk_events,
            timeline=timeline)
    finally:
        cleanup_errors = []
        try:
            await frames.stop()
        except BaseException as error:
            cleanup_errors.append(('frames', error))
        fake.release_stop.set()
        if not session.done():
            wire.disconnect()
        for name, operation in [('session', session), ('control', control.stop()),
                                ('adapter', adapter.disconnect())]:
            try:
                if name == 'adapter':
                    await asyncio.wait_for(operation, 3)
                else:
                    await finish_cleanup(operation, control)
            except BaseException as error:
                cleanup_errors.append((name, error))
        if cleanup_errors:
            primary = cleanup_errors[0][1]
            for name, error in cleanup_errors[1:]:
                primary.add_note(f'{name} cleanup also raised {type(error).__name__}')
            raise primary
        assert not state.teleop_sender_tasks and state.teleop_owner is None
        assert fake.network_calls == fake.discovery_calls == 0
        assert factory.await_count == 1
        assert not any(call[0] in {'stop_sys', 'movej'} for call in fake.write_calls)


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
@pytest.mark.parametrize('cause', ['release', 'stale', 'disconnect'])
async def test_background_stop_takeover_and_session_cleanup(strategy, cause):
    async with background(strategy) as r:
        boundary = len(r.wire.messages)
        before = len(r.fake.write_calls)
        if cause == 'release':
            r.frames.release_and_regrip()
            hold, _ = await r.wire.mode('HOLD', after=boundary)
            await r.wire.mode('ACTIVE', after=hold + 1)
            assert r.state.latest.grip_release_count == 1
            # Regrip-held Trigger creates a new baseline, not old claw intent.
            assert [call for call in r.fake.write_calls[before:] if call[0] == 'set_claw'] == []
            r.frames.trigger = 0.0
            await wait_until(lambda: r.control._armed_trigger_baseline == 0.0)
            r.frames.trigger = 0.6
            await wait_until(lambda: ('set_claw', 30, 40) in r.fake.write_calls)
        elif cause == 'stale':
            await r.frames.stop()
            await r.wire.mode('DISARMED', after=boundary)
            assert [call for call in r.fake.write_calls[before:] if call[0] == 'stop_move'] == [('stop_move',)]
        else:
            await r.frames.stop()
            r.wire.disconnect()
            await asyncio.wait_for(r.session, 3)
            assert r.state.teleop_owner is None and not r.state.teleop_sender_tasks
            assert r.state.latest.depth == 0
            assert [call for call in r.fake.write_calls[before:] if call[0] == 'stop_move'] == [('stop_move',)]
        assert r.control._task is not None and not r.control._task.done()
        assert r.control._fault is None
        # Equal coarse monotonic timestamps cannot establish event ordering.
        # These synchronous appends share one event loop and preserve local order.
        requested = r.timeline.index(('event', 'stop_requested'))
        confirmed = r.timeline.index(('event', 'stop_confirmed'), requested + 1)
        assert not any(kind == 'sdk' and method in {'move_pvat', 'set_claw'}
                       for kind, method in r.timeline[requested + 1:confirmed])


@pytest.mark.parametrize('strategy', ['legacy', 'transactional'])
@pytest.mark.parametrize('reply, fault', [('late', 'stop_unverified'), ('moving', 'stop_incomplete')])
async def test_background_stop_fault_latches_through_reply_idle_and_disconnect(strategy, reply, fault):
    async with background(strategy) as r:
        r.fake.stop_mode = reply
        before, boundary = len(r.fake.write_calls), len(r.wire.messages)
        r.frames.release_and_regrip()
        _, message = await r.wire.mode('FAULT', after=boundary)
        assert message['fault'] == fault
        assert r.fake.write_calls[before:] == [('stop_move',)]
        r.fake.release_stop.set()
        await asyncio.wait_for(r.fake.stop_returned.wait(), 1)
        r.fake.robot_state = 'IDLE'
        r.fake.kin_data['actual_joint_speed'] = [0.0] * 6
        after = len(r.wire.messages)
        _, message = await r.wire.mode('FAULT', after=after)
        assert message['fault'] == fault
        r.wire.send({'v': 1, 'type': 'arm_request', 'request_id': 'forbidden-rearm'})
        _, rejection = await r.wire.wait(lambda message: message.get('type') == 'arm_rejected')
        assert rejection['request_id'] == 'forbidden-rearm'
        await r.frames.stop()
        r.wire.disconnect()
        await finish_cleanup(r.session, r.control)
        assert r.state.teleop_owner is None and not r.state.teleop_sender_tasks
        assert r.state.latest.depth == 0
        assert r.control._fault == fault
        assert r.fake.write_calls[before:] == [('stop_move',)]
        if reply == 'late':
            await wait_until(lambda: any(event.get('kind') == 'stop_rpc_lifecycle'
                and event.get('outcome') == 'returned_late' for event in r.sdk_events))


async def test_background_cleanup_still_joins_control_and_pump_after_session_error():
    r = None
    try:
        with pytest.raises(RuntimeError, match='^synthetic receiver failure$'):
            async with background('legacy') as r:
                r.wire.incoming.put_nowait(RuntimeError('synthetic receiver failure'))
                await wait_until(r.session.done)
        assert r.control._task is None
        assert not r.adapter._pump.running
        assert r.adapter._client is None
        assert not r.state.teleop_sender_tasks and r.state.teleop_owner is None
    finally:
        # Make the failing regression itself safe before fixing the shared helper.
        if r is not None:
            await r.control.stop()
            await r.adapter.disconnect()
