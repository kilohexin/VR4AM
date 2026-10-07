"""Full ASGI WebSocket + app lifespan + original loop, injected SDK only.

Real monotonic scheduling, synthetic IK replies/telemetry, no TCP server or SDK
import. This verifies software composition, never physical dynamics or stopping.
"""

import asyncio
from contextlib import asynccontextmanager
from dataclasses import asdict, replace
import json
from unittest.mock import AsyncMock

import pytest

from app.config import Settings
from app.control.robot_control import RobotControl
from app.control.transactional_settings import PursuitSettings
from app.control.transactional_strategy import TransactionalTargetStrategy
from app.main import create_app
from app.recording.commissioning import CommissioningRecorder
from app.robots.base import BackendCommandError
from app.schemas.messages import TeleopMode
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_backpressure_experiments import joints


OPTIONS = PursuitSettings(1000, 80, 80, .001, .002, 3)


async def until(predicate, *, timeout=2):
    async def wait():
        while not predicate():
            await asyncio.sleep(.001)
    await asyncio.wait_for(wait(), timeout)


class Socket:
    def __init__(self, app):
        self.app = app
        self.incoming = asyncio.Queue()
        self.messages = []
        self.accepted = asyncio.Event()
        self.closed = False
        self.task = None

    async def receive(self):
        return await self.incoming.get()

    async def send(self, event):
        if event['type'] == 'websocket.accept':
            self.accepted.set()
        elif event['type'] == 'websocket.send':
            self.messages.append(json.loads(event['text']))
        elif event['type'] == 'websocket.close':
            self.closed = True

    async def start(self):
        scope = dict(type='websocket', asgi={'version': '3.0', 'spec_version': '2.4'},
            scheme='ws', path='/ws/v1/teleop', raw_path=b'/ws/v1/teleop',
            query_string=b'', root_path='', headers=[], client=('synthetic', 0),
            server=('offline', 0), subprotocols=[])
        self.incoming.put_nowait({'type': 'websocket.connect'})
        self.task = asyncio.create_task(self.app(scope, self.receive, self.send), name='test-asgi-websocket')
        await asyncio.wait_for(self.accepted.wait(), 2)
        return self

    def put(self, payload):
        self.incoming.put_nowait({'type': 'websocket.receive', 'text': json.dumps(payload)})

    async def request(self, kind, expected, request_id):
        self.put(dict(v=1, type=kind, request_id=request_id))
        await until(lambda: any(m.get('request_id') == request_id
            for m in self.messages) or self.task.done())
        if self.task.done():
            await self.task
        response = next(m for m in self.messages if m.get('request_id') == request_id)
        assert response['type'] == expected, response
        return response

    async def disconnect(self):
        if not self.task.done():
            self.incoming.put_nowait({'type': 'websocket.disconnect', 'code': 1000})
        await asyncio.wait_for(self.task, 2)


class Session:
    def __init__(self, app, sdk, recorder):
        self.app, self.sdk, self.recorder = app, sdk, recorder
        self.control = app.state.control
        self.policy = self.control.target_strategy
        self.socket = Socket(app)
        self.seq = 0
        self.grip = False
        self.trigger = 0.
        self.x = 0.
        self.solution = 0.
        self.feed = None

    def frame(self):
        self.seq += 1
        self.socket.put(dict(v=1, type='vr_frame', session_id='injected-session', seq=self.seq,
            client_mono_ms=self.control.clock.now_ns() / 1e6, tracking_valid=True, visibility='visible',
            right=dict(p=[self.x, 0., 0.], q=[0., 0., 0., 1.], grip=self.grip, trigger=self.trigger)))

    async def heartbeat(self):
        while True:
            self.frame()
            await asyncio.sleep(.02)

    async def start(self):
        await self.socket.start()
        await self.socket.request('hello', 'hello_ack', 'hello')
        previous_seq = self.seq
        self.feed = asyncio.create_task(self.heartbeat(), name='test-vr-heartbeat')
        await until(lambda: self.control.last_seq is not None and self.control.last_seq > previous_seq)
        await self.socket.request('arm_request', 'arm_ack', 'arm')
        self.grip = True
        await until(lambda: self.control.mode == TeleopMode.ACTIVE)

    async def advance(self, solution):
        version = self.policy.coordinator.snapshot().version
        self.solution = solution
        self.x = .1
        await until(lambda: self.app.state.backend.read_motion_view().accepted is not None
            and self.app.state.backend.read_motion_view().accepted.solution_q[0] == pytest.approx(solution))
        await until(lambda: self.policy.coordinator.snapshot().version > version)

    async def warm(self, pursuit=False):
        for q in (.04, .08, .12, .16, .20, .22):
            await self.advance(q)
        if pursuit:
            self.solution = .26
            await until(lambda: self.policy.pursuit is not None)
            await until(lambda: self.policy.pursuit.dispatched)

    async def stop_feed(self):
        if self.feed is not None:
            self.feed.cancel()
            await asyncio.gather(self.feed, return_exceptions=True)
            self.feed = None

    async def close(self):
        await self.stop_feed()
        self.sdk.release_ik.set()
        await self.socket.disconnect()


@asynccontextmanager
async def session(tmp_path, monkeypatch, *, options=OPTIONS, allow_stop_error=False,
                  expected_session_error=None):
    import app.main as main
    real = control_settings()
    real = replace(real, control=replace(real.control,
        max_joint_speed_radps=.05, max_tcp_speed_mps=.005,
        max_tcp_rotation_radps=.05, max_tcp_acceleration_mps2=.02,
        max_tcp_angular_acceleration_radps2=.1, max_tcp_step_m=.0005,
        max_tcp_rotation_step_deg=.2, translation_scale=.2))
    settings = replace(Settings.load(), backend='lebai', lebai=real, transactional_target=options)
    sdk = FakeLebaiClient.idle()
    factory = AsyncMock(return_value=sdk)
    metadata = dict(backend='LEBAI', runtime='INJECTED_SDK_TEST', hardware_verified=False,
        target_strategy='transactional_experimental', pursuit_settings=asdict(options))
    recorder = CommissioningRecorder(tmp_path, metadata, session_id='synthetic-transactional')
    monkeypatch.setattr(main, 'build_recorder', lambda *args: recorder)
    app = create_app(settings=settings, client_factory=factory, backend_label='LEBAI')
    r = None
    try:
        async with app.router.lifespan_context(app):
            r = Session(app, sdk, recorder)
            r.cleanup_errors = []
            original_ik = sdk.kinematics_inverse
            async def scripted_ik(pose, seed):
                sdk.ik_results.clear()
                sdk.ik_results.append(joints(r.solution))
                return await original_ik(pose, seed)
            sdk.kinematics_inverse = scripted_ik
            try:
                assert type(r.control) is RobotControl
                assert type(r.policy) is TransactionalTargetStrategy
                await r.start()
                yield r
            finally:
                try:
                    await r.close()
                except BackendCommandError as error:
                    if not ((allow_stop_error and r.control._fault == 'stop_unverified')
                            or str(error) == expected_session_error):
                        raise
                    r.cleanup_errors.append(str(error))
            assert app.state.teleop_owner is None
            assert not app.state.teleop_sender_tasks
            assert app.state.latest.depth == 0
    except BackendCommandError as error:
        if not allow_stop_error or r.control._fault != 'stop_unverified':
            raise
        r.cleanup_errors.append(str(error))
    factory.assert_awaited_once()
    assert sdk.network_calls == sdk.discovery_calls == 0
    assert r.control._task is None or r.control._task.done()
    assert not app.state.backend._pump.running
    assert recorder._writer_task is None or recorder._writer_task.done()
    assert not any(c[0] in {'stop_sys', 'movej'} for c in sdk.write_calls)


async def test_motion_gripper_release_and_jsonl_request_correlation(tmp_path, monkeypatch):
    async with session(tmp_path, monkeypatch) as r:
        await r.warm()
        r.trigger = .8
        await until(lambda: any(c[0] == 'set_claw' and c[2] == 20 for c in r.sdk.write_calls))
        r.trigger = 0.
        await until(lambda: any(c[0] == 'set_claw' and c[2] == 100 for c in r.sdk.write_calls))
        r.grip = False
        await until(lambda: r.control.mode == TeleopMode.HOLD)
        writes = len([c for c in r.sdk.write_calls if c[0] == 'move_pvat'])
        await asyncio.sleep(.06)
        assert len([c for c in r.sdk.write_calls if c[0] == 'move_pvat']) == writes
        assert r.policy._authority()[1] is False
    events = [json.loads(line) for line in r.recorder.jsonl_path.read_text(encoding='utf-8').splitlines()]
    assert any(e['kind'] == 'session_started' for e in events)
    pvat = [e for e in events if e['kind'] == 'pvat_sent']
    assert pvat and all(e['submission_id'] > 0 for e in pvat)
    frames = {e['frame']['seq'] for e in events if e['kind'] == 'vr_frame'}
    assert all(e['command_id'] in frames for e in pvat)
    assert any(e['kind'] == 'stop_confirmed' for e in events)


async def test_tracking_pursuit_uses_accepted_solution_and_exits_on_observations(tmp_path, monkeypatch):
    async with session(tmp_path, monkeypatch) as r:
        await r.warm(pursuit=True)
        accepted = r.app.state.backend.read_motion_view().accepted
        count = len(r.sdk.ik_calls)
        r.x = -.1
        r.trigger = .8
        await until(lambda: any(c[0] == 'set_claw' and c[2] == 20 for c in r.sdk.write_calls))
        assert r.app.state.backend.read_motion_view().accepted == accepted
        assert len(r.sdk.ik_calls) == count
        r.sdk.kin_data['actual_joint_pose'] = joints(.22)
        await until(lambda: r.control.mode == TeleopMode.DISARMED)
        assert r.policy.last_exit == 'observed_settled'
        assert r.control.last_target is None


async def test_budget_exit_requires_explicit_arm_and_new_anchor(tmp_path, monkeypatch):
    async with session(tmp_path, monkeypatch, options=replace(OPTIONS, budget_ms=200)) as r:
        await r.warm(pursuit=True)
        old = r.policy.coordinator.snapshot()
        await until(lambda: r.control.mode == TeleopMode.DISARMED)
        assert r.policy.last_exit == 'budget_expired'
        assert r.policy.coordinator.snapshot() == old
        count = len([c for c in r.sdk.write_calls if c[0] == 'move_pvat'])
        await asyncio.sleep(.06)
        assert len([c for c in r.sdk.write_calls if c[0] == 'move_pvat']) == count
        rejected = await r.socket.request('arm_request', 'arm_rejected', 'held-arm')
        assert rejected['type'] == 'arm_rejected'
        r.grip = False
        await until(lambda: r.control.mode == TeleopMode.READY)
        await r.socket.request('arm_request', 'arm_ack', 'rearm')
        r.solution = .02
        r.grip = True
        await until(lambda: r.control.mode == TeleopMode.ACTIVE)
        assert r.policy.pursuit is None
        assert r.policy.coordinator.snapshot().position[0] == pytest.approx(.3)
        assert r.policy.coordinator.snapshot().position != old.position
        assert r.policy.coordinator.snapshot().linear_velocity == (0., 0., 0.)


@pytest.mark.parametrize('boundary', ['release', 'disconnect'])
async def test_pending_ik_revocation_prevents_late_dispatch(tmp_path, monkeypatch, boundary):
    async with session(tmp_path, monkeypatch) as r:
        r.sdk.block_ik = True
        r.sdk.ik_started.clear()
        r.solution = .04
        r.x = .1
        await asyncio.wait_for(r.sdk.ik_started.wait(), 1)
        receipt = r.policy.last_receipt
        r.x = -.1
        if boundary == 'release':
            r.grip = False
            await until(lambda: receipt.completion.invalidated)
        else:
            await r.stop_feed()
            r.socket.incoming.put_nowait({'type': 'websocket.disconnect', 'code': 1000})
            await until(lambda: receipt.completion.invalidated)
        r.sdk.release_ik.set()
        await until(lambda: receipt.completion.result is not None)
        assert receipt.completion.result.invalidated
        assert not any(c[0] == 'move_pvat' for c in r.sdk.write_calls)
        if boundary == 'release':
            await until(lambda: r.control.mode == TeleopMode.HOLD)
        else:
            await asyncio.wait_for(r.socket.task, 2)
            assert r.app.state.teleop_owner is None


@pytest.mark.parametrize('pursuit', [False, True])
async def test_frame_stream_loss_stops_candidate_and_pursuit(tmp_path, monkeypatch, pursuit):
    async with session(tmp_path, monkeypatch) as r:
        await r.warm(pursuit=pursuit)
        await r.stop_feed()
        await until(lambda: r.control.mode == TeleopMode.STALE)
        await until(lambda: any(c[0] == 'stop_move' for c in r.sdk.write_calls))
        count = len([c for c in r.sdk.write_calls if c[0] == 'move_pvat'])
        await asyncio.sleep(.06)
        assert len([c for c in r.sdk.write_calls if c[0] == 'move_pvat']) == count
        assert not r.policy._authority()[1]


async def test_sdk_motion_failure_latches_original_fault_and_revokes_session_motion(tmp_path, monkeypatch):
    async with session(tmp_path, monkeypatch, expected_session_error='sdk_call_failed:move_pvat') as r:
        await r.warm()
        async def fail(*args):
            raise RuntimeError('injected_move_failure')
        r.sdk.move_pvat = fail
        await until(lambda: r.app.state.backend.pump_fault is not None)
        assert str(r.app.state.backend.pump_fault) == 'sdk_call_failed:move_pvat'
        await until(lambda: r.socket.task.done())
        assert str(r.socket.task.exception()) == 'sdk_call_failed:move_pvat'
        count = len(r.sdk.ik_calls)
        await asyncio.sleep(.06)
        assert len(r.sdk.ik_calls) == count
        assert r.control.last_target is None


async def test_disconnect_reconnect_releases_owner_and_discards_old_input(tmp_path, monkeypatch):
    async with session(tmp_path, monkeypatch) as r:
        await r.warm(pursuit=True)
        old_epoch = r.control.control_generation
        await r.close()
        assert r.app.state.teleop_owner is None and r.app.state.latest.depth == 0
        assert not r.app.state.teleop_sender_tasks
        r.socket = Socket(r.app)
        r.grip = False
        r.trigger = 0.
        r.solution = .02
        await r.start()
        assert r.control.control_generation > old_epoch
        assert r.policy.pursuit is None
        assert r.policy.coordinator.snapshot().linear_velocity == (0., 0., 0.)
        assert r.control.latest is r.app.state.latest


async def test_late_stop_keeps_unverified_latch_and_rejects_reset(tmp_path, monkeypatch):
    async with session(tmp_path, monkeypatch, allow_stop_error=True) as r:
        await r.warm()
        completed = asyncio.Event()
        original = r.sdk.stop_move
        async def delayed_stop():
            await asyncio.sleep(.609)
            await original()
            completed.set()
        r.sdk.stop_move = delayed_stop
        r.grip = False
        await until(lambda: r.control._fault == 'stop_unverified')
        await asyncio.wait_for(completed.wait(), 2)
        assert r.app.state.backend.read_motion_view().latched_fault == 'stop_unverified'
        assert r.control.mode == TeleopMode.FAULT
        result = await r.socket.request('reset_fault', 'fault_reset_result', 'reset-late-stop')
        assert not result['accepted']
        assert r.control._fault == 'stop_unverified'
    assert not any(c[0] == 'stop_sys' for c in r.sdk.write_calls)
