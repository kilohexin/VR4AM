"""Full original VR tick with a TEST-ONLY candidate/pursuit dispatch strategy."""

import asyncio
from collections import deque
from contextlib import asynccontextmanager

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper
from app.control.robot_control import LatestVRFrame
from app.schemas.messages import ControllerState, TeleopMode, VRFrame
from tests.control.test_robot_control import TargetSampleRecorder
from tests.control.transactional_tick_probe import TransactionalControl
from tests.robots.retarget_backpressure_probe import ControlClock
from tests.robots.test_explicit_catch_up import result
from tests.robots.test_lebai_backpressure_experiments import Experiment, joints


class Rig:
    def __init__(self):
        self.run = Experiment()
        self.clock = ControlClock(self.run.clock)
        self.latest = LatestVRFrame()
        self.recorder = TargetSampleRecorder()
        self.control = TransactionalControl(backend=self.run.adapter,
            latest=self.latest, clock=self.clock, recorder=self.recorder,
            mapper=CoordinateMapper(translation_scale=.2, rotation_dead_zone_deg=.5))
        self.seq = 0

    def publish(self, *, grip=True, x=.1, rz=.3, trigger=0):
        self.seq += 1
        self.latest.publish(VRFrame(v=1, type='vr_frame', session_id='transactional-offline',
            seq=self.seq, client_mono_ms=self.clock.now_ns()/1e6,
            tracking_valid=True, visibility='visible',
            right=ControllerState(p=(x, 0, 0),
                q=tuple(Rotation.from_rotvec((0, 0, rz)).as_quat()), grip=grip, trigger=trigger)),
            self.clock.now_ns())

    async def tick(self, *, q=None, elapsed=40, **frame):
        self.clock.advance_ms(elapsed)
        await self.run.adapter.get_state()
        if q is not None:
            self.run.client.ik_results = deque(joints(v) for v in (q if isinstance(q, list) else [q]))
        self.publish(**frame)
        await self.control.tick()

    async def dispatch(self, **kwargs):
        before = len(self.control.requests)
        await self.tick(**kwargs)
        assert len(self.control.requests) == before + 1
        receipt = self.control.requests[-1][1]
        await result(receipt)
        # Same consumed VR frame: collect the result, never issue another target.
        await self.control.tick()
        return receipt

    async def warm(self, pursuit=False):
        for q in [.04, .08, .12, .16, .20, .22]:
            await self.dispatch(q=q)
        assert self.control.coordinator.snapshot().version == 6
        if pursuit:
            await self.dispatch(q=.26)
            assert self.control.pursuit is not None


@asynccontextmanager
async def rig():
    r = Rig()
    await r.run.adapter.connect()
    try:
        await r.control.connect()
        r.publish(grip=False, x=0, rz=0)
        await r.control.tick()
        await r.control.arm()
        r.publish(x=0, rz=0)
        await r.control.tick()
        assert r.control.mode is TeleopMode.ACTIVE
        yield r
    finally:
        r.run.client.release_ik.set()
        r.run.client.kin_data['actual_joint_speed'] = [0.] * 6
        await r.run.adapter.disconnect()
        r.run.factory.assert_awaited_once()
        assert r.run.client.network_calls == r.run.client.discovery_calls == 0


async def test_pending_candidate_keeps_committed_target_and_coalesces_filtered_input():
    async with rig() as r:
        before = r.control.coordinator.snapshot()
        r.run.client.block_ik = True
        await r.tick(q=.04)
        await asyncio.wait_for(r.run.client.ik_started.wait(), 1)
        original_request = r.control.requests[-1][1]
        first_filtered = r.control.latest_requested.model_copy(deep=True)
        for x in [-.1, .2]:
            # No SDK observation while IK holds the SDK lock.
            r.clock.advance_ms(20)
            r.publish(x=x)
            await r.control.tick()
            assert r.control.coordinator.snapshot() == before
            assert tuple(r.control.last_target.p) == before.position
            assert len(r.control.requests) == 1
        assert r.control.latest_requested != first_filtered
        r.run.client.release_ik.set()
        await result(original_request)
        await r.control.tick()
        assert r.control.coordinator.snapshot().version == 1
        assert tuple(r.control.last_target.p) == r.control.coordinator.snapshot().position
        assert len(r.control.requests) == 1
        await r.dispatch(q=.08, x=.2)
        assert r.control.coordinator.snapshot().version == 2
        assert r.control.candidate_inputs[-1] == r.control.latest_requested


async def test_pursuit_keeps_history_while_original_mapping_and_filter_read_new_hands():
    async with rig() as r:
        await r.warm(pursuit=True)
        before = r.control.coordinator.snapshot()
        accepted = r.run.adapter._accepted_target_ref
        ik_count = len(r.run.client.ik_calls)
        filtered = r.control.latest_requested.model_copy(deep=True)
        assert np.linalg.norm(before.linear_velocity) > 0
        for x, rz in [(-.1, -.3), (.2, .5), (0, -.5)]:
            await r.dispatch(x=x, rz=rz)
            assert r.control.requests[-1][0] == 'catch_up'
            assert r.control.coordinator.snapshot() == before
            assert tuple(r.control.last_target.p) == before.position
            assert r.run.adapter._accepted_target_ref == accepted
        assert r.control.latest_requested != filtered
        assert len(r.run.client.ik_calls) == ik_count
        assert any(s['mapped_tcp'] != s['filtered_tcp'] for s in r.recorder.samples)
        count = len(r.control.requests)
        for _ in range(5):
            await r.control.tick()
        assert len(r.control.requests) == count


@pytest.mark.parametrize('exit_reason', ['observed_settled', 'budget_expired'])
async def test_pursuit_exit_stops_and_requires_explicit_arm_and_fresh_anchor(exit_reason):
    async with rig() as r:
        await r.warm(pursuit=True)
        before = r.control.coordinator.snapshot()
        old_pursuit = r.control.pursuit
        if exit_reason == 'observed_settled':
            await r.dispatch()
            r.run.client.kin_data['actual_joint_pose'] = joints(.22)
            for _ in range(2):
                await r.dispatch(x=-.1)
            await r.tick(x=-.1)
        else:
            await r.tick(elapsed=500, x=-.1)
        assert r.control.mode is TeleopMode.DISARMED
        assert r.control.exits[-1] == exit_reason
        assert r.control.coordinator.snapshot() == before
        count = len(r.control.requests)
        await r.tick(x=.2)
        assert len(r.control.requests) == count
        await r.tick(grip=False, x=-.1)
        await r.control.arm()
        r.run.client.kin_data['actual_tcp_pose']['x'] = .31
        await r.tick(x=-.1)
        assert r.control.mode is TeleopMode.ACTIVE
        assert r.control.last_target.p[0] == pytest.approx(.31)
        assert r.control.filter.value == r.control.last_target
        after = r.control.coordinator.snapshot()
        assert after.linear_velocity == after.angular_velocity == (0., 0., 0.)
        assert not after.braking
        assert r.control.pursuit is None
        assert old_pursuit._check() == 'authority_revoked'
        assert len(r.control.requests) == count
        await r.dispatch(q=r.run.client.kin_data['actual_joint_pose'][0] + .02, x=-.09)
        assert r.control.coordinator.snapshot().version == after.version + 1


@pytest.mark.parametrize('stop_end', ['error', 'late'])
async def test_pursuit_budget_stop_failure_preserves_unverified_latch(stop_end):
    async with rig() as r:
        await r.warm(pursuit=True)
        async def failed_stop():
            r.run.client.write_calls.append(('stop_move',))
            if stop_end == 'error':
                raise RuntimeError('injected_stop_error')
            r.clock.advance_ms(609)
        r.run.client.stop_move = failed_stop
        count = len(r.control.requests)
        await r.tick(elapsed=500)
        assert (await r.control.state_message()).fault == 'stop_unverified'
        assert not (await r.control.reset_fault()).accepted
        with pytest.raises(RuntimeError, match='arm_blocked_by_fault'):
            await r.control.arm()
        assert len(r.control.requests) == count
        assert not any(c[0] in {'stop_sys', 'movej'} for c in r.run.client.write_calls)


async def test_ik_rejection_faults_instead_of_entering_pursuit():
    async with rig() as r:
        await r.warm()
        before = r.control.coordinator.snapshot()
        await r.dispatch(q=[.5, .45, .4])
        assert r.control._fault == 'candidate_dispatch_failed'
        assert r.control.pursuit is None
        assert r.control.coordinator.snapshot() == before
        assert any(c[0] == 'stop_move' for c in r.run.client.write_calls)


@pytest.mark.parametrize('pursuit', [False, True])
@pytest.mark.parametrize('boundary', ['release', 'stale', 'newer', 'disarm'])
async def test_gripper_await_cannot_dispatch_old_candidate_or_pursuit(pursuit, boundary):
    async with rig() as r:
        await r.warm(pursuit=pursuit)
        before = r.control.coordinator.snapshot()
        count = len(r.control.requests)
        entered, release = asyncio.Event(), asyncio.Event()
        claw = r.run.client.set_claw
        async def blocked_claw(*args):
            await claw(*args)
            entered.set()
            await release.wait()
        r.run.client.set_claw = blocked_claw
        task = asyncio.create_task(r.tick(trigger=.8))
        stop = None
        try:
            await asyncio.wait_for(entered.wait(), 1)
            if boundary == 'stale':
                r.clock.advance_ms(100)
            elif boundary == 'release':
                r.publish(grip=False, trigger=.8)
            elif boundary == 'newer':
                r.publish(x=-.1, trigger=.8)
            else:
                stop = asyncio.create_task(r.control.disarm())
                await asyncio.sleep(0)
            release.set()
            await asyncio.wait_for(task, 1)
            if stop is not None:
                await asyncio.wait_for(stop, 2)
            assert len(r.control.requests) == count
            assert r.control.coordinator.snapshot() == before
            if boundary in {'release', 'stale'}:
                await r.control.tick()
                assert any(c[0] == 'stop_move' for c in r.run.client.write_calls)
                assert len(r.control.requests) == count
            elif boundary == 'newer':
                await r.control.tick()
                assert len(r.control.requests) == count + 1
                assert r.control.requests[-1][1].command.command_id == r.seq
        finally:
            release.set()
            await asyncio.gather(task, *([stop] if stop is not None else []), return_exceptions=True)


async def test_pursuit_budget_expiring_during_gripper_await_stops_in_same_tick():
    async with rig() as r:
        await r.warm(pursuit=True)
        await r.tick(elapsed=450)
        await result(r.control.requests[-1][1])
        entered, release = asyncio.Event(), asyncio.Event()
        claw = r.run.client.set_claw
        async def blocked_claw(*args):
            await claw(*args)
            entered.set()
            await release.wait()
        r.run.client.set_claw = blocked_claw
        count = len(r.control.requests)
        task = asyncio.create_task(r.tick(elapsed=20, trigger=.8))
        try:
            await asyncio.wait_for(entered.wait(), 1)
            # Frame stays fresh (40ms); only the pursuit episode budget expires.
            r.clock.advance_ms(40)
            release.set()
            await asyncio.wait_for(task, 2)
            assert r.control.exits[-1] == 'budget_expired'
            assert r.control.mode is TeleopMode.DISARMED
            assert r.control.last_target is None
            assert len(r.control.requests) == count
        finally:
            release.set()
            await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('pursuit', [False, True])
async def test_grip_release_then_recapture_discards_buffered_target(pursuit):
    async with rig() as r:
        await r.warm(pursuit=pursuit)
        before = r.control.coordinator.snapshot()
        await r.tick(grip=False, x=-.2)
        assert r.control.mode is TeleopMode.HOLD
        count = len(r.control.requests)
        await r.tick(x=-.2)
        assert r.control.mode is TeleopMode.ACTIVE
        after = r.control.coordinator.snapshot()
        assert after.version == before.version + 1
        assert after.linear_velocity == after.angular_velocity == (0., 0., 0.)
        assert r.control.latest_requested is None
        assert r.control.pursuit is None
        assert r.control.filter.value == r.control.last_target
        assert len(r.control.requests) == count


@pytest.mark.parametrize('pursuit', [False, True])
@pytest.mark.parametrize('boundary', ['release', 'stale', 'newer'])
async def test_target_recording_await_cannot_dispatch_old_input_or_clear_history(pursuit, boundary):
    async with rig() as r:
        await r.warm(pursuit=pursuit)
        before = r.control.coordinator.snapshot()
        count = len(r.control.requests)
        entered, release = asyncio.Event(), asyncio.Event()
        write = r.recorder.write_event
        async def blocked_write(event, timestamp):
            await write(event, timestamp)
            entered.set()
            await release.wait()
        r.recorder.write_event = blocked_write
        r.control._last_target_sample_ns = None
        task = asyncio.create_task(r.tick(x=-.1))
        try:
            await asyncio.wait_for(entered.wait(), 1)
            if boundary == 'stale':
                r.clock.advance_ms(100)
            else:
                r.publish(grip=boundary != 'release', x=.2)
            release.set()
            await asyncio.wait_for(task, 1)
            assert r.control.coordinator.snapshot() == before
            assert len(r.control.requests) == count
            r.recorder.write_event = write
            await r.control.tick()
            if boundary == 'newer':
                assert len(r.control.requests) == count + 1
                assert r.control.requests[-1][1].command.command_id == r.seq
            else:
                assert len(r.control.requests) == count
                assert any(c[0] == 'stop_move' for c in r.run.client.write_calls)
        finally:
            release.set()
            r.recorder.write_event = write
            await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('failure', ['logging', 'sdk'])
async def test_original_pump_fault_code_survives_transactional_collection(failure):
    async with rig() as r:
        await r.warm()
        before = r.control.coordinator.snapshot()
        if failure == 'logging':
            async def broken_log(event, timestamp):
                if event['kind'] == 'pvat_sent':
                    raise RuntimeError('injected_log_failure')
            r.run.adapter._event_callback = broken_log
        else:
            async def broken_move(*args):
                raise RuntimeError('injected_sdk_failure')
            r.run.client.move_pvat = broken_move
        await r.dispatch(q=.24)
        expected = 'recording_unavailable' if failure == 'logging' else 'sdk_call_failed:move_pvat'
        assert r.control._fault == expected
        assert r.control.coordinator.snapshot() == before
        assert r.control.pursuit is None


async def test_workspace_hold_does_not_bypass_pursuit_budget_supervision():
    async with rig() as r:
        await r.warm(pursuit=True)
        r.control.limiter.limits.workspace_half_extent_m = .001
        count = len(r.control.requests)
        before = r.control.coordinator.snapshot()
        await r.tick(x=.2)
        assert len(r.control.requests) == count
        assert r.control.coordinator.snapshot() == before
        await r.tick(elapsed=500, x=.2)
        assert r.control.mode is TeleopMode.DISARMED
        assert r.control.exits[-1] == 'budget_expired'
        assert len(r.control.requests) == count


@pytest.mark.parametrize('pursuit', [False, True])
async def test_deliberate_trigger_close_and_open_share_original_tick_with_arm_dispatch(pursuit):
    async with rig() as r:
        await r.warm(pursuit=pursuit)
        before = r.control.coordinator.snapshot()
        await r.dispatch(q=.24, trigger=.8)
        await r.dispatch(q=.24, elapsed=100, trigger=0)
        assert [c for c in r.run.client.write_calls if c[0] == 'set_claw'] == [
            ('set_claw', 30, 20), ('set_claw', 30, 100)]
        assert r.control.mode is TeleopMode.ACTIVE
        assert r.control.requests[-1][1].command.command_id == r.seq
        if pursuit:
            assert r.control.coordinator.snapshot() == before
            assert all(kind == 'catch_up' for kind, _ in r.control.requests[-2:])
        else:
            assert r.control.coordinator.snapshot().version == before.version + 2
