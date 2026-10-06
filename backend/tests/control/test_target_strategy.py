"""Strategy seams remain inside the original owner freshness/stop boundary."""

import asyncio

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.target_strategy import TargetPlan
from app.robots.base import BackendCommandError
from app.schemas.messages import Pose
from tests.control.test_robot_control import FakeBackend, FakeClock, TargetSampleRecorder, frame


class Strategy:
    def __init__(self, kind='candidate', commit=False):
        self.kind, self.commit = kind, commit
        self.pose = Pose(p=(.301, 0, .3), q=(0, 0, 0, 1))
        self.prepared = []
        self.dispatched = []
        self.entered, self.release = asyncio.Event(), asyncio.Event()
        self.block = False
        self.error = None

    def prepare(self, previous, requested, dt, *, limiter):
        plan = TargetPlan(self.kind, None if self.kind == 'wait' else self.pose.model_copy(deep=True))
        self.prepared.append(plan)
        return plan

    async def dispatch(self, plan, command_id, *, backend):
        self.dispatched.append((plan, command_id))
        self.entered.set()
        if self.block:
            await self.release.wait()
        if self.error:
            raise BackendCommandError(self.error)
        await backend.command_tcp(plan.pose, command_id)
        return plan.pose if self.commit else None


async def setup(strategy):
    backend, clock, latest, recorder = FakeBackend(), FakeClock(), LatestVRFrame(), TargetSampleRecorder()
    control = RobotControl(backend=backend, clock=clock, latest=latest,
        recorder=recorder, target_strategy=strategy)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    clock.advance_ms(20)
    latest.publish(frame(3, True, p=(.01, 1.2, -.3)), clock.now_ns())
    return control, backend, clock, latest, recorder


def delay_default_backend(backend, gate):
    original = backend.command_tcp
    async def command(target, command_id):
        gate.entered.set()
        await gate.release.wait()
        if gate.error:
            raise BackendCommandError(gate.error)
        await original(target, command_id)
    backend.command_tcp = command


@pytest.mark.parametrize(('kind', 'commit'), [('candidate', True), ('candidate', False), ('catch_up', False), ('wait', False)])
async def test_sampling_describes_plan_and_dispatch_result_owns_target_commit(kind, commit):
    strategy = Strategy(kind, commit)
    control, backend, _, _, recorder = await setup(strategy)
    before = control.last_target.model_copy(deep=True)
    await control.tick()
    sample = recorder.samples[-1]
    assert sample['dispatch_kind'] == kind
    assert sample['limited_tcp'] == (None if kind == 'wait' else strategy.pose.model_dump(mode='json'))
    assert control.last_target == (strategy.pose if commit else before)
    assert len(backend.targets) == (0 if kind == 'wait' else 1)
    assert len(strategy.dispatched) == (0 if kind == 'wait' else 1)


@pytest.mark.parametrize('end', ['returned', 'soft_error'])
@pytest.mark.parametrize('boundary', ['newer', 'stale', 'disarm'])
@pytest.mark.parametrize('use_default', [False, True])
async def test_dispatch_return_cannot_write_old_pose_or_consume_new_input(end, boundary, use_default):
    strategy = Strategy(commit=True)
    strategy.block = True
    strategy.error = 'ik_tracking_lag' if end == 'soft_error' else None
    control, backend, clock, latest, _ = await setup(None if use_default else strategy)
    if use_default:
        delay_default_backend(backend, strategy)
    before = control.last_target.model_copy(deep=True)
    task = asyncio.create_task(control.tick())
    try:
        await asyncio.wait_for(strategy.entered.wait(), 1)
        if boundary == 'newer':
            latest.publish(frame(4, True), clock.now_ns())
        elif boundary == 'stale':
            clock.advance_ms(100)
        else:
            await control.disarm()
        strategy.release.set()
        await asyncio.wait_for(task, 1)
        assert control.last_target == (None if boundary == 'disarm' else before)
        assert control.last_seq == 2
        assert control._constraint is None
    finally:
        strategy.release.set()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('boundary', ['newer', 'stale', 'disarm'])
async def test_prepared_plan_is_abandoned_when_recording_revokes_input(boundary):
    strategy = Strategy(commit=True)
    control, backend, clock, latest, recorder = await setup(strategy)
    entered, release = asyncio.Event(), asyncio.Event()
    original = recorder.write_event
    async def record(event, timestamp):
        await original(event, timestamp)
        entered.set()
        await release.wait()
    recorder.write_event = record
    task = asyncio.create_task(control.tick())
    try:
        await asyncio.wait_for(entered.wait(), 1)
        if boundary == 'newer':
            latest.publish(frame(4, True), clock.now_ns())
        elif boundary == 'stale':
            clock.advance_ms(100)
        else:
            await control.disarm()
        release.set()
        await asyncio.wait_for(task, 1)
        assert len(strategy.prepared) == 1
        assert strategy.dispatched == []
        assert backend.targets == []
    finally:
        release.set()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize('boundary', ['newer', 'stale', 'disarm'])
@pytest.mark.parametrize('use_default', [False, True])
async def test_late_hard_failure_is_not_discarded_as_old_input(boundary, use_default):
    strategy = Strategy(commit=True)
    strategy.block = True
    strategy.error = 'sdk_call_failed:move_pvat'
    control, backend, clock, latest, _ = await setup(None if use_default else strategy)
    if use_default:
        delay_default_backend(backend, strategy)
    task = asyncio.create_task(control.tick())
    try:
        await asyncio.wait_for(strategy.entered.wait(), 1)
        if boundary == 'stale':
            clock.advance_ms(100)
        elif boundary == 'newer':
            latest.publish(frame(4, True), clock.now_ns())
        else:
            await control.disarm()
        strategy.release.set()
        await asyncio.wait_for(task, 1)
        assert control._fault == 'sdk_call_failed:move_pvat'
        assert backend.stops
    finally:
        strategy.release.set()
        await asyncio.gather(task, return_exceptions=True)


@pytest.mark.parametrize(('kind', 'pose'), [('unknown', None), ('candidate', None), ('catch_up', None),
    ('wait', Pose(p=(0, 0, 0), q=(0, 0, 0, 1)))])
def test_plan_rejects_ambiguous_sampling_contract(kind, pose):
    with pytest.raises(ValueError, match='invalid_target_plan'):
        TargetPlan(kind, pose)


async def test_real_tick_samples_actual_draft_catch_up_and_wait_without_backend_facade():
    from tests.control.test_transactional_vr_tick import rig
    from tests.robots.test_explicit_catch_up import result
    async with rig() as r:
        assert r.control.backend is r.run.adapter
        before = r.control.coordinator.snapshot()
        r.run.client.block_ik = True
        await r.tick(q=.04)
        await asyncio.wait_for(r.run.client.ik_started.wait(), 1)
        receipt = r.control.requests[-1][1]
        sample = r.recorder.samples[-1]
        assert sample['dispatch_kind'] == 'candidate'
        assert sample['limited_tcp'] == r.control.coordinator._pending.pose.model_dump(mode='json')
        assert sample['limited_tcp'] != sample['previous_target_tcp']
        assert r.control.coordinator.snapshot() == before
        r.clock.advance_ms(20)
        r.publish(x=-.1)
        r.control._last_target_sample_ns = None
        await r.control.tick()
        assert r.recorder.samples[-1]['dispatch_kind'] == 'wait'
        assert r.recorder.samples[-1]['limited_tcp'] is None
        r.run.client.release_ik.set()
        await result(receipt)
        await r.control.tick()
        for q in [.08, .12, .16, .20, .22]:
            await r.dispatch(q=q)
        await r.dispatch(q=.26)
        r.control._last_target_sample_ns = None
        await r.dispatch()
        assert r.recorder.samples[-1]['dispatch_kind'] == 'catch_up'
        assert r.recorder.samples[-1]['limited_tcp'] == r.run.adapter._last_sent_tcp.model_dump(mode='json')


async def test_prepared_candidate_can_be_abandoned_and_cannot_be_reused_or_transferred():
    from tests.control.test_candidate_commit import setup as candidate_setup, finish
    from tests.control.test_braking_approach import ORIGIN, pose_delta
    coordinator, backend, _ = candidate_setup()
    before = coordinator.snapshot()
    abandoned = coordinator.prepare(pose_delta(ORIGIN, .02), .02)
    assert coordinator.snapshot() == before
    assert backend.receipts == []
    fresh = coordinator.prepare(pose_delta(ORIGIN, -.02), .02)
    receipt = await coordinator.submit_prepared(fresh, 1)
    finish(receipt)
    assert coordinator.settle() == 'committed'
    for draft in (abandoned, fresh):
        with pytest.raises(RuntimeError, match='prepared_candidate_obsolete'):
            await coordinator.submit_prepared(draft, 2)
    other, _, _ = candidate_setup()
    with pytest.raises(RuntimeError, match='prepared_candidate_obsolete'):
        await other.submit_prepared(abandoned, 3)
