"""Owner applies synchronous lifecycle decisions; fake backend only."""

import asyncio

import pytest

from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.target_strategy import TargetUpdate
from app.schemas.messages import Pose, TeleopMode
from tests.control.test_robot_control import FakeBackend, FakeClock, TargetSampleRecorder, frame


class Lifecycle:
    def __init__(self):
        self.anchors, self.revoked, self.contexts = [], [], []
        self.update = None
        self.fail = None
        self.error_type = RuntimeError

    def on_anchor(self, pose, context):
        self.anchors.append((pose, context))
        if self.fail == 'anchor':
            raise self.error_type('injected_anchor_failure')

    def on_revoke(self, generation):
        self.revoked.append(generation)
        if self.fail == 'revoke':
            raise self.error_type('injected_revoke_failure')

    def collect(self, context):
        self.contexts.append(context)
        if self.fail == 'collect':
            raise self.error_type('injected_collect_failure')
        return self.update or TargetUpdate(context.generation)


async def setup():
    life, backend, clock, latest = Lifecycle(), FakeBackend(), FakeClock(), LatestVRFrame()
    control = RobotControl(backend=backend, clock=clock, latest=latest,
        recorder=TargetSampleRecorder(), target_lifecycle=life)
    await control.connect()
    latest.publish(frame(1, False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(frame(2, True), clock.now_ns())
    await control.tick()
    return control, life, backend, clock, latest


async def test_anchor_is_reported_once_after_activation_with_detached_pose():
    control, life, _, _, _ = await setup()
    assert len(life.anchors) == 1
    pose, context = life.anchors[0]
    assert context.authorized
    assert context.generation == control.control_generation
    pose.p = (.4, 0, .3)
    assert control.last_target.p == (.3, 0, .3)
    await control.tick()
    assert len(life.anchors) == 1


@pytest.mark.parametrize('boundary', ['current', 'old_generation', 'stale', 'release', 'new_session'])
async def test_collected_pose_requires_live_authority_and_generation(boundary):
    control, life, _, clock, latest = await setup()
    before = control.last_target.model_copy(deep=True)
    pose = Pose(p=(.31, 0, .3), q=(0, 0, 0, 1))
    life.update = TargetUpdate(control.control_generation - int(boundary == 'old_generation'), committed=pose)
    if boundary == 'stale':
        clock.advance_ms(100)
    elif boundary == 'release':
        latest.publish(frame(3, False), clock.now_ns())
    elif boundary == 'new_session':
        latest.publish(frame(1, True, session_id='new'), clock.now_ns())
    await control.tick()
    if boundary == 'current':
        assert control.last_target == pose
        pose.p = (.4, 0, .3)
        assert control.last_target.p == (.31, 0, .3)
    elif boundary == 'old_generation':
        assert control.last_target == before
    else:
        assert control.last_target != pose
        assert control.mode is not TeleopMode.ACTIVE


async def test_stop_decision_uses_owner_and_revoke_precedes_backend_stop():
    control, life, backend, _, _ = await setup()
    original = backend.stop
    async def stop(reason):
        assert life.revoked[-1] == control.control_generation
        await original(reason)
    backend.stop = stop
    life.update = TargetUpdate(control.control_generation, stop=True)
    await control.tick()
    assert control.mode is TeleopMode.DISARMED
    assert control.last_target is None
    assert len(backend.stops) == 1
    assert backend.targets == []


async def test_current_generation_fault_is_observed_even_with_stale_input():
    control, life, backend, clock, _ = await setup()
    life.update = TargetUpdate(control.control_generation, fault='sdk_call_failed:move_pvat')
    clock.advance_ms(100)
    await control.tick()
    assert control._fault == 'sdk_call_failed:move_pvat'
    assert backend.stops


@pytest.mark.parametrize('hook', ['anchor', 'collect', 'revoke'])
@pytest.mark.parametrize('error_type', [RuntimeError, asyncio.CancelledError])
async def test_callback_failure_cannot_prevent_stop_or_grant_authority(hook, error_type):
    control, life, backend, clock, latest = await setup()
    life.fail = hook
    life.error_type = error_type
    if hook == 'anchor':
        latest.publish(frame(3, False), clock.now_ns())
        await control.tick()
        latest.publish(frame(4, True), clock.now_ns())
    elif hook == 'revoke':
        await control.disarm()
        assert backend.stops
        assert not control._control_generation_is_current(control.control_generation)
    await control.tick()
    assert control._fault == 'target_strategy_failed'
    assert backend.stops
    assert backend.targets == []


@pytest.mark.parametrize('actions', [dict(stop=True, fault='bad'),
    dict(stop=True, committed=Pose(p=(0, 0, 0), q=(0, 0, 0, 1))),
    dict(fault='bad', committed=Pose(p=(0, 0, 0), q=(0, 0, 0, 1)))])
def test_update_cannot_mix_commit_stop_and_fault(actions):
    with pytest.raises(ValueError, match='conflicting_target_update'):
        TargetUpdate(1, **actions)


@pytest.mark.parametrize('action', [dict(stop=True), dict(fault='old_fault')])
async def test_old_generation_actions_are_ignored(action):
    control, life, backend, _, _ = await setup()
    life.update = TargetUpdate(control.control_generation - 1, **action)
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    assert control._fault is None
    assert backend.stops == []


async def test_revoke_hook_failure_cannot_replace_unverified_stop():
    from app.robots.base import BackendCommandError
    control, life, backend, _, _ = await setup()
    life.fail = 'revoke'
    async def failed_stop(reason):
        backend.stops.append(reason)
        raise BackendCommandError('sdk_timeout:stop_move')
    backend.stop = failed_stop
    with pytest.raises(BackendCommandError):
        await control.disarm()
    await control.tick()
    assert control._fault == 'stop_unverified'
    assert not (await control.reset_fault()).accepted


async def test_owner_stop_cancellation_still_propagates_and_latches():
    control, life, backend, _, _ = await setup()
    entered = asyncio.Event()
    async def blocked_stop(reason):
        backend.stops.append(reason)
        entered.set()
        await asyncio.Event().wait()
    backend.stop = blocked_stop
    life.update = TargetUpdate(control.control_generation, stop=True)
    task = asyncio.create_task(control.tick())
    try:
        await asyncio.wait_for(entered.wait(), 1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert control._fault == 'stop_unverified'
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)


async def test_invalid_collect_result_faults_without_motion():
    control, life, backend, _, _ = await setup()
    life.collect = lambda context: None
    await control.tick()
    assert control._fault == 'target_strategy_failed'
    assert backend.stops
    assert backend.targets == []


def test_transactional_fixture_uses_original_tick_and_unwrapped_limiter():
    from app.control.safety import SafetyLimiter
    from tests.control.test_transactional_vr_tick import Rig
    r = Rig()
    assert type(r.control).tick is RobotControl.tick
    assert type(r.control.limiter) is SafetyLimiter
    assert r.control.backend is r.run.adapter
