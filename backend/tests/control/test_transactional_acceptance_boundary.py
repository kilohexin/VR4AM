"""Receipt/history boundaries with explicit in-memory backend; no SDK/I/O."""
from dataclasses import replace

import pytest

from app.control.target_strategy import TargetContext
from app.control.transactional_settings import PursuitSettings
from app.control.transactional_strategy import TransactionalTargetStrategy
from app.robots.command_admission import CommandAdmission, CommandCompletion, CommandRef
from app.robots.motion_view import AcceptedTarget, MotionView
from app.schemas.messages import Pose
from tests.robots.test_vr_profile_experiments import limiter


OPTIONS = PursuitSettings(500, 40, 40, .001, .002, 3)
ANCHOR = Pose(p=(.3, 0, .4), q=(0, 0, 0, 1))
REQUEST = Pose(p=(.306, 0, .4), q=(0, 0, 0, 1))


class Backend:
    def __init__(self):
        self.receipts = []
        self.pose = None
        self.view = MotionView(0, 7, True, True, None, None, True, False, None, None)

    async def submit_tcp(self, pose, command_id):
        self.pose = pose.model_copy(deep=True)
        ref = CommandRef(command_id, 7, len(self.receipts) + 1)
        receipt = CommandAdmission(ref, None, completion=CommandCompletion(ref))
        self.receipts.append(receipt)
        return receipt

    async def submit_catch_up(self, *args):
        raise AssertionError('unexpected_catch_up')

    def read_motion_view(self):
        return self.view

    def complete(self):
        receipt = self.receipts[-1]
        accepted = AcceptedTarget(receipt.command, tuple(self.pose.p), tuple(self.pose.q), (0.,)*6)
        self.view = replace(self.view, accepted=accepted)
        receipt.completion._resolve('sdk_returned', sdk_outcome='returned',
            accepted_command=receipt.command, pvat_mode='advance')


async def setup():
    backend, limits = Backend(), limiter()
    context = TargetContext(1, 20_000_000, True, True)
    policy = TransactionalTargetStrategy(backend, limits, OPTIONS, context=lambda: context)
    policy.on_anchor(ANCHOR, context)
    plan = policy.prepare(ANCHOR, REQUEST, .02, limiter=limits)
    await policy.dispatch(plan, 1, backend=backend)
    backend.complete()
    return policy, backend, context, plan


@pytest.mark.parametrize('boundary', ['latched', 'preflight', 'detached', 'stopping',
    'acceptance_missing', 'acceptance_replaced', 'accepted_pose_changed'])
async def test_unusable_or_replaced_acceptance_cannot_commit_history(boundary):
    policy, backend, context, _plan = await setup()
    before = policy.coordinator.snapshot()
    view = backend.view
    if boundary == 'latched':
        view = replace(view, latched_fault='stop_unverified')
    elif boundary == 'preflight':
        view = replace(view, preflight_ready=False)
    elif boundary == 'detached':
        view = replace(view, client_attached=False)
    elif boundary == 'stopping':
        view = replace(view, stop_in_progress=True)
    elif boundary == 'acceptance_missing':
        view = replace(view, accepted=None)
    elif boundary == 'acceptance_replaced':
        view = replace(view, accepted=replace(view.accepted,
            command=replace(view.accepted.command, submission_id=999)))
    else:
        view = replace(view, accepted=replace(view.accepted,
            position=(view.accepted.position[0] + .001, *view.accepted.position[1:])))
    backend.view = view
    update = policy.collect(context)
    assert update.committed is None
    assert update.fault == ('stop_unverified' if boundary == 'latched' else 'candidate_dispatch_failed')
    assert policy.coordinator.snapshot() == before, 'failed acceptance must not advance pose/velocity/version'
    assert len(backend.receipts) == 1


async def test_matching_receipt_and_accepted_pose_commit_once():
    policy, backend, context, plan = await setup()
    before = policy.coordinator.snapshot()
    update = policy.collect(context)
    assert update.committed == plan.pose == backend.view.accepted.tcp
    assert update.fault is None
    assert policy.coordinator.snapshot().version == before.version + 1
    assert policy.collect(context).committed is None


@pytest.mark.parametrize('boundary', ['missing', 'replaced', 'pose_changed'])
async def test_committed_history_cannot_survive_unrelated_acceptance_change(boundary):
    policy, backend, context, _plan = await setup()
    assert policy.collect(context).committed is not None
    before = policy.coordinator.snapshot()
    accepted = backend.view.accepted
    if boundary == 'missing':
        accepted = None
    elif boundary == 'replaced':
        accepted = replace(accepted, command=replace(accepted.command, submission_id=999))
    else:
        accepted = replace(accepted, position=(accepted.position[0] + .001, *accepted.position[1:]))
    backend.view = replace(backend.view, accepted=accepted)
    update = policy.collect(context)
    assert update.committed is None and update.fault == 'candidate_dispatch_failed'
    assert policy.coordinator.snapshot() == before


async def test_same_epoch_anchor_cannot_commit_a_completed_old_receipt():
    policy, _backend, context, _plan = await setup()
    before = policy.coordinator.snapshot()
    with pytest.raises(RuntimeError, match='^new_epoch_required$'):
        policy.on_anchor(ANCHOR, context)
    assert policy.coordinator.snapshot() == before


async def test_acceptance_change_after_prepare_cannot_submit_a_new_command():
    policy, backend, context, _plan = await setup()
    assert policy.collect(context).committed is not None
    before = policy.coordinator.snapshot()
    plan = policy.prepare(policy.coordinator.pose, REQUEST, .02, limiter=policy.limits)
    backend.view = replace(backend.view, accepted=replace(backend.view.accepted,
        command=replace(backend.view.accepted.command, submission_id=999)))
    with pytest.raises(RuntimeError, match='candidate_dispatch_failed'):
        await policy.dispatch(plan, 2, backend=backend)
    assert len(backend.receipts) == 1
    assert policy.coordinator.snapshot() == before


async def test_old_collect_context_cannot_commit_new_epoch_history():
    policy, backend, old_context, _plan = await setup()
    current = replace(old_context, generation=2)
    policy.context = lambda: current
    policy.on_anchor(ANCHOR, current)
    plan = policy.prepare(ANCHOR, REQUEST, .02, limiter=policy.limits)
    await policy.dispatch(plan, 2, backend=backend)
    backend.complete()
    before = policy.coordinator.snapshot()
    update = policy.collect(old_context)
    assert update.committed is None and update.fault is None
    assert policy.coordinator.snapshot() == before
    assert policy.collect(current).committed == plan.pose


@pytest.mark.parametrize('collected', [False, True])
async def test_real_control_stops_when_other_fake_request_replaces_acceptance(collected):
    from collections import deque
    from tests.control.test_transactional_vr_tick import rig
    from tests.robots.test_lebai_backpressure_experiments import joints
    from tests.robots.test_explicit_catch_up import result
    async with rig() as r:
        await r.tick(q=.04)
        own = r.control.requests[-1][1]
        await result(own)
        if collected:
            await r.control.tick()
        before = r.control.coordinator.snapshot()
        accepted = r.run.adapter.read_motion_view().accepted
        assert accepted.command == own.command
        r.clock.advance_ms(40)
        r.publish()
        await r.run.adapter.get_state()
        r.run.client.ik_results = deque([joints(.08)])
        # A separate public submission really replaces the adapter's accepted
        # command; no private fields are overwritten in this integration test.
        other = await r.run.adapter.submit_tcp(accepted.tcp, 999)
        assert (await result(other)).pvat_mode == 'advance'
        assert r.run.adapter.read_motion_view().accepted.command == other.command
        await r.control.tick()
        assert r.control._fault == 'candidate_dispatch_failed'
        assert r.control.coordinator.snapshot() == before
        assert len(r.control.requests) == 1
        assert [call[0] for call in r.run.client.write_calls] == ['move_pvat', 'move_pvat', 'stop_move']


async def test_real_control_matching_advances_keep_owner_and_adapter_target_in_sync():
    from tests.control.test_transactional_vr_tick import rig
    async with rig() as r:
        for version, q in enumerate([.04, .08], 1):
            await r.dispatch(q=q)
            accepted = r.run.adapter.read_motion_view().accepted
            assert r.control.coordinator.committed_command == accepted.command
            assert r.control.last_target == r.control.coordinator.pose == accepted.tcp
            assert r.control.coordinator.snapshot().version == version
            assert r.control._fault is None
        assert [call[0] for call in r.run.client.write_calls] == ['move_pvat', 'move_pvat']
