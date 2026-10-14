"""Test-only candidate transactions; never infer physical speed from SDK return."""

import asyncio
from collections import deque

import pytest

from app.robots.command_admission import CommandAdmission, CommandCompletion, CommandRef
from app.schemas.messages import TeleopMode
from tests.control.candidate_commit_probe import CandidateCoordinator
from tests.control.test_braking_approach import ORIGIN, pose_delta
from tests.control.test_braking_supervision import active_control
from tests.control.elapsed_step_probe import publish
from tests.robots.retarget_backpressure_probe import ControlClock
from tests.robots.test_lebai_backpressure_experiments import Experiment, joints, wait_until
from tests.robots.test_vr_profile_experiments import limiter


class AdmissionBackend:
    def __init__(self):
        self.receipts = []
        self.generation = 0

    async def submit_tcp(self, pose, command_id):
        ref = CommandRef(command_id, self.generation, len(self.receipts) + 1)
        receipt = CommandAdmission(ref, None, completion=CommandCompletion(ref))
        self.receipts.append(receipt)
        return receipt


def setup():
    backend = AdmissionBackend()
    authority = {'epoch': 1, 'active': True}
    coordinator = CandidateCoordinator(backend, limiter(), ORIGIN,
        authority=lambda: (authority['epoch'], authority['active']),
        generation_current=lambda g: g == backend.generation)
    return coordinator, backend, authority


def finish(receipt, outcome='sdk_returned', mode='advance', **overrides):
    details = dict(sdk_outcome='returned', accepted_command=receipt.command, pvat_mode=mode)
    details.update(overrides)
    receipt.completion._resolve(outcome, **details)


async def warm(coordinator, count=15):
    target = pose_delta(ORIGIN, 0.02, 0.3)
    for seq in range(count):
        receipt = await coordinator.propose(target, seq, 0.02)
        finish(receipt)
        assert coordinator.settle() == 'committed'
    assert coordinator.snapshot().linear_velocity[0] > 0
    assert coordinator.snapshot().angular_velocity[2] > 0
    return target


async def test_submission_waiting_and_repeated_settle_do_not_advance_history():
    coordinator, backend, _ = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()
    receipt = await coordinator.propose(target, 20, 0.02)
    assert coordinator.snapshot() == before
    for _ in range(4):
        assert coordinator.settle() == 'waiting'
        with pytest.raises(RuntimeError, match='request_pending'):
            await coordinator.propose(target, 21, 0.02)
    assert len(backend.receipts) == 16
    finish(receipt)
    assert coordinator.settle() == 'committed'
    after = coordinator.snapshot()
    assert after.version == before.version + 1
    assert after.position != before.position
    assert coordinator.settle() == 'idle'
    assert coordinator.snapshot() == after


@pytest.mark.parametrize(('outcome', 'mode'), [
    ('overwritten', None), ('invalidated', None), ('ik_rejected', None),
    ('failed', None), ('cancelled', None), ('handled', None),
    ('sdk_returned', 'catch_up'), ('sdk_returned', 'interpolated_advance'),
])
async def test_nonadvance_freezes_without_reset_or_retry(outcome, mode):
    coordinator, backend, _ = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()
    receipt = await coordinator.propose(target, 20, 0.02)
    finish(receipt, outcome, mode)
    assert coordinator.settle() == 'frozen'
    assert coordinator.snapshot() == before
    assert coordinator.frozen_reason
    with pytest.raises(RuntimeError, match='reanchor_required'):
        await coordinator.propose(target, 21, 0.02)
    assert len(backend.receipts) == 16


@pytest.mark.parametrize('boundary', ['epoch', 'inactive', 'pump_generation', 'handle_invalidated', 'wrong_acceptance', 'wrong_result'])
async def test_late_or_mismatched_success_never_commits(boundary):
    coordinator, backend, authority = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()
    receipt = await coordinator.propose(target, 20, 0.02)
    if boundary == 'epoch':
        authority['epoch'] += 1
    elif boundary == 'inactive':
        authority['active'] = False
    elif boundary == 'pump_generation':
        backend.generation += 1
    elif boundary == 'handle_invalidated':
        receipt.completion._invalidate()
    if boundary == 'wrong_acceptance':
        finish(receipt, accepted_command=CommandRef(20, 0, 999))
    elif boundary == 'wrong_result':
        # Malformed receipt/handle pair: do not trust the receipt's outer ID.
        receipt.completion._command = CommandRef(20, 0, 999)
        finish(receipt)
    else:
        finish(receipt)
    assert coordinator.settle() == 'frozen'
    assert coordinator.snapshot() == before


async def test_reanchor_requires_resolved_request_new_active_epoch_and_explicit_pose():
    coordinator, _, authority = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()
    receipt = await coordinator.propose(target, 20, 0.02)
    authority['epoch'] += 1
    assert coordinator.settle() == 'waiting_invalidated'
    with pytest.raises(RuntimeError, match='request_pending'):
        coordinator.reanchor(ORIGIN)
    finish(receipt)
    assert coordinator.settle() == 'frozen'
    authority['active'] = False
    with pytest.raises(RuntimeError, match='authority_unavailable'):
        coordinator.reanchor(ORIGIN)
    authority['active'] = True
    new_pose = pose_delta(ORIGIN, 0.005, 0.1)
    coordinator.reanchor(new_pose)
    assert coordinator.snapshot().position == new_pose.p
    assert coordinator.snapshot().linear_velocity == (0., 0., 0.)
    assert coordinator.snapshot().angular_velocity == (0., 0., 0.)
    assert coordinator.snapshot().version == before.version + 1
    with pytest.raises(RuntimeError, match='new_epoch_required'):
        coordinator.reanchor(ORIGIN)


async def test_failed_admission_does_not_mutate_history():
    coordinator, backend, _ = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()

    async def fail(*args):
        raise RuntimeError('injected_gate_failure')

    backend.submit_tcp = fail
    with pytest.raises(RuntimeError, match='injected_gate_failure'):
        await coordinator.propose(target, 20, 0.02)
    assert coordinator.snapshot() == before
    assert coordinator.frozen_reason == 'admission_failed'


@pytest.mark.parametrize('end', ['returned', 'revoked', 'cancelled'])
async def test_concurrent_admission_is_exclusive_and_revalidated(end):
    coordinator, backend, authority = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()
    entered, release = asyncio.Event(), asyncio.Event()
    original = backend.submit_tcp

    async def delayed(*args):
        entered.set()
        await release.wait()
        return await original(*args)

    backend.submit_tcp = delayed
    task = asyncio.create_task(coordinator.propose(target, 20, 0.02))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        with pytest.raises(RuntimeError, match='request_pending'):
            await coordinator.propose(target, 21, 0.02)
        authority['epoch'] += int(end == 'revoked')
        with pytest.raises(RuntimeError, match='request_pending'):
            coordinator.reanchor(ORIGIN)
        if end == 'cancelled':
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
            assert coordinator.frozen_reason == 'admission_failed'
            assert len(backend.receipts) == 15
        else:
            release.set()
            receipt = await asyncio.wait_for(task, 1)
            finish(receipt)
            assert coordinator.settle() == ('frozen' if end == 'revoked' else 'committed')
        if end != 'returned':
            assert coordinator.snapshot() == before
    finally:
        release.set()
        if not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


async def test_generation_change_between_requests_forbids_submission_from_old_history():
    coordinator, backend, _ = setup()
    target = await warm(coordinator)
    before = coordinator.snapshot()
    backend.generation += 1
    with pytest.raises(RuntimeError, match='authority_unavailable'):
        await coordinator.propose(target, 20, 0.02)
    assert len(backend.receipts) == 15
    assert coordinator.snapshot() == before


async def test_reanchor_rejects_older_authority_epoch():
    coordinator, _, authority = setup()
    await warm(coordinator)
    before = coordinator.snapshot()
    authority['epoch'] = 0
    with pytest.raises(RuntimeError, match='new_epoch_required'):
        coordinator.reanchor(ORIGIN)
    assert coordinator.snapshot() == before


async def test_candidate_failure_admits_nothing_and_requires_owner_reanchor():
    coordinator, backend, _ = setup()
    coordinator._candidate.limits.workspace_radius = 0.000001
    before = coordinator.snapshot()
    with pytest.raises(RuntimeError, match='generated_step_outside_workspace'):
        await coordinator.propose(pose_delta(ORIGIN, 0.1), 20, 0.02)
    assert backend.receipts == []
    assert coordinator.snapshot() == before
    assert coordinator.frozen_reason == 'candidate_failed'


async def test_rejection_preserves_already_committed_braking_phase():
    coordinator, _, _ = setup()
    await warm(coordinator)
    reverse = pose_delta(ORIGIN, -0.02, -0.3)
    receipt = await coordinator.propose(reverse, 20, 0.02)
    finish(receipt)
    assert coordinator.settle() == 'committed'
    before = coordinator.snapshot()
    assert before.braking
    assert before.linear_velocity[0] > 0
    receipt = await coordinator.propose(reverse, 21, 0.02)
    finish(receipt, 'ik_rejected', None, sdk_outcome='not_started')
    assert coordinator.settle() == 'frozen'
    assert coordinator.snapshot() == before


@pytest.mark.parametrize('outcome', ['advance', 'old_feedback_then_advance', 'catch_up', 'rejected', 'overwritten', 'logging_failed'])
async def test_coordinator_with_real_adapter_and_control_authority(outcome):
    run = Experiment()
    clock = ControlClock(run.clock)
    await run.adapter.connect()
    try:
        control, latest, _, _, _, _ = await active_control(run.adapter, clock)
        coordinator = CandidateCoordinator(run.adapter, limiter(), control.last_target,
            authority=lambda: (control.control_generation, control.mode is TeleopMode.ACTIVE and control._fault is None),
            generation_current=lambda g: run.adapter._pump.is_current(g) and run.adapter.pump_fault is None)
        target = pose_delta(control.last_target, 0.02, 0.3)
        for seq, q in enumerate([0.04, 0.08, 0.12, 0.16, 0.20, 0.22], 3):
            clock.advance_ms(40)
            # Direct coordinator exercise still needs fresh VR authority; do
            # not tick the owner and issue a second automatic target here.
            publish(latest, clock, seq, True, .1, .4)
            await run.adapter.get_state()
            run.client.ik_results = deque([joints(q)])
            receipt = await coordinator.propose(target, seq, 0.02)
            await asyncio.wait_for(receipt.completion.wait(), 1)
            assert coordinator.settle() == 'committed'
        before = coordinator.snapshot()
        if outcome == 'old_feedback_then_advance':
            clock.advance_ms(40)
            publish(latest, clock, 98, True, .1, .4)
            await run.adapter.get_state()
            run.client.ik_results = deque([joints(0.26)])
            old = await run.adapter.submit_tcp(target, 98)
            assert (await asyncio.wait_for(old.completion.wait(), 1)).pvat_mode == 'catch_up'
        clock.advance_ms(40)
        publish(latest, clock, 99, True, .1, .4)
        await run.adapter.get_state()
        run.client.ik_results = deque(joints(q) for q in
            ([0.26] if outcome == 'catch_up' else [0.5, 0.45, 0.4] if outcome == 'rejected' else [0.24]))
        if outcome == 'logging_failed':
            async def fail_log(event, timestamp):
                if event['kind'] == 'pvat_sent':
                    raise RuntimeError('injected_log_failure')
            run.adapter._event_callback = fail_log
        receipt = await coordinator.propose(target, 99, 0.02)
        if outcome == 'old_feedback_then_advance':
            assert receipt.previous_feedback.command == old.command
            assert receipt.previous_feedback.code == 'ik_tracking_lag'
        if outcome == 'overwritten':
            await run.adapter.submit_tcp(target, 100)
        await asyncio.wait_for(receipt.completion.wait(), 1)
        if outcome == 'logging_failed':
            await wait_until(lambda: run.adapter.pump_fault is not None)
        settled = coordinator.settle()
        if outcome in {'advance', 'old_feedback_then_advance'}:
            assert settled == 'committed'
            assert coordinator.snapshot().version == before.version + 1
        else:
            assert settled == 'frozen'
            assert coordinator.snapshot() == before
            assert coordinator.snapshot().linear_velocity[0] > 0
        assert not any(c[0] == 'stop_move' for c in run.client.write_calls)
        # Explicit fake stop and real control reactivation create a NEW epoch;
        # the coordinator itself never emits a stop, reset or Home command.
        await control.disarm()
        clock.advance_ms(20)
        publish(latest, clock, 101, False, 0, 0)
        await control.tick()
        if outcome != 'logging_failed':
            await control.arm()
            publish(latest, clock, 102, True, 0, 0)
            await control.tick()
            coordinator.reanchor(control.last_target)
            assert coordinator.snapshot().linear_velocity == (0., 0., 0.)
    finally:
        run.client.release_ik.set()
        run.client.kin_data['actual_joint_speed'] = [0.] * 6
        await run.adapter.disconnect()
        run.factory.assert_awaited_once()
        assert run.client.network_calls == run.client.discovery_calls == 0
