"""Admission is local queue evidence, never a motion acknowledgement."""

import asyncio
from collections import deque
from dataclasses import FrozenInstanceError, replace

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.robots.command_admission import CommandRef
from app.robots.lebai_pump import PvatPump
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target
from tests.robots.test_lebai_backpressure_experiments import experiment, joints, wait_until


async def test_pump_receipt_is_immutable_and_distinguishes_repeated_ids():
    handled = []

    async def handler(request):
        handled.append(request)

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        first = pump.submit(_target(), 7)
        second = pump.submit(_target(0.301), 7)
        assert first.command == CommandRef(7, 0, 1)
        assert first.replaced_pending is None
        assert second.command == CommandRef(7, 0, 2)
        assert second.replaced_pending == first.command
        with pytest.raises(FrozenInstanceError):
            second.command.submission_id = 99
        with pytest.raises(FrozenInstanceError):
            second.replaced_pending = None
        await wait_until(lambda: len(handled) == 1)
        assert handled[0].reference == second.command
        generation = pump.invalidate()
        third = pump.submit(_target(), 7)
        assert third.command == CommandRef(7, generation, 3)
        assert third.replaced_pending is None
        assert second.command.generation == 0  # Old snapshots do not mutate.
    finally:
        await pump.stop()


async def test_only_pending_request_is_reported_as_replaced():
    started, release = asyncio.Event(), asyncio.Event()

    async def handler(request):
        started.set()
        await release.wait()

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        first = pump.submit(_target(), 1)
        await asyncio.wait_for(started.wait(), 1)
        second = pump.submit(_target(), 2)
        third = pump.submit(_target(), 3)
        assert second.replaced_pending is None
        assert third.replaced_pending == second.command
        assert third.replaced_pending != first.command
    finally:
        release.set()
        await pump.stop()


async def test_explicit_submission_returns_before_ik_and_links_sent_event():
    adapter, client, _ = await _connected_control_adapter(block_ik=True)
    events = []

    async def record(event, timestamp):
        events.append(event)

    adapter._event_callback = record
    try:
        receipt = await adapter.submit_tcp(_target(), 42)
        assert receipt.command.command_id == 42
        assert receipt.previous_feedback is None
        assert not client.write_calls
        await asyncio.wait_for(client.ik_started.wait(), 1)
        assert not client.write_calls
        client.release_ik.set()
        await wait_until(lambda: any(e['kind'] == 'pvat_sent' for e in events))
        sent = next(e for e in events if e['kind'] == 'pvat_sent')
        assert (sent['command_id'], sent['generation'], sent['submission_id']) == (
            42, receipt.command.generation, receipt.command.submission_id)
        assert sent['accepted_target_submission_id'] == receipt.command.submission_id
        await wait_until(lambda: any(e['kind'] == 'sdk_request_lifecycle' for e in events))
        rpc = next(e for e in events if e['kind'] == 'sdk_request_lifecycle')
        assert rpc['submission_id'] == receipt.command.submission_id
        assert rpc['generation'] == receipt.command.generation
    finally:
        client.release_ik.set()
        await adapter.disconnect()
    assert client.network_calls == client.discovery_calls == 0


@pytest.mark.parametrize('outcome', ['catch_up', 'rejected'])
async def test_prior_feedback_has_its_own_identity_and_new_command_can_recover(outcome):
    async with experiment() as run:
        previously_accepted = run.adapter._accepted_target_ref
        run.clock.advance(0.04)
        run.client.ik_results.extend(joints(q) for q in ([0.26] if outcome == 'catch_up' else [0.5, 0.45, 0.4]))
        old = await run.adapter.submit_tcp(_target(0.307), 90)
        if outcome == 'catch_up':
            sent = await asyncio.wait_for(run.sent.get(), 1)
            assert sent['pvat_mode'] == 'catch_up'
            assert sent['accepted_target_submission_id'] == previously_accepted.submission_id
            assert sent['submission_id'] == old.command.submission_id
        else:
            await wait_until(lambda: run.adapter.constraint == 'motion_continuity_boundary')
            assert run.sent.empty()
            rejected = [e for e in run.events if e['kind'] == 'ik_candidate_rejected'
                        and e['command_id'] == 90]
            assert len(rejected) == 3
            assert all(e['submission_id'] == old.command.submission_id
                       and e['generation'] == old.command.generation for e in rejected)
        run.clock.advance(0.04)
        run.client.ik_results.extend([joints(0.24)])
        current = await run.adapter.submit_tcp(_target(0.308), 90)  # Same external id.
        feedback = current.previous_feedback
        assert feedback.command == old.command
        assert feedback.command != current.command
        assert feedback.code == ('ik_tracking_lag' if outcome == 'catch_up' else 'ik_joint_jump')
        assert feedback.outcome == outcome
        event = await asyncio.wait_for(run.sent.get(), 1)
        assert event['pvat_mode'] == 'advance'
        assert event['submission_id'] == current.command.submission_id
        assert event['accepted_target_submission_id'] == current.command.submission_id
        # The frozen receipt retains the earlier feedback after recovery.
        assert current.previous_feedback == feedback
        run.clock.advance(0.04)
        run.client.ik_results.extend([joints(0.24)])
        subsequent = await run.adapter.submit_tcp(_target(0.309), 91)
        assert subsequent.previous_feedback is None
        await asyncio.wait_for(run.sent.get(), 1)


async def test_legacy_entry_still_enqueues_then_raises_prior_error():
    async with experiment() as run:
        await run.step(0.26)
        before = run.adapter._pump._submission_id
        run.clock.advance(0.04)
        run.client.ik_results.extend([joints(0.24)])
        with pytest.raises(BackendCommandError, match='^ik_tracking_lag$'):
            await run.adapter.command_tcp(_target(0.308), 99)
        assert run.adapter._pump._submission_id == before + 1
        event = await asyncio.wait_for(run.sent.get(), 1)
        assert event['command_id'] == 99
        assert event['pvat_mode'] == 'advance'


@pytest.mark.parametrize('entry', ['submit_tcp', 'command_tcp'])
@pytest.mark.parametrize('gate', ['stale', 'preflight', 'stopped', 'fault', 'readonly'])
async def test_hard_gates_do_not_enqueue_or_issue_receipt(entry, gate):
    adapter, client, clock = await _connected_control_adapter()
    try:
        if gate == 'stale':
            clock.advance_ms(1000)
        elif gate == 'preflight':
            adapter._preflight_ready = False
        elif gate == 'stopped':
            await adapter._pump.stop()
        elif gate == 'fault':
            async def failed_stop():
                client.write_calls.append(('stop_move',))
                raise RuntimeError('injected_stop_failure')
            client.stop_move = failed_stop
            with pytest.raises(BackendCommandError):
                await adapter.stop(StopReason.FAULT)
            assert (await adapter.get_state()).fault == 'stop_unverified'
        else:
            adapter.settings = replace(adapter.settings, mode='readonly')
        before = adapter._pump._submission_id
        writes_before = list(client.write_calls)
        with pytest.raises(BackendCommandError):
            await getattr(adapter, entry)(_target(), 1)
        assert adapter._pump._submission_id == before
        assert not adapter._pump.has_pending
        assert client.write_calls == writes_before
    finally:
        await adapter.disconnect()


async def test_stop_and_reconnect_cannot_reuse_old_admission_identity():
    adapter, client, _ = await _connected_control_adapter()
    try:
        old = await adapter.submit_tcp(_target(), 5)
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert not adapter._pump.is_current(old.command.generation)
        await adapter.disconnect()
        await adapter.connect()
        new = await adapter.submit_tcp(_target(), 5)
        assert new.command.generation > old.command.generation
        assert new.command.submission_id > old.command.submission_id
        assert new.replaced_pending is None
    finally:
        await adapter.disconnect()


async def test_feedback_after_verified_stop_keeps_old_generation_explicit():
    async with experiment() as run:
        await run.step(0.26)
        feedback = run.adapter._constraint_feedback
        assert feedback.code == 'ik_tracking_lag'
        await run.adapter.stop(StopReason.GRIP_RELEASED)
        assert (await run.adapter.preflight()).ready
        run.client.ik_results.extend([joints(0.01)])
        receipt = await run.adapter.submit_tcp(_target(), 99)
        assert receipt.previous_feedback == feedback
        assert receipt.previous_feedback.command.generation < receipt.command.generation
        event = await asyncio.wait_for(run.sent.get(), 1)
        assert event['accepted_target_generation'] == receipt.command.generation
        assert event['accepted_target_submission_id'] == receipt.command.submission_id


async def test_invalidated_ik_cannot_publish_new_feedback_or_acceptance():
    adapter, client, _ = await _connected_control_adapter(block_ik=True)
    task = None
    try:
        receipt = await adapter.submit_tcp(_target(), 3)
        await asyncio.wait_for(client.ik_started.wait(), 1)
        task = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
        await wait_until(lambda: not adapter._pump.is_current(receipt.command.generation))
        client.release_ik.set()
        await asyncio.wait_for(task, 1)
        assert adapter._accepted_target_ref is None
        assert adapter._constraint_feedback is None
        assert client.write_calls == [('stop_move',)]
        assert (await adapter.preflight()).ready
        new = await adapter.submit_tcp(_target(), 3)
        assert new.previous_feedback is None
        assert new.command.generation != receipt.command.generation
    finally:
        client.release_ik.set()
        if task is not None and not task.done():
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await adapter.disconnect()


async def test_direct_constraint_without_source_does_not_invent_command_identity():
    adapter, _, _ = await _connected_control_adapter()
    try:
        adapter._note_soft_constraint('ik_invalid', persistent=False)
        receipt = await adapter.submit_tcp(_target(), 99)
        assert receipt.previous_feedback.code == 'ik_unreachable'
        assert receipt.previous_feedback.command is None
        assert receipt.previous_feedback.outcome == 'rejected'
    finally:
        await adapter.disconnect()


async def test_interpolated_acceptance_is_not_reported_as_total_rejection():
    adapter, client, _ = await _connected_control_adapter(backend_label='LEBAI_FAKE')
    events = []

    async def record(event, timestamp):
        events.append(event)

    adapter._event_callback = record
    try:
        client.ik_results = deque([joints(0.0032), None, joints(0.0040)])
        await adapter.submit_tcp(_target(0.30), 1)
        await wait_until(lambda: any(e['kind'] == 'pvat_sent' for e in events))
        partial = await adapter.submit_tcp(_target(0.34), 2)
        await wait_until(lambda: len([e for e in events if e['kind'] == 'pvat_sent']) == 2)
        event = [e for e in events if e['kind'] == 'pvat_sent'][-1]
        assert event['recovery_fraction'] == pytest.approx(0.75)
        assert event['accepted_target_submission_id'] == partial.command.submission_id
        receipt = await adapter.submit_tcp(_target(0.33), 3)
        assert receipt.previous_feedback.command == partial.command
        assert receipt.previous_feedback.outcome == 'interpolated_advance'
        assert receipt.previous_feedback.code == 'ik_unreachable'
    finally:
        await adapter.disconnect()
