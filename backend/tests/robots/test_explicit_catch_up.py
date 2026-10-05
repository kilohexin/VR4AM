"""Explicit pursuit of an accepted target, using injected SDKs only."""

import asyncio
from collections import deque
from dataclasses import replace

import pytest

from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target
from tests.robots.test_lebai_backpressure_experiments import experiment, joints, wait_until


async def result(receipt):
    return await asyncio.wait_for(receipt.completion.wait(), 1)


async def test_explicit_pursuit_preserves_acceptance_and_never_runs_ik():
    async with experiment() as run:
        accepted = run.adapter._accepted_target_ref
        target = run.adapter._last_sent_tcp.model_copy(deep=True)
        # No possible IK result: explicit pursuit must use the pinned solution.
        async def forbidden_ik(*args):
            raise AssertionError('pursuit_must_not_generate_candidate')
        run.client.kinematics_inverse = forbidden_ik
        for seq in range(10, 15):
            run.clock.advance(.04)
            await run.adapter.get_state()
            receipt = await run.adapter.submit_catch_up(accepted, seq)
            done = await result(receipt)
            assert done.outcome == 'sdk_returned'
            assert done.pvat_mode == 'catch_up'
            assert done.accepted_command == accepted != receipt.command
            assert run.adapter._accepted_target_ref == accepted
            assert run.adapter._last_sent_tcp == target
            assert run.adapter._last_accepted_solution_q == tuple(joints(.22))
        await wait_until(lambda: len([e for e in run.events if e['kind'] == 'pvat_sent']) == 11)
        assert all(e['request_kind'] == 'catch_up' for e in run.events
                   if e['kind'] == 'pvat_sent' and e['command_id'] >= 10)


@pytest.mark.parametrize('invalid', ['none', 'command', 'generation', 'submission'])
async def test_missing_or_wrong_accepted_reference_enqueues_nothing(invalid):
    async with experiment() as run:
        accepted = run.adapter._accepted_target_ref
        ref = None if invalid == 'none' else replace(accepted, **{
            {'command': 'command_id', 'generation': 'generation', 'submission': 'submission_id'}[invalid]: 999})
        before = run.adapter._pump._submission_id
        with pytest.raises(BackendCommandError, match='catch_up_target_changed'):
            await run.adapter.submit_catch_up(ref, 10)
        assert run.adapter._pump._submission_id == before
        assert not run.adapter._pump.has_pending


async def test_no_accepted_target_and_stopped_epoch_cannot_be_pursued():
    adapter, _, _ = await _connected_control_adapter()
    try:
        with pytest.raises(BackendCommandError, match='catch_up_target_changed'):
            await adapter.submit_catch_up(None, 10)
        receipt = await adapter.submit_tcp(_target(), 11)
        await result(receipt)
        await adapter.stop(StopReason.GRIP_RELEASED)
        with pytest.raises(BackendCommandError, match='preflight_not_ready'):
            await adapter.submit_catch_up(receipt.command, 12)
    finally:
        await adapter.disconnect()


async def test_pending_pursuit_is_invalidated_if_inflight_candidate_changes_acceptance():
    async with experiment() as run:
        accepted = run.adapter._accepted_target_ref
        run.clock.advance(.04)
        run.client.ik_results = deque([joints(.24)])
        run.client.block_ik = True
        run.client.ik_started.clear()
        advancing = await run.adapter.submit_tcp(_target(.307), 10)
        await asyncio.wait_for(run.client.ik_started.wait(), 1)
        pursuit = await run.adapter.submit_catch_up(accepted, 11)
        run.client.release_ik.set()
        assert (await result(advancing)).pvat_mode == 'advance'
        done = await result(pursuit)
        assert done.outcome == 'invalidated'
        assert done.invalidated
        assert done.sdk_outcome == 'not_started'
        assert done.error == 'catch_up_target_changed'
        assert run.adapter._accepted_target_ref == advancing.command
        assert run.adapter.pump_fault is None
        assert len([c for c in run.client.write_calls if c[0] == 'move_pvat']) == 7


async def test_pursuit_keeps_tracking_error_limit_and_latches_failure():
    async with experiment() as run:
        run.client.kin_data['actual_joint_pose'] = joints(-.04)
        receipt = await run.adapter.submit_catch_up(run.adapter._accepted_target_ref, 10)
        done = await result(receipt)
        assert done.outcome == 'failed'
        assert done.error == 'ik_tracking_diverged'
        assert done.sdk_outcome == 'not_started'
        await wait_until(lambda: run.adapter.pump_fault is not None)
        assert len([c for c in run.client.write_calls if c[0] == 'move_pvat']) == 6


async def test_pursuit_does_not_bypass_stale_snapshot_gate():
    async with experiment() as run:
        accepted = run.adapter._accepted_target_ref
        run.clock.advance(1)
        with pytest.raises(BackendCommandError, match='robot_state_stale'):
            await run.adapter.submit_catch_up(accepted, 10)
        assert not run.adapter._pump.has_pending
