"""Characterize generated/admitted/sent history using ONLY injected SDK clients."""

import asyncio
import json

import numpy as np
import pytest

from app.schemas.messages import TeleopMode
from tests.control.elapsed_step_probe import publish
from tests.robots.retarget_backpressure_probe import retarget_run
from tests.robots.test_lebai_backpressure_experiments import wait_until


def emit(run, case, **extra):
    print('HISTORY_RESULT ' + json.dumps(dict(
        channel=run.channel, case=case, current_id=run.seq,
        accepted_id=run.adapter._accepted_target_command_id,
        control_matches_adapter_target=run.control.last_target == run.adapter._last_sent_tcp,
        current_candidate_speed=float(np.linalg.norm(run.candidate.velocity(run.channel))),
        candidate_braking=run.candidate.braking,
        mode=run.control.mode.value, **extra,
    )))


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_queued_constraint_feedback_resets_history_before_current_result(channel):
    async with retarget_run(channel) as run:
        await run.warm()
        accepted_id = run.adapter._accepted_target_command_id
        accepted_pose = run.adapter._last_sent_tcp
        first = await run.step([0.26])
        assert first['pvat_mode'] == 'catch_up'
        assert first['accepted_target_command_id'] == accepted_id
        # First catch-up was reported AFTER this control tick, so control already
        # advanced its generated target while the adapter retained the old one.
        assert run.control.last_target != accepted_pose
        previous_control_target = run.control.last_target
        previous_velocity = run.candidate.velocity(channel).copy()
        assert np.linalg.norm(previous_velocity) > 0
        second = await run.step([0.26])
        assert second['command_id'] == run.seq
        assert second['pvat_mode'] == 'catch_up'
        assert second['accepted_target_command_id'] == accepted_id
        assert second['requested_tcp'] == run.candidate.generated[-1]['pose'].model_dump()
        assert run.control.last_target == previous_control_target
        assert np.linalg.norm(run.candidate.velocity(channel)) == 0
        assert run.control.mode is TeleopMode.ACTIVE
        assert not [c for c in run.client.write_calls if c[0] == 'stop_move']
        assert run.candidate.resets[-1]['speed'] > 0
        emit(run, 'catch_up_reset', generated_id=run.seq,
            before_reset_speed=float(np.linalg.norm(previous_velocity)),
            control_target_advanced=False, pvat_mode=second['pvat_mode'])


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_recovery_command_is_sent_despite_old_error_and_zero_history(channel):
    async with retarget_run(channel) as run:
        await run.warm()
        await run.step([0.26])
        previous = run.control.last_target
        recovered = await run.step([0.24])
        assert recovered['pvat_mode'] == 'advance'
        assert recovered['accepted_target_command_id'] == run.seq
        assert run.adapter._last_sent_tcp != previous
        assert run.control.last_target == previous
        assert np.linalg.norm(run.candidate.velocity(channel)) == 0
        assert run.adapter.constraint is None
        assert run.control.mode is TeleopMode.ACTIVE
        emit(run, 'recovered_but_local_history_frozen', pvat_mode=recovered['pvat_mode'])
        # A later call can again advance the local history; there was no stop
        # or physical standstill between the zero and restarted arithmetic state.
        resumed = await run.step([0.24])
        assert resumed['accepted_target_command_id'] == run.seq
        assert run.control.last_target == run.adapter._last_sent_tcp
        assert np.linalg.norm(run.candidate.velocity(channel)) > 0


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_true_ik_rejection_does_not_roll_back_generated_target(channel):
    async with retarget_run(channel) as run:
        await run.warm()
        accepted_id = run.adapter._accepted_target_command_id
        accepted_pose = run.adapter._last_sent_tcp
        await run.submit([0.5, 0.45, 0.4])
        await wait_until(lambda: run.adapter.constraint == 'motion_continuity_boundary')
        assert run.sent.empty()
        assert run.adapter._accepted_target_command_id == accepted_id
        assert run.adapter._last_sent_tcp == accepted_pose
        assert run.control.last_target != accepted_pose
        assert np.linalg.norm(run.candidate.velocity(channel)) > 0
        rejections = [e for e in run.events if e.get('kind') == 'ik_candidate_rejected'
                      and e.get('command_id') == run.seq]
        assert [e['reason'] for e in rejections] == ['ik_joint_jump'] * 3
        current = await run.step([0.24])
        assert current['pvat_mode'] == 'advance'
        assert current['accepted_target_command_id'] == run.seq
        assert np.linalg.norm(run.candidate.velocity(channel)) == 0
        emit(run, 'true_rejection_then_recovery', rejected_id=run.seq - 1,
             rejection_reasons=[e['reason'] for e in rejections])


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_pending_overwrite_advances_unsent_history(channel):
    async with retarget_run(channel) as run:
        await run.warm()
        run.client.block_ik = True
        run.client.ik_started.clear()
        await run.submit([0.22])
        inflight = run.seq
        await asyncio.wait_for(run.client.ik_started.wait(), 1)
        for _ in range(3):
            # Can't read a snapshot while the fake IK owns the SDK lock. These
            # 60 ms stay within the existing cached snapshot freshness budget.
            await run.submit([0.22], refresh=False, advance_ms=20)
        newest = run.seq
        generated = run.control.last_target
        assert run.adapter._pump._pending.command_id == newest
        assert run.adapter._accepted_target_command_id == inflight - 1
        assert np.linalg.norm(run.candidate.velocity(channel)) > 0
        run.client.release_ik.set()
        first = await asyncio.wait_for(run.sent.get(), 1)
        last = await asyncio.wait_for(run.sent.get(), 1)
        assert [first['command_id'], last['command_id']] == [inflight, newest]
        assert last['requested_tcp'] == generated.model_dump()
        assert last['accepted_target_command_id'] == newest
        assert run.sent.empty()
        assert [r['seq'] for r in run.candidate.generated[-4:]] == list(range(inflight, newest + 1))
        emit(run, 'overwritten_pending', generated_ids=list(range(inflight, newest + 1)),
             sent_ids=[inflight, newest])


@pytest.mark.parametrize('channel', ['linear', 'angular'])
async def test_old_feedback_clears_active_braking_phase_without_a_stop(channel):
    async with retarget_run(channel) as run:
        await run.warm()
        await run.step([0.26])
        await run.step([0.26], reverse=True)
        generated = run.candidate.generated[-1]
        assert generated['braking']
        assert np.linalg.norm(generated['velocity']) > 0
        assert run.candidate.resets[-1]['braking']
        assert not run.candidate.braking
        assert np.linalg.norm(run.candidate.velocity(channel)) == 0
        assert run.control.mode is TeleopMode.ACTIVE
        assert not [c for c in run.client.write_calls if c[0] == 'stop_move']
        emit(run, 'braking_phase_cleared_by_old_feedback',
             generated_braking=True, generated_speed=float(np.linalg.norm(generated['velocity'])))


@pytest.mark.parametrize('channel', ['linear', 'angular'])
@pytest.mark.parametrize('failure', [False, True])
async def test_release_during_backpressure_revokes_inflight_and_pending(channel, failure):
    async with retarget_run(channel) as run:
        await run.warm()
        await run.step([0.26])
        run.client.block_ik = True
        run.client.ik_started.clear()
        await run.submit([0.26])
        await asyncio.wait_for(run.client.ik_started.wait(), 1)
        await run.submit([0.26], refresh=False, advance_ms=20)
        assert run.adapter._pump.has_pending
        before = list(run.client.write_calls)
        if failure:
            async def failed_stop():
                run.client.write_calls.append(('stop_move',))
                raise RuntimeError('injected_stop_failure')
            run.client.stop_move = failed_stop
        run.control_clock.advance_ms(20)
        run.seq += 1
        publish(run.latest, run.clock, run.seq, False, 0, 0)
        generation = run.adapter._pump._generation
        task = asyncio.create_task(run.control.tick())
        try:
            await wait_until(lambda: not run.adapter._pump.is_current(generation))
            run.client.release_ik.set()
            await asyncio.wait_for(task, 1)
            assert run.client.write_calls[len(before):] == [('stop_move',)]
            assert not run.adapter._pump.has_pending
            assert run.sent.empty()
            assert run.control.mode is (TeleopMode.FAULT if failure else TeleopMode.HOLD)
            if failure:
                assert (await run.control.state_message()).fault == 'stop_unverified'
                with pytest.raises(RuntimeError, match='arm_blocked_by_fault'):
                    await run.control.arm()
            else:
                run.control_clock.advance_ms(20)
                run.seq += 1
                publish(run.latest, run.clock, run.seq, True, 0, 0)
                await run.control.tick()
                assert run.control.mode is TeleopMode.ACTIVE
                assert not run.candidate.braking
                assert np.linalg.norm(run.candidate.velocity(channel)) == 0
            emit(run, 'release_failed' if failure else 'release_and_reanchor',
                 new_writes=[c[0] for c in run.client.write_calls[len(before):]])
        finally:
            run.client.release_ik.set()
            if not task.done():
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
