"""Bounded offline pursuit; observed convergence never resumes old history."""

import asyncio
from collections import deque
from contextlib import asynccontextmanager

import pytest

from app.schemas.messages import TeleopMode
from tests.control.catch_up_probe import CatchUpProbe
from tests.control.candidate_commit_probe import CandidateCoordinator
from tests.control.elapsed_step_probe import publish
from tests.control.test_braking_approach import pose_delta
from tests.control.test_braking_supervision import active_control
from tests.robots.retarget_backpressure_probe import ControlClock
from tests.robots.test_explicit_catch_up import result
from tests.robots.test_lebai_backpressure_experiments import Experiment, joints, wait_until
from tests.robots.test_vr_profile_experiments import limiter


@asynccontextmanager
async def pursuing():
    run = Experiment()
    clock = ControlClock(run.clock)
    await run.adapter.connect()
    try:
        control, latest, *_ = await active_control(run.adapter, clock)
        epoch = control.control_generation

        def input_valid():
            frame = latest.snapshot()
            return (control.mode is TeleopMode.ACTIVE
                    and control._control_generation_is_current(control.control_generation)
                    and frame is not None and frame.frame.right.grip
                    and frame.frame.tracking_valid and frame.frame.visibility == 'visible'
                    and 0 <= clock.now_ns() - frame.received_ns < 100_000_000)

        current = lambda g: run.adapter._pump.is_current(g) and run.adapter.pump_fault is None
        candidate = CandidateCoordinator(run.adapter, limiter(), control.last_target,
            authority=lambda: (control.control_generation, input_valid()), generation_current=current)
        target = pose_delta(control.last_target, .02, .3)
        for seq, q in enumerate([.04, .08, .12, .16, .20, .22, .26], 3):
            clock.advance_ms(40)
            publish(latest, clock, seq, True, .1, 0)
            await run.adapter.get_state()
            run.client.ik_results = deque([joints(q)])
            receipt = await candidate.propose(target, seq, .02)
            await result(receipt)
            assert candidate.settle() == ('frozen' if q == .26 else 'committed')
        probe = CatchUpProbe(run.adapter, run.adapter._accepted_target_ref,
            run.adapter._last_accepted_solution_q, clock=clock.now_ns,
            authorized=lambda: input_valid() and control.control_generation == epoch,
            generation_current=current)
        yield run, control, latest, clock, candidate, probe
    finally:
        run.client.release_ik.set()
        run.client.kin_data['actual_joint_speed'] = [0.] * 6
        await run.adapter.disconnect()
        run.factory.assert_awaited_once()
        assert run.client.network_calls == run.client.discovery_calls == 0


async def sample(run, latest, clock, seq, q=.22, speed=0):
    clock.advance_ms(40)
    publish(latest, clock, seq, True, .1, 0)
    run.client.kin_data['actual_joint_pose'] = joints(q)
    run.client.kin_data['actual_joint_speed'] = [speed] + [0.] * 5
    await run.adapter.get_state()
    return run.adapter._snapshot


async def test_pursuit_completion_and_idle_do_not_thaw_candidate_or_prove_convergence():
    async with pursuing() as (run, control, latest, clock, candidate, probe):
        before = candidate.snapshot()
        assert before.linear_velocity[0] > 0
        for seq in range(20, 25):
            snapshot = await sample(run, latest, clock, seq, q=0)
            probe.observe(snapshot)  # New feedback releases the preceding wait.
            receipt = await probe.submit(seq)
            await result(receipt)
            assert probe.settle() == 'pursuing'
            assert probe.observe(snapshot) == 'pursuing'
            assert candidate.snapshot() == before
        for seq in range(25, 28):
            snapshot = await sample(run, latest, clock, seq)
            assert probe.observe(snapshot) == ('observed_settled' if seq == 27 else 'pursuing')
        assert candidate.snapshot() == before
        assert candidate.frozen_reason == 'catch_up'
        with pytest.raises(RuntimeError, match='observed_settled'):
            await probe.submit(30)
        with pytest.raises(RuntimeError, match='reanchor_required'):
            await candidate.propose(pose_delta(control_pose(candidate), .01), 31, .02)
        await control.disarm()
        publish(latest, clock, 32, False, 0, 0)
        await control.tick()
        await control.arm()
        publish(latest, clock, 33, True, 0, 0)
        await control.tick()
        candidate.reanchor(control.last_target)
        assert candidate.snapshot().linear_velocity == (0., 0., 0.)
        assert candidate.snapshot().angular_velocity == (0., 0., 0.)
        assert not candidate.snapshot().braking
        assert probe.settle() == 'authority_revoked'


def control_pose(candidate):
    return candidate._pose.model_copy(deep=True)


@pytest.mark.parametrize('bad', ['duplicate', 'stale', 'moving', 'far', 'nonfinite'])
async def test_exit_requires_distinct_fresh_finite_near_stationary_samples(bad):
    async with pursuing() as (run, _, latest, clock, _, probe):
        receipt = await probe.submit(20)
        await result(receipt)
        probe.settle()
        snapshot = await sample(run, latest, clock, 21)
        assert probe.observe(snapshot) == 'pursuing'
        if bad == 'duplicate':
            for _ in range(5):
                assert probe.observe(snapshot) == 'pursuing'
        elif bad == 'stale':
            clock.advance_ms(50)
            publish(latest, clock, 22, True, .1, 0)
            assert probe.observe(snapshot) == 'feedback_expired'
        else:
            snapshot = await sample(run, latest, clock, 22,
                q=.21 if bad == 'far' else .22, speed=.01 if bad == 'moving' else 0)
            if bad == 'nonfinite':
                from dataclasses import replace
                snapshot = replace(snapshot, actual_q=(float('nan'), *snapshot.actual_q[1:]))
            assert probe.observe(snapshot) == 'pursuing'
        # Invalid observations reset the streak; no automatic resume.
        assert probe.good_samples == 0


async def test_no_feedback_progress_expires_and_owner_stops_without_resetting_history():
    async with pursuing() as (run, control, latest, clock, candidate, probe):
        before = candidate.snapshot()
        clock.advance_ms(500)
        publish(latest, clock, 20, True, .1, 0)
        with pytest.raises(RuntimeError, match='budget_expired'):
            await probe.submit(20)
        await control.disarm()
        assert candidate.snapshot() == before
        assert any(c[0] == 'stop_move' for c in run.client.write_calls)
        assert not any(c[0] == 'stop_sys' for c in run.client.write_calls)


@pytest.mark.parametrize('boundary', ['release', 'stale', 'fault', 'stop_error', 'stop_late'])
@pytest.mark.parametrize('phase', ['snapshot', 'pvat'])
async def test_real_owner_stop_preempts_inflight_and_pending_pursuit(boundary, phase):
    async with pursuing() as (run, control, latest, clock, candidate, probe):
        before = candidate.snapshot()
        entered, release = asyncio.Event(), asyncio.Event()
        read = run.client.get_kin_data
        move = run.client.move_pvat

        async def blocked_read():
            entered.set()
            await release.wait()
            return await read()

        async def blocked_move(*args):
            await move(*args)
            entered.set()
            await release.wait()

        if phase == 'snapshot':
            run.client.get_kin_data = blocked_read
        else:
            run.client.move_pvat = blocked_move
        task = None
        try:
            receipt = await probe.submit(20)
            await asyncio.wait_for(entered.wait(), 1)
            pending = await run.adapter.submit_catch_up(probe.accepted, 21)
            writes = list(run.client.write_calls)
            with pytest.raises(RuntimeError, match='request_pending'):
                await probe.submit(22)
            if boundary == 'stale':
                clock.advance_ms(100)
            elif boundary != 'fault':
                publish(latest, clock, 23, False, .1, 0)
            if boundary in {'stop_error', 'stop_late'}:
                async def failed_stop():
                    run.client.write_calls.append(('stop_move',))
                    if boundary == 'stop_error':
                        raise RuntimeError('injected_stop_error')
                    clock.advance_ms(609)
                run.client.stop_move = failed_stop
            if boundary != 'fault':
                assert probe.settle() == 'authority_revoked'
            task = asyncio.create_task(control._enter_fault('target_braking_infeasible')
                if boundary == 'fault' else control.tick())
            await wait_until(lambda: run.adapter._stop_transaction is not None)
            assert (await result(pending)).invalidated
            run.client.get_kin_data = read
            release.set()
            await asyncio.wait_for(task, 2)
            done = await result(receipt)
            assert done.invalidated
            assert done.sdk_outcome == ('returned' if phase == 'pvat' else 'not_started')
            assert probe.settle() == 'authority_revoked'
            assert candidate.snapshot() == before
            new_writes = run.client.write_calls[len(writes):]
            assert new_writes and all(c[0] == 'stop_move' for c in new_writes)
            assert not run.adapter._pump.has_pending
            with pytest.raises(RuntimeError, match='authority_revoked'):
                await probe.submit(24)
            if boundary in {'stop_error', 'stop_late'}:
                assert (await control.state_message()).fault == 'stop_unverified'
                assert not (await control.reset_fault()).accepted
        finally:
            release.set()
            run.client.get_kin_data = read
            run.client.move_pvat = move
            if task is not None:
                await asyncio.gather(task, return_exceptions=True)


async def test_overwritten_pursuit_is_terminal_and_does_not_resume_candidates():
    async with pursuing() as (run, _, _, _, candidate, probe):
        before = candidate.snapshot()
        receipt = await probe.submit(20)
        replacement = await run.adapter.submit_catch_up(probe.accepted, 21)
        assert (await result(receipt)).outcome == 'overwritten'
        assert probe.settle() == 'dispatch_failed'
        assert (await result(replacement)).pvat_mode == 'catch_up'
        assert candidate.snapshot() == before
        with pytest.raises(RuntimeError, match='dispatch_failed'):
            await probe.submit(22)
