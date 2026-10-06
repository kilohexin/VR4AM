"""Passive acceptance/observation views; injected fake clients only."""

import asyncio
from collections import deque
from dataclasses import FrozenInstanceError

import pytest

from app.robots.base import BackendCommandError, StopReason
from tests.robots.test_explicit_catch_up import result
from tests.robots.test_lebai_backpressure_experiments import (
    Experiment, experiment, joints, target,
)


def test_cold_view_does_not_connect_or_create_work():
    run = Experiment()
    view = run.adapter.read_motion_view()
    assert not view.client_attached and not view.pump_running
    assert view.accepted is None and view.sample is None
    assert not view.is_current(view.generation)
    run.factory.assert_not_called()
    assert run.client.read_calls == run.client.write_calls == []


async def test_view_is_detached_immutable_and_does_not_refresh_or_write():
    async with experiment() as run:
        view = run.adapter.read_motion_view()
        assert view.is_current(view.generation)
        assert view.accepted.command == run.adapter._accepted_target_ref
        assert view.accepted.solution_q == tuple(joints(.22))
        assert view.accepted.tcp == run.adapter._last_sent_tcp
        assert view.sample.generation == view.generation
        assert view.sample.actual_q == tuple(joints(0))
        assert view.sample.read_purpose == 'motion'
        assert view.sample.captured_ns <= view.sample.read_completed_ns <= view.observed_ns
        with pytest.raises(FrozenInstanceError):
            view.generation = -1
        with pytest.raises(FrozenInstanceError):
            view.accepted.solution_q = ()
        with pytest.raises(FrozenInstanceError):
            view.sample.captured_ns = -1
        pose = view.accepted.tcp
        pose.p = (99., 99., 99.)
        assert view.accepted.tcp != pose
        reads, writes, events = list(run.client.read_calls), list(run.client.write_calls), len(run.events)
        run.clock.advance(.2)
        for _ in range(10):
            later = run.adapter.read_motion_view()
            assert later.sample == view.sample
            assert later.accepted == view.accepted
            assert later.observed_ns - later.sample.captured_ns >= 200_000_000
        assert run.client.read_calls == reads and run.client.write_calls == writes
        assert len(run.events) == events
        assert view.observed_ns < later.observed_ns


async def test_pending_sdk_does_not_advance_acceptance_and_catch_up_pins_it():
    async with experiment() as run:
        before = run.adapter.read_motion_view()
        original = run.client.move_pvat
        started, release = asyncio.Event(), asyncio.Event()

        async def blocked(*args, **kwargs):
            started.set()
            await release.wait()
            return await original(*args, **kwargs)

        run.client.move_pvat = blocked
        run.clock.advance(.04)
        await run.adapter.get_state()
        run.client.ik_results = deque([joints(.24)])
        receipt = await run.adapter.submit_tcp(target(.307), 70)
        try:
            await asyncio.wait_for(started.wait(), 1)
            assert run.adapter.read_motion_view().accepted == before.accepted
        finally:
            release.set()
        done = await result(receipt)
        accepted = run.adapter.read_motion_view().accepted
        assert accepted.command == done.accepted_command == receipt.command
        assert accepted != before.accepted
        run.clock.advance(.04)
        await run.adapter.get_state()
        done = await result(await run.adapter.submit_catch_up(accepted.command, 71))
        assert done.pvat_mode == 'catch_up'
        assert run.adapter.read_motion_view().accepted == accepted


@pytest.mark.parametrize('boundary', ['reading', 'queued'])
async def test_generation_change_during_read_never_relabels_observation(boundary):
    run = Experiment()
    await run.adapter.connect()
    before = run.adapter.read_motion_view()
    original = run.client.get_kin_data
    started, release = asyncio.Event(), asyncio.Event()

    async def blocked():
        started.set()
        await release.wait()
        return await original()

    run.client.get_kin_data = blocked
    if boundary == 'queued':
        await run.adapter._sdk_lock.acquire()
    pending = asyncio.create_task(run.adapter.get_state())
    try:
        if boundary == 'reading':
            await asyncio.wait_for(started.wait(), 1)
        else:
            await asyncio.sleep(0)
        run.adapter._pump.invalidate()
        if boundary == 'queued':
            run.adapter._sdk_lock.release()
        release.set()
        await pending
        after = run.adapter.read_motion_view()
        assert after.generation != before.generation
        assert not after.is_current(before.generation)
        assert after.sample.generation == before.generation
        assert after.sample.generation != after.generation
        run.clock.advance(.01)
        await run.adapter.get_state()
        fresh = run.adapter.read_motion_view()
        assert fresh.sample.generation == fresh.generation
        assert fresh.sample.captured_ns > before.sample.captured_ns
    finally:
        release.set()
        await run.adapter.disconnect()


async def test_stop_disconnect_reconnect_revoke_views_without_mutating_old_copy():
    run = Experiment()
    await run.adapter.connect()
    try:
        await run.warm_up()
        before = run.adapter.read_motion_view()
        await run.adapter.stop(StopReason.GRIP_RELEASED)
        stopped = run.adapter.read_motion_view()
        assert stopped.generation != before.generation and stopped.accepted is None
        assert not stopped.is_current(before.generation)
        assert not stopped.preflight_ready
        await run.adapter.disconnect()
        detached = run.adapter.read_motion_view()
        assert not detached.client_attached and detached.sample is None and detached.accepted is None
        # Same injected object on purpose: source generations must still differ.
        await run.adapter.connect()
        again = run.adapter.read_motion_view()
        assert again.sample.generation == again.generation != before.generation
        assert again.accepted is None
        assert before.accepted is not None and before.sample is not None
        assert run.factory.await_count == 2
    finally:
        await run.adapter.disconnect()


@pytest.mark.parametrize('fault', ['pump', 'latched'])
async def test_fault_view_readable_without_clearing_gates(fault):
    run = Experiment()
    await run.adapter.connect()
    try:
        if fault == 'pump':
            run.adapter._pump._fault = BackendCommandError('pvat_pump_failed')
        else:
            run.adapter._latched_fault = 'stop_unverified'
            run.adapter._preflight_ready = False
        reads = list(run.client.read_calls)
        view = run.adapter.read_motion_view()
        assert (view.pump_fault if fault == 'pump' else view.latched_fault) == (
            'pvat_pump_failed' if fault == 'pump' else 'stop_unverified')
        if fault == 'pump':
            assert not view.is_current(view.generation)
        assert run.client.read_calls == reads
        with pytest.raises(BackendCommandError):
            await run.adapter.submit_tcp(target(), 100)
    finally:
        await run.adapter.disconnect()


async def test_late_read_after_disconnect_cannot_repopulate_public_cache():
    run = Experiment()
    await run.adapter.connect()
    original = run.client.get_kin_data
    started, release = asyncio.Event(), asyncio.Event()

    async def blocked():
        started.set()
        await release.wait()
        return await original()

    run.client.get_kin_data = blocked
    pending = asyncio.create_task(run.adapter.get_state())
    try:
        await asyncio.wait_for(started.wait(), 1)
        # No accepted motion: local teardown can finish while this fake read waits.
        await run.adapter.disconnect()
        release.set()
        await pending
        view = run.adapter.read_motion_view()
        assert not view.client_attached and view.sample is None
    finally:
        release.set()
        await run.adapter.disconnect()


async def test_failed_connection_recording_clears_public_sample():
    run = Experiment()

    async def fail_record(*args):
        raise RuntimeError('injected_recording_failure')

    run.adapter._event_callback = fail_record
    with pytest.raises(BackendCommandError, match='recording_unavailable'):
        await run.adapter.connect()
    view = run.adapter.read_motion_view()
    assert not view.client_attached and not view.pump_running
    assert view.sample is None and view.accepted is None


@pytest.mark.parametrize('outcome', ['returned', 'failed', 'late'])
async def test_view_during_and_after_stop_preserves_revocation_and_latch(outcome):
    async with experiment() as run:
        before = run.adapter.read_motion_view()
        original = run.client.stop_move
        started, release = asyncio.Event(), asyncio.Event()

        async def blocked():
            started.set()
            await release.wait()
            if outcome == 'failed':
                raise RuntimeError('injected_stop_failure')
            if outcome == 'late':
                run.clock.advance(.609)
            return await original()

        run.client.stop_move = blocked
        pending = asyncio.create_task(run.adapter.stop(StopReason.GRIP_RELEASED))
        try:
            await asyncio.wait_for(started.wait(), 1)
            reads = list(run.client.read_calls)
            current = run.adapter.read_motion_view()
            assert current.stop_in_progress and current.accepted is None
            assert not current.preflight_ready and not current.is_current(before.generation)
            assert run.client.read_calls == reads
        finally:
            release.set()
        if outcome == 'returned':
            await pending
        else:
            with pytest.raises(BackendCommandError):
                await pending
        after = run.adapter.read_motion_view()
        assert not after.stop_in_progress
        assert after.latched_fault == (None if outcome == 'returned' else 'stop_unverified')
        assert after.accepted is None and not after.preflight_ready
        assert not any(c[0] == 'stop_sys' for c in run.client.write_calls)
