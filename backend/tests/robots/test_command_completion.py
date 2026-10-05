"""Per-request software outcomes, using injected handlers/SDK only."""

import asyncio
import gc
import weakref
from dataclasses import FrozenInstanceError

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_pump import PvatPump
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target
from tests.robots.test_lebai_backpressure_experiments import experiment, joints, wait_until


async def test_overwritten_pending_has_immediate_final_result():
    async def handler(request):
        pass

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        first = pump.submit(_target(), 1)
        second = pump.submit(_target(), 1)
        result = first.completion.result
        assert result.outcome == 'overwritten'
        assert result.command == first.command
        assert result.sdk_outcome == 'not_started'
        with pytest.raises(FrozenInstanceError):
            result.outcome = 'handled'
        with pytest.raises(AttributeError):
            first.completion.command = second.command
        handled = await asyncio.wait_for(second.completion.wait(), 1)
        assert handled.outcome == 'handled'
        assert handled.sdk_outcome == 'unknown'  # A generic handler supplies no SDK evidence.
        assert handled.command != result.command
    finally:
        await pump.stop()


async def test_invalidate_completes_pending_but_waits_for_inflight_handler():
    entered, release = asyncio.Event(), asyncio.Event()

    async def handler(request):
        entered.set()
        await release.wait()

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        active = pump.submit(_target(), 1)
        await asyncio.wait_for(entered.wait(), 1)
        pending = pump.submit(_target(), 2)
        pump.invalidate()
        assert pending.completion.result.outcome == 'invalidated'
        assert active.completion.invalidated
        assert active.completion.result is None
        release.set()
        result = await asyncio.wait_for(active.completion.wait(), 1)
        assert result.outcome == 'invalidated'
        assert result.invalidated
    finally:
        release.set()
        await pump.stop()


@pytest.mark.parametrize('failure', ['backend', 'unexpected', 'cancel'])
async def test_handler_exit_resolves_active_and_pending(failure):
    entered, release = asyncio.Event(), asyncio.Event()

    async def handler(request):
        entered.set()
        await release.wait()
        if failure == 'backend':
            raise BackendCommandError('robot_state_stale')
        raise RuntimeError('private injected error')

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        active = pump.submit(_target(), 1)
        await asyncio.wait_for(entered.wait(), 1)
        pending = pump.submit(_target(), 2)
        if failure == 'cancel':
            await pump.stop()
        else:
            release.set()
        result = await asyncio.wait_for(active.completion.wait(), 1)
        assert result.outcome == ('cancelled' if failure == 'cancel' else 'failed')
        assert result.error == (None if failure == 'cancel' else
                                'robot_state_stale' if failure == 'backend' else 'pvat_pump_failed')
        assert (await pending.completion.wait()).outcome == 'invalidated'
    finally:
        release.set()
        await pump.stop()


async def test_cancelling_one_waiter_does_not_cancel_request_or_other_waiters():
    entered, release = asyncio.Event(), asyncio.Event()

    async def handler(request):
        entered.set()
        await release.wait()

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        receipt = pump.submit(_target(), 1)
        await asyncio.wait_for(entered.wait(), 1)
        cancelled = asyncio.create_task(receipt.completion.wait())
        surviving = asyncio.create_task(receipt.completion.wait())
        await asyncio.sleep(0)
        cancelled.cancel()
        with pytest.raises(asyncio.CancelledError):
            await cancelled
        assert receipt.completion.result is None
        release.set()
        result = await asyncio.wait_for(surviving, 1)
        assert result.outcome == 'handled'
        assert await receipt.completion.wait() is result
    finally:
        release.set()
        await pump.stop()


@pytest.mark.parametrize('mode', ['advance', 'catch_up', 'rejected'])
async def test_adapter_result_is_per_request_without_next_submission(mode):
    async with experiment() as run:
        accepted = run.adapter._accepted_target_ref
        run.clock.advance(0.04)
        run.client.ik_results.extend(joints(q) for q in
            ([0.24] if mode == 'advance' else [0.26] if mode == 'catch_up' else [0.5, 0.45, 0.4]))
        receipt = await run.adapter.submit_tcp(_target(0.308), 99)
        result = await asyncio.wait_for(receipt.completion.wait(), 1)
        assert result.command == receipt.command
        assert result.outcome == ('ik_rejected' if mode == 'rejected' else 'sdk_returned')
        assert result.sdk_outcome == ('not_started' if mode == 'rejected' else 'returned')
        assert result.accepted_command == (receipt.command if mode == 'advance' else accepted)
        assert result.pvat_mode == (None if mode == 'rejected' else mode)
        assert result.error == ('ik_joint_jump' if mode == 'rejected' else None)


@pytest.mark.parametrize('logging', ['blocked', 'failed'])
async def test_sdk_outcome_precedes_logging_and_survives_recording_failure(logging):
    adapter, client, _ = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()

    async def record(event, timestamp):
        if event['kind'] == 'pvat_sent':
            entered.set()
            if logging == 'blocked':
                await release.wait()
            else:
                raise RuntimeError('injected recording failure')

    adapter._event_callback = record
    try:
        receipt = await adapter.submit_tcp(_target(), 1)
        result = await asyncio.wait_for(receipt.completion.wait(), 1)
        assert result.outcome == 'sdk_returned'
        assert result.accepted_command == receipt.command
        await asyncio.wait_for(entered.wait(), 1)
        if logging == 'failed':
            await wait_until(lambda: adapter.pump_fault is not None)
            assert str(adapter.pump_fault) == 'recording_unavailable'
        assert receipt.completion.result is result
    finally:
        release.set()
        await adapter.disconnect()


@pytest.mark.parametrize('rpc_outcome', ['returned', 'error', 'timeout', 'cancelled'])
@pytest.mark.parametrize('invalidated', [False, True])
async def test_sdk_exit_keeps_actual_rpc_evidence_after_invalidation(rpc_outcome, invalidated):
    adapter, client, _ = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()

    async def send(*args):
        entered.set()
        await release.wait()
        if rpc_outcome == 'error':
            raise RuntimeError('private sdk details')

    client.move_pvat = send
    try:
        receipt = await adapter.submit_tcp(_target(), 7)
        await asyncio.wait_for(entered.wait(), 1)
        if invalidated:
            adapter._pump.invalidate()
            assert receipt.completion.invalidated
            assert receipt.completion.result is None
        if rpc_outcome == 'cancelled':
            await adapter._pump.stop()
        elif rpc_outcome != 'timeout':
            release.set()
        result = await asyncio.wait_for(receipt.completion.wait(), 1)
        assert result.sdk_outcome == rpc_outcome
        assert result.sdk_request_id is not None
        assert result.invalidated is (invalidated or rpc_outcome == 'cancelled')
        if invalidated or rpc_outcome != 'returned':
            assert result.accepted_command is None
        else:
            assert result.accepted_command == receipt.command
        assert result.outcome == ('cancelled' if rpc_outcome == 'cancelled' else
                                  'invalidated' if invalidated else
                                  'sdk_returned' if rpc_outcome == 'returned' else 'failed')
    finally:
        release.set()
        await adapter.disconnect()


async def test_completed_handles_are_not_retained_by_pump():
    async def handler(request):
        pass

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        refs = []
        for index in range(1000):
            receipt = pump.submit(_target(), index)
            refs.append(weakref.ref(receipt.completion))
        gc.collect()
        assert sum(ref() is not None for ref in refs) == 1
        await pump.stop()
        assert receipt.completion.result.outcome == 'invalidated'
    finally:
        await pump.stop()


async def test_external_pump_task_cancel_does_not_strand_pending_or_accept_more():
    entered = asyncio.Event()

    async def handler(request):
        entered.set()
        await asyncio.Event().wait()

    pump = PvatPump(handler, period_s=0.001)
    await pump.start()
    try:
        active = pump.submit(_target(), 1)
        await asyncio.wait_for(entered.wait(), 1)
        pending = pump.submit(_target(), 2)
        task = pump._task
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert active.completion.result.outcome == 'cancelled'
        assert pending.completion.result.outcome == 'invalidated'
        assert not pump.running
        assert str(pump.fault) == 'pvat_pump_failed'
        with pytest.raises(BackendCommandError, match='pvat_pump_failed'):
            await pump.start()
        with pytest.raises(BackendCommandError, match='pvat_pump_not_running'):
            pump.submit(_target(), 3)
    finally:
        await pump.stop()


@pytest.mark.parametrize('logging', ['blocked', 'failed'])
async def test_rejection_result_does_not_depend_on_diagnostic_delivery(logging):
    async with experiment() as run:
        entered, release = asyncio.Event(), asyncio.Event()

        async def record(event, timestamp):
            if event['kind'] == 'ik_candidate_rejected':
                entered.set()
                if logging == 'blocked':
                    await release.wait()
                else:
                    raise RuntimeError('injected diagnostic failure')

        run.adapter._event_callback = record
        try:
            run.clock.advance(0.04)
            run.client.ik_results.extend(joints(q) for q in [0.5, 0.45, 0.4])
            receipt = await run.adapter.submit_tcp(_target(0.308), 99)
            result = await asyncio.wait_for(receipt.completion.wait(), 1)
            assert result.outcome == 'ik_rejected'
            assert result.error == 'ik_joint_jump'
            await asyncio.wait_for(entered.wait(), 1)
            assert run.adapter.pump_fault is None
        finally:
            release.set()


async def test_persistent_ik_failure_resolves_fifth_request_and_keeps_fault():
    async with experiment() as run:
        for index in range(5):
            run.clock.advance(0.04)
            run.client.ik_results.extend(joints(q) for q in [0.5, 0.45, 0.4])
            receipt = await run.adapter.submit_tcp(_target(0.308), 90 + index)
            result = await asyncio.wait_for(receipt.completion.wait(), 1)
            assert result.outcome == 'ik_rejected'
            assert result.error == ('ik_failure_persistent' if index == 4 else 'ik_joint_jump')
        await wait_until(lambda: run.adapter.pump_fault is not None)
        assert str(run.adapter.pump_fault) == 'ik_failure_persistent'
        assert result is receipt.completion.result


async def test_spontaneous_sdk_cancel_latches_pump_fault_without_auto_restart():
    adapter, client, _ = await _connected_control_adapter()

    async def cancelled_send(*args):
        raise asyncio.CancelledError()

    client.move_pvat = cancelled_send
    try:
        receipt = await adapter.submit_tcp(_target(), 1)
        result = await asyncio.wait_for(receipt.completion.wait(), 1)
        assert result.outcome == 'cancelled'
        assert result.sdk_outcome == 'cancelled'
        await wait_until(lambda: adapter.pump_fault is not None)
        assert str(adapter.pump_fault) == 'pvat_pump_failed'
        await adapter.preflight()
        assert not adapter._pump.running
        with pytest.raises(BackendCommandError, match='pvat_pump_failed'):
            await adapter.submit_tcp(_target(), 2)
    finally:
        await adapter.disconnect()
