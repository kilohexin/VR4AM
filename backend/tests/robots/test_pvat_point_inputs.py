"""Point provenance uses injected fake clients only; never imports the SDK."""
import asyncio
from dataclasses import FrozenInstanceError, replace

import pytest

from app.robots.lebai_ik_policy import evaluate_ik_candidate
from app.robots.lebai_pump import PvatRequest
from app.robots.lebai_pvat import build_pvat_point
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target
from tests.robots.test_lebai_backpressure_experiments import experiment, wait_until
from tests.robots.test_lebai_pvat import LIMITS, ZERO


@pytest.mark.parametrize('previous', [None, ZERO, (.02, 0., 0., 0., 0., 0.)])
def test_point_keeps_exact_reference_and_distinguishes_equal_values(previous):
    actual_speed = (.005, 0., 0., 0., 0., 0.) if previous != ZERO else ZERO
    point = build_pvat_point((.002, *ZERO[1:]), ZERO, actual_speed, previous, LIMITS)
    assert point.reference_qd == (actual_speed if previous is None else previous)
    assert point.reference_source == ('observed_velocity' if previous is None else 'previous_command_velocity')


def test_point_reference_is_immutable_and_copied_from_mutable_inputs():
    reference = [.02, *ZERO[1:]]
    point = build_pvat_point((.002, *ZERO[1:]), ZERO, ZERO, reference, LIMITS)
    reference[0] = 99.
    assert point.reference_qd == (.02, *ZERO[1:])
    with pytest.raises(FrozenInstanceError):
        point.reference_source = 'observed_velocity'


@pytest.mark.asyncio
@pytest.mark.parametrize('gap_ms', [-1, 0, 79, 80, 120])
async def test_point_reference_matches_existing_horizon_choice(gap_ms):
    adapter, _, clock = await _connected_control_adapter()
    try:
        adapter._previous_sent_qd = (.12, *ZERO[1:])
        adapter._last_pvat_started_ns = clock.now_ns()
        adapter._last_pvat_clock_started_ns = clock.now_ns()
        clock.advance_ms(gap_ms)
        snapshot = replace(adapter._snapshot, captured_ns=clock.now_ns())
        solution = (.024, *snapshot.actual_q[1:])
        metrics = evaluate_ik_candidate(solution, snapshot.actual_q, previous_solution_q=solution)
        point = adapter._build_advancing_candidate(snapshot, _target(), metrics, 1.).point
        previous_used = 0 <= gap_ms < 80
        assert point.reference_source == ('previous_command_velocity' if previous_used else 'observed_velocity')
        assert point.reference_qd == (adapter._previous_sent_qd if previous_used else snapshot.actual_qd)
        assert point.qd[0] == pytest.approx(.15 if previous_used else .04)
    finally:
        await adapter.disconnect()


def assert_inputs(record, snapshot, point):
    inputs = record['point_inputs']
    assert inputs == {
        'snapshot_read_requested_ns': snapshot.read_requested_ns,
        'snapshot_read_started_ns': snapshot.captured_ns,
        'snapshot_read_completed_ns': snapshot.read_completed_ns,
        'actual_q': list(snapshot.actual_q),
        'actual_qd': list(snapshot.actual_qd),
        'actual_qdd': list(snapshot.actual_qdd),
        'controller_target_q': list(snapshot.target_q),
        'controller_target_qd': list(snapshot.target_qd),
        'controller_target_qdd': list(snapshot.target_qdd),
        'velocity_reference_source': point.reference_source,
        'velocity_reference_qd': list(point.reference_qd),
    }


@pytest.mark.asyncio
@pytest.mark.parametrize('previous', [None, (.02, *ZERO[1:])])
async def test_lifecycle_and_sent_capture_inputs_without_additional_reads(previous):
    adapter, client, clock = await _connected_control_adapter()
    events = []

    async def record(event, _timestamp):
        events.append(event)

    adapter._event_callback = record
    adapter._previous_sent_qd = previous
    adapter._last_pvat_started_ns = None if previous is None else clock.now_ns()
    adapter._last_pvat_clock_started_ns = None if previous is None else clock.now_ns()
    solution = (.002, *adapter._snapshot.actual_q[1:])
    client.ik_results.append(list(solution))
    # Replace the initially queued zero-displacement solution.
    client.ik_results.popleft()
    client.kin_data['target_joint_pose'] = [.001, *adapter._snapshot.target_q[1:]]
    client.kin_data['target_joint_speed'] = [.003, *ZERO[1:]]
    client.kin_data['target_joint_acc'] = [.004, *ZERO[1:]]
    client.read_calls.clear()
    await adapter._pump.start()
    try:
        await adapter._send_target(PvatRequest(_target(), 71, adapter._pump._generation))
        await wait_until(lambda: any(x['kind'] == 'sdk_request_lifecycle' for x in events))
        snapshot = adapter._snapshot
        point = build_pvat_point(solution, snapshot.actual_q, snapshot.actual_qd, previous, adapter._pvat_limits)
        lifecycle = next(x for x in events if x['kind'] == 'sdk_request_lifecycle')
        sent = next(x for x in events if x['kind'] == 'pvat_sent')
        for entry in (lifecycle, sent):
            assert_inputs(entry, snapshot, point)
        assert client.read_calls.count('get_kin_data') == 1
        assert client.read_calls.count('kinematics_inverse') == 1
        assert client.network_calls == client.discovery_calls == 0
        assert client.write_calls == [('move_pvat', list(point.q), list(point.qd), list(point.qdd), point.horizon_s)]
        # A recorder cannot mutate another record or the chosen point/snapshot.
        lifecycle['point_inputs']['velocity_reference_qd'][0] = 99.
        assert sent['point_inputs']['velocity_reference_qd'] == list(point.reference_qd)
        assert point.reference_qd[0] != 99.
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('outcome', ['returned', 'timeout', 'cancelled', 'error'])
async def test_invalidated_sdk_attempt_keeps_original_inputs(outcome):
    adapter, client, _ = await _connected_control_adapter()
    events = []
    entered, release = asyncio.Event(), asyncio.Event()

    async def record(event, _timestamp):
        events.append(event)

    async def send(*args):
        entered.set()
        await release.wait()
        if outcome == 'error':
            raise RuntimeError('synthetic transport error')

    adapter._event_callback = record
    client.move_pvat = send
    await adapter._pump.start()
    task = asyncio.create_task(adapter._send_target(PvatRequest(_target(), 81, adapter._pump._generation)))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        snapshot = adapter._snapshot
        adapter._pump.invalidate()
        if outcome == 'cancelled':
            task.cancel()
            with pytest.raises(asyncio.CancelledError):
                await task
        else:
            if outcome != 'timeout':
                release.set()
            await task
        await wait_until(lambda: any(x['kind'] == 'sdk_request_lifecycle' for x in events))
        lifecycle = next(x for x in events if x['kind'] == 'sdk_request_lifecycle')
        assert lifecycle['rpc_outcome'] == outcome and lifecycle['invalidated']
        point = build_pvat_point(snapshot.actual_q, snapshot.actual_q, snapshot.actual_qd, None, adapter._pvat_limits)
        assert_inputs(lifecycle, snapshot, point)
        assert not any(x['kind'] == 'pvat_sent' for x in events)
    finally:
        release.set()
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_explicit_catch_up_records_the_pinned_point_inputs_without_ik():
    async with experiment() as run:
        run.clock.advance(.04)
        run.client.read_calls.clear()
        receipt = await run.adapter.submit_catch_up(run.adapter._accepted_target_ref, 91)
        done = await asyncio.wait_for(receipt.completion.wait(), 1)
        assert done.pvat_mode == 'catch_up'
        await wait_until(lambda: any(x['kind'] == 'pvat_sent' and x['command_id'] == 91 for x in run.events))
        sent = next(x for x in run.events if x['kind'] == 'pvat_sent' and x['command_id'] == 91)
        inputs = sent['point_inputs']
        assert inputs['actual_q'] == run.client.kin_data['actual_joint_pose']
        assert inputs['velocity_reference_source'] == 'previous_command_velocity'
        assert 'kinematics_inverse' not in run.client.read_calls


@pytest.mark.asyncio
async def test_interpolated_advance_records_the_selected_point_reference():
    adapter, client, clock = await _connected_control_adapter()
    events = []

    async def record(event, _timestamp):
        events.append(event)

    adapter._event_callback = record
    await adapter._pump.start()
    try:
        await adapter._send_target(PvatRequest(_target(.300), 101, adapter._pump._generation))
        clock.advance_ms(40)
        client.ik_results.extend([[q, *adapter._snapshot.actual_q[1:]] for q in (.06, .03)])
        await adapter._send_target(PvatRequest(_target(.301), 102, adapter._pump._generation))
        sent = next(x for x in events if x['kind'] == 'pvat_sent' and x['command_id'] == 102)
        assert sent['pvat_mode'] == 'interpolated_advance'
        assert sent['ik_solution_q'][0] == .03
        assert sent['point_inputs']['velocity_reference_source'] == 'previous_command_velocity'
        assert sent['point_inputs']['velocity_reference_qd'] == [0.] * 6
        assert len(client.ik_calls) == 3
    finally:
        await adapter.disconnect()
