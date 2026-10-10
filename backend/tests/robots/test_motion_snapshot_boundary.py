"""Fake-only motion/read boundary regressions; no live stopping evidence."""
import asyncio

import pytest

from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_pump import PvatRequest
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target


@pytest.mark.asyncio
@pytest.mark.parametrize('fault', ['state', 'estop', 'tcp'])
@pytest.mark.parametrize('expired', [False, True])
async def test_fault_observed_between_motion_read_and_dispatch_prevents_pvat(fault, expired):
    adapter, client, clock = await _connected_control_adapter()
    motion_read_done, release_motion = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            motion_read_done.set()
            await release_motion.wait()
        return snapshot

    adapter._read_snapshot = read
    task = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    try:
        await asyncio.wait_for(motion_read_done.wait(), 1)
        clock.advance_ms(1)
        original_tcp = dict(client.tcp)
        if fault == 'state':
            client.robot_state = 'ERROR'
        elif fault == 'estop':
            client.estop_reason = 'hardestop'
        else:
            client.tcp['z'] = .3
        await adapter.get_state()
        # Restore the fake controller for ordinary stop verification. The
        # completed fault observation must not disappear before dispatch.
        client.robot_state, client.estop_reason, client.tcp = 'IDLE', 0, original_tcp
        await adapter.get_state()
        if expired:
            clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000 + 1)
        release_motion.set()
        outcome = await asyncio.gather(task, return_exceptions=True)
        assert client.ik_calls == []
        assert 'move_pvat' not in [call[0] for call in client.write_calls]
        assert isinstance(outcome[0], BackendCommandError)
        assert str(outcome[0]) == {'state': 'robot_state_fault',
                                  'estop': 'estop:hard_estop', 'tcp': 'tcp_mismatch'}[fault]
        assert 'stop_move' in [call[0] for call in client.write_calls]
        assert 'stop_sys' not in [call[0] for call in client.write_calls]
    finally:
        release_motion.set()
        await asyncio.gather(task, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('advance_ms', [0, 1])
async def test_newer_complete_healthy_observation_supplies_ik_and_point_inputs(advance_ms):
    adapter, client, clock = await _connected_control_adapter()
    motion_read_done, release_motion = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot
    events = []

    async def record(event, timestamp):
        events.append(event)

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            motion_read_done.set()
            await release_motion.wait()
        return snapshot

    adapter._event_callback = record
    adapter._read_snapshot = read
    client.read_calls.clear()
    task = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    try:
        await asyncio.wait_for(motion_read_done.wait(), 1)
        clock.advance_ms(advance_ms)
        q = list(client.kin_data['actual_joint_pose'])
        q[0] += .001
        client.kin_data['actual_joint_pose'] = q
        client.ik_results.clear()
        client.ik_results.append(q)
        await adapter.get_state()
        latest = adapter._snapshot
        release_motion.set()
        await asyncio.wait_for(task, 1)
        assert client.ik_calls[0][1] == q
        event = next(e for e in events if e['kind'] == 'pvat_sent')
        assert event['point_inputs']['actual_q'] == q
        assert event['point_inputs']['snapshot_read_started_ns'] == latest.captured_ns
        assert event['timing_ms']['motion_snapshot_age_at_send'] == advance_ms
        assert event['timing_ms']['snapshot_age_at_send'] == 0
        motion = next(e for e in events if e['kind'] == 'robot_kinematics'
                      and e['snapshot_read_purpose'] == 'motion')
        assert event['diagnostic_timing']['snapshot_timing'] == motion['diagnostic_timing']
        assert event['diagnostic_timing']['dispatch_snapshot_timing'] == latest.diagnostic_timing
        for method in ('get_robot_state', 'get_estop_reason', 'get_kin_data',
                       'get_tcp', 'get_claw', 'get_running_motion'):
            assert client.read_calls.count(method) == 2
    finally:
        release_motion.set()
        await asyncio.gather(task, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_observer_inflight_at_motion_entry_cannot_lose_its_fault():
    adapter, client, clock = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()
    original = client.get_robot_state
    count = 0

    async def read_state():
        nonlocal count
        count += 1
        if count == 1:
            entered.set()
            await release.wait()
            return 'ERROR'
        return await original()

    client.get_robot_state = read_state
    observer = asyncio.create_task(adapter.get_state())
    motion = None
    try:
        await asyncio.wait_for(entered.wait(), 1)
        motion = asyncio.create_task(adapter._send_target(
            PvatRequest(_target(), 1, adapter._pump.generation)))
        await asyncio.sleep(0)
        release.set()
        await asyncio.wait_for(observer, 1)
        outcome = await asyncio.gather(motion, return_exceptions=True)
        assert client.ik_calls == []
        assert 'move_pvat' not in [c[0] for c in client.write_calls]
        assert str(outcome[0]) == 'robot_state_fault'
    finally:
        release.set()
        await asyncio.gather(*[t for t in (observer, motion) if t is not None],
                             return_exceptions=True)
        client.get_robot_state = original
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_recording_error_does_not_replace_completed_runtime_fault():
    adapter, client, clock = await _connected_control_adapter()
    read_done, release = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def record(event, timestamp):
        if event['kind'] == 'robot_kinematics' and event['robot_state'] == 'FAULT':
            raise RuntimeError('injected_record_failure')

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            read_done.set()
            await release.wait()
        return snapshot

    adapter._event_callback, adapter._read_snapshot = record, read
    task = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    try:
        await asyncio.wait_for(read_done.wait(), 1)
        client.robot_state = 'ERROR'
        with pytest.raises(BackendCommandError, match='recording_unavailable'):
            await adapter.get_state()
        client.robot_state = 'IDLE'
        release.set()
        outcome = await asyncio.gather(task, return_exceptions=True)
        assert str(outcome[0]) == 'robot_state_fault'
        assert client.ik_calls == []
        assert 'move_pvat' not in [c[0] for c in client.write_calls]
    finally:
        client.robot_state = 'IDLE'
        release.set()
        await asyncio.gather(task, return_exceptions=True)
        adapter._event_callback, adapter._read_snapshot = None, original
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('failure', ['sdk_error', 'cancelled'])
async def test_intervening_read_failure_is_not_erased_by_later_health(failure):
    adapter, client, clock = await _connected_control_adapter()
    ready, resume, reading, release_read = (asyncio.Event() for _ in range(4))
    original_read, original_tcp = adapter._read_snapshot, client.get_tcp

    async def read(**kwargs):
        result = await original_read(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            ready.set()
            await resume.wait()
        return result

    async def broken_tcp():
        reading.set()
        if failure == 'cancelled':
            await release_read.wait()
        raise RuntimeError('injected_tcp_read_error')

    adapter._read_snapshot = read
    motion = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    observer = None
    try:
        await asyncio.wait_for(ready.wait(), 1)
        client.get_tcp = broken_tcp
        observer = asyncio.create_task(adapter.get_state())
        await asyncio.wait_for(reading.wait(), 1)
        if failure == 'cancelled':
            observer.cancel()
        outcome = await asyncio.gather(observer, return_exceptions=True)
        assert isinstance(outcome[0], BaseException)
        client.get_tcp = original_tcp
        await adapter.get_state()
        resume.set()
        outcome = await asyncio.gather(motion, return_exceptions=True)
        assert str(outcome[0]) == ('robot_state_stale' if failure == 'cancelled'
                                    else 'sdk_call_failed:get_tcp')
        assert client.ik_calls == []
        assert 'move_pvat' not in [c[0] for c in client.write_calls]
        assert 'stop_move' in [c[0] for c in client.write_calls]
    finally:
        client.get_tcp = original_tcp
        resume.set()
        release_read.set()
        await asyncio.gather(*[t for t in (motion, observer) if t is not None],
                             return_exceptions=True)
        adapter._read_snapshot = original_read
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_stop_in_snapshot_send_gap_invalidates_without_late_pvat():
    adapter, client, clock = await _connected_control_adapter()
    ready, resume = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def read(**kwargs):
        snapshot = await original(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            ready.set()
            await resume.wait()
        return snapshot

    adapter._read_snapshot = read
    task = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    try:
        await asyncio.wait_for(ready.wait(), 1)
        await asyncio.wait_for(adapter.stop(StopReason.GRIP_RELEASED), 1)
        resume.set()
        await asyncio.wait_for(task, 1)
        assert client.ik_calls == []
        assert [c[0] for c in client.write_calls] == ['stop_move']
    finally:
        resume.set()
        await asyncio.gather(task, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_fault_from_invalidated_read_cannot_replace_new_generation():
    adapter, client, clock = await _connected_control_adapter()
    entered, release = asyncio.Event(), asyncio.Event()
    original = client.get_robot_state
    count = 0

    async def read_state():
        nonlocal count
        count += 1
        if count == 1:
            entered.set()
            await release.wait()
            return 'ERROR'
        return await original()

    client.get_robot_state = read_state
    observer = asyncio.create_task(adapter.get_state())
    try:
        await asyncio.wait_for(entered.wait(), 1)
        adapter._pump.invalidate()
        release.set()
        await asyncio.wait_for(observer, 1)
        await adapter._send_target(PvatRequest(_target(), 1, adapter._pump.generation))
        assert [c[0] for c in client.write_calls] == ['move_pvat']
    finally:
        release.set()
        await asyncio.gather(observer, return_exceptions=True)
        client.get_robot_state = original
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('invalid', ['client', 'generation', 'incomplete', 'stale'])
async def test_dispatch_rejects_unusable_snapshot_provenance_without_sdk(invalid):
    adapter, client, clock = await _connected_control_adapter()
    try:
        read = adapter._latest_snapshot_read
        if invalid == 'client':
            read.client = object()
        elif invalid == 'generation':
            read.generation -= 1
        elif invalid == 'incomplete':
            read.snapshot = None
        else:
            clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000 + 1)
        with pytest.raises(BackendCommandError, match='robot_state_stale'):
            adapter._dispatch_snapshot(client, adapter._pump.generation, 0)
        assert client.ik_calls == [] and client.write_calls == []
    finally:
        await adapter.disconnect()


@pytest.mark.asyncio
@pytest.mark.parametrize('stop_error', [False, True])
async def test_production_pump_fifo_observer_fault_blocks_dispatch_outside_lock(stop_error):
    adapter, client, clock = await _connected_control_adapter()
    motion_reading, release = asyncio.Event(), asyncio.Event()
    original_kin, original_stop = client.get_kin_data, client.stop_move
    count = 0
    events = []

    async def kin():
        nonlocal count
        count += 1
        if count == 1:
            motion_reading.set()
            await release.wait()
        return await original_kin()

    async def record(event, timestamp):
        events.append(event)
        if event['kind'] == 'robot_kinematics' and event['snapshot_read_purpose'] == 'state':
            client.robot_state = 'IDLE'

    async def failed_stop():
        await original_stop()
        raise RuntimeError('injected_stop_failure')

    client.get_kin_data, adapter._event_callback = kin, record
    if stop_error:
        client.stop_move = failed_stop
    observer = None
    try:
        admission = await adapter.submit_tcp(_target(), 1)
        await asyncio.wait_for(motion_reading.wait(), 1)
        client.robot_state = 'ERROR'
        observer = asyncio.create_task(adapter.get_state())
        await asyncio.sleep(0)  # Actual FIFO waiter behind the motion read.
        release.set()
        await asyncio.wait_for(observer, 1)
        result = await asyncio.wait_for(admission.completion.wait(), 1)
        assert result.outcome == 'failed' and result.sdk_outcome == 'not_started'
        assert result.error == ('stop_unverified' if stop_error else 'robot_state_fault')
        assert client.ik_calls == []
        assert [c[0] for c in client.write_calls] == ['stop_move']
        assert any(e['kind'] == 'stop_diagnostics' and
                   e['initial_fault'] == 'robot_state_fault' for e in events)
    finally:
        release.set()
        if observer is not None:
            await asyncio.gather(observer, return_exceptions=True)
        client.get_kin_data, client.stop_move = original_kin, original_stop
        adapter._event_callback = None
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_unprepared_read_fault_does_not_poison_later_ready_generation():
    adapter, client, clock = await _connected_control_adapter()
    try:
        await adapter._pump.stop()
        client.robot_state = 'ERROR'
        await adapter.get_state()
        assert adapter._snapshot_read_fault is None
        client.robot_state = 'IDLE'
        assert (await adapter.preflight()).ready
        await adapter._send_target(PvatRequest(_target(), 1, adapter._pump.generation))
        assert [c[0] for c in client.write_calls] == ['move_pvat']
    finally:
        client.robot_state = 'IDLE'
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_connection_identity_change_is_rejected_without_stopping_new_client():
    adapter, client, clock = await _connected_control_adapter()
    try:
        adapter._client = object()
        with pytest.raises(BackendCommandError, match='robot_disconnected'):
            adapter._dispatch_snapshot(client, adapter._pump.generation, 0)
        assert client.ik_calls == [] and client.write_calls == []
    finally:
        adapter._client = client
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_prepared_generation_fault_before_handler_is_not_auto_cleared_by_health():
    adapter, client, clock = await _connected_control_adapter()
    try:
        client.robot_state = 'ERROR'
        await adapter.get_state()
        client.robot_state = 'IDLE'
        await adapter.get_state()
        with pytest.raises(BackendCommandError, match='robot_state_fault'):
            await adapter._send_target(PvatRequest(_target(), 1, adapter._pump.generation))
        assert client.ik_calls == []
        assert [c[0] for c in client.write_calls] == ['stop_move']
    finally:
        client.robot_state = 'IDLE'
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_cancelled_observer_that_never_acquired_sdk_lock_does_not_create_read_fault():
    adapter, client, clock = await _connected_control_adapter()
    await adapter._sdk_lock.acquire()
    observer = asyncio.create_task(adapter.get_state())
    try:
        await asyncio.sleep(0)
        observer.cancel()
        await asyncio.gather(observer, return_exceptions=True)
        assert adapter._snapshot_read_fault is None
        adapter._sdk_lock.release()
        await adapter._send_target(PvatRequest(_target(), 1, adapter._pump.generation))
        assert [c[0] for c in client.write_calls] == ['move_pvat']
    finally:
        if adapter._sdk_lock.locked():
            adapter._sdk_lock.release()
        await asyncio.gather(observer, return_exceptions=True)
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_newer_snapshot_does_not_extend_original_motion_deadline_during_ik():
    adapter, client, clock = await _connected_control_adapter()
    ready, release = asyncio.Event(), asyncio.Event()
    original_read, original_ik = adapter._read_snapshot, client.kinematics_inverse

    async def read(**kwargs):
        result = await original_read(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            ready.set()
            await release.wait()
        return result

    async def slow_ik(*args):
        result = await original_ik(*args)
        clock.advance_ms(adapter._command_snapshot_max_age_ns() // 1_000_000)
        return result

    adapter._read_snapshot, client.kinematics_inverse = read, slow_ik
    motion = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    try:
        await asyncio.wait_for(ready.wait(), 1)
        clock.advance_ms(1)
        await adapter.get_state()
        release.set()
        outcome = await asyncio.gather(motion, return_exceptions=True)
        assert str(outcome[0]) == 'robot_state_stale'
        assert len(client.ik_calls) == 1
        assert client.write_calls == []
    finally:
        release.set()
        await asyncio.gather(motion, return_exceptions=True)
        adapter._read_snapshot, client.kinematics_inverse = original_read, original_ik
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_late_observer_record_failure_during_ik_is_rechecked_before_send():
    adapter, client, clock = await _connected_control_adapter(block_ik=True)
    recording, fail_recording = asyncio.Event(), asyncio.Event()

    async def record(event, timestamp):
        if event['kind'] == 'robot_kinematics' and event['snapshot_read_purpose'] == 'state':
            recording.set()
            await fail_recording.wait()
            raise RuntimeError('late_record_failure')

    adapter._event_callback = record
    observer = asyncio.create_task(adapter.get_state())
    motion = None
    try:
        await asyncio.wait_for(recording.wait(), 1)
        motion = asyncio.create_task(adapter._send_target(
            PvatRequest(_target(), 1, adapter._pump.generation)))
        await asyncio.wait_for(client.ik_started.wait(), 1)
        fail_recording.set()
        await asyncio.gather(observer, return_exceptions=True)
        client.release_ik.set()
        outcome = await asyncio.gather(motion, return_exceptions=True)
        assert str(outcome[0]) == 'recording_unavailable'
        assert [c[0] for c in client.write_calls] == ['stop_move']
    finally:
        fail_recording.set()
        client.release_ik.set()
        await asyncio.gather(*[t for t in (observer, motion) if t is not None],
                             return_exceptions=True)
        adapter._event_callback = None
        await adapter.disconnect()


@pytest.mark.asyncio
async def test_client_replaced_during_motion_read_cannot_receive_fault_stop():
    adapter, client, clock = await _connected_control_adapter()
    ready, release = asyncio.Event(), asyncio.Event()
    original = adapter._read_snapshot

    async def read(**kwargs):
        result = await original(**kwargs)
        if kwargs.get('read_purpose') == 'motion':
            ready.set()
            await release.wait()
        return result

    adapter._read_snapshot = read
    client.robot_state = 'ERROR'
    motion = asyncio.create_task(adapter._send_target(
        PvatRequest(_target(), 1, adapter._pump.generation)))
    try:
        await asyncio.wait_for(ready.wait(), 1)
        adapter._client = object()
        release.set()
        outcome = await asyncio.gather(motion, return_exceptions=True)
        assert str(outcome[0]) == 'robot_disconnected'
        assert client.ik_calls == [] and client.write_calls == []
        assert adapter._latched_fault is None
    finally:
        adapter._client = client
        client.robot_state = 'IDLE'
        release.set()
        await asyncio.gather(motion, return_exceptions=True)
        adapter._read_snapshot = original
        await adapter.disconnect()
