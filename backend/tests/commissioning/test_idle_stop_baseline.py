import asyncio
import copy
from unittest.mock import AsyncMock

import pytest

from app.commissioning import idle_stop_baseline as baseline
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


class Recorder(NoopRecorder):
    dropped_normal_events = 0

    def __init__(self):
        self.events = []
        self.fail_ready = False

    async def write_event(self, event, timestamp):
        self.events.append(copy.deepcopy(event))

    async def write_critical_event(self, kind, payload, timestamp):
        if self.fail_ready and kind == 'idle_stop_admission':
            raise RuntimeError('recording failed')
        self.events.append({'kind': kind, **copy.deepcopy(payload)})


async def run(client, recorder=None, authorization_check=lambda: None, **kwargs):
    clock = kwargs.pop('clock', FakeClock())
    factory = AsyncMock(return_value=client)
    result = await baseline.run_baseline(control_settings(), recorder or Recorder(), factory,
        authorization_check, clock=clock.now_ns, sleep=clock.sleep, **kwargs)
    factory.assert_awaited_once()
    return result


async def test_idle_baseline_sends_one_stop_and_preserves_original_verification():
    client, recorder = FakeLebaiClient.idle(), Recorder()
    result = await run(client, recorder)
    assert result['complete'] and result['stop_confirmed']
    assert client.write_calls == [('stop_move',)]
    assert result['sdk_stop_entries'] == 1 and result['latched_fault'] is None
    assert result['native_sdk_await_qpc_ms'] is not None and result['native_sdk_await_qpc_ms'] >= 0
    assert result['local_resources_released'] and not result['native_connection_release_verified']
    assert not result['hardware_stop_verified']
    assert any(e['kind'] == 'stop_observation_summary' and e['complete'] for e in recorder.events)


@pytest.mark.parametrize('state', ['MOVING', 'PAUSED', 'STOP', 'ROBOT_ON', 'ESTOP'])
async def test_nonidle_admission_has_zero_writes_even_during_cleanup(state):
    client = FakeLebaiClient.idle()
    client.robot_state = state
    result = await run(client)
    assert not result['complete'] and result['sdk_stop_entries'] == 0
    assert client.write_calls == [] and result['local_resources_released']


@pytest.mark.parametrize('speed', [.009587, .000000001, float('nan')])
async def test_idle_label_with_nonzero_or_invalid_velocity_has_zero_writes(speed):
    client = FakeLebaiClient.idle()
    client.kin_data['actual_joint_speed'][0] = speed
    result = await run(client)
    assert not result['complete'] and client.write_calls == []


async def test_recorder_failure_before_stop_has_zero_writes():
    recorder = Recorder()
    recorder.fail_ready = True
    client = FakeLebaiClient.idle()
    result = await run(client, recorder)
    assert not result['complete'] and client.write_calls == []


async def test_authorization_rejected_before_connection_has_zero_connections():
    factory = AsyncMock()
    def denied():
        raise ValueError('authorization expired')
    result = await baseline.run_baseline(control_settings(), Recorder(), factory, denied)
    assert not result['complete'] and result['sdk_stop_entries'] == 0
    factory.assert_not_called()


async def test_state_change_after_admission_record_has_zero_writes():
    client = FakeLebaiClient.idle()
    recorder = Recorder()
    original = recorder.write_critical_event
    async def change(kind, payload, timestamp):
        await original(kind, payload, timestamp)
        if kind == 'idle_stop_admission':
            client.robot_state = 'MOVING'
    recorder.write_critical_event = change
    result = await run(client, recorder)
    assert not result['complete'] and client.write_calls == []


async def test_expiry_after_record_has_zero_writes():
    client, recorder = FakeLebaiClient.idle(), Recorder()
    expired = False
    original = recorder.write_critical_event
    async def expire(kind, payload, timestamp):
        nonlocal expired
        await original(kind, payload, timestamp)
        if kind == 'idle_stop_admission':
            expired = True
    recorder.write_critical_event = expire
    def check():
        if expired:
            raise ValueError('expired')
    result = await run(client, recorder, check)
    assert not result['complete'] and client.write_calls == []


@pytest.mark.parametrize('kind', ['timeout', 'error', 'moving'])
async def test_stop_failure_keeps_latch_and_does_not_retry(kind):
    client, clock, recorder = FakeLebaiClient.idle(), FakeClock(), Recorder()
    original = client.stop_move
    async def failed():
        await original()
        if kind == 'timeout':
            clock.advance_ms(610)
        elif kind == 'error':
            raise RuntimeError('native error')
        else:
            client.robot_state = 'MOVING'
    client.stop_move = failed
    result = await run(client, recorder, clock=clock)
    assert not result['complete'] and not result['stop_confirmed']
    assert result['latched_fault'] == ('stop_incomplete' if kind == 'moving' else 'stop_unverified')
    assert client.write_calls == [('stop_move',)]
    assert any(e['kind'] == 'stop_observation_summary' for e in recorder.events)


async def test_cancel_before_stop_record_cannot_create_cleanup_stop():
    client, recorder = FakeLebaiClient.idle(), Recorder()
    reached, block = asyncio.Event(), asyncio.Event()
    original = recorder.write_critical_event
    async def delayed(kind, payload, timestamp):
        await original(kind, payload, timestamp)
        if kind == 'idle_stop_admission':
            reached.set()
            await block.wait()
    recorder.write_critical_event = delayed
    task = asyncio.create_task(run(client, recorder))
    try:
        await asyncio.wait_for(reached.wait(), 1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    finally:
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
    assert client.write_calls == []


async def test_cancel_after_local_request_creation_before_sdk_task_has_zero_writes(monkeypatch):
    client, recorder = FakeLebaiClient.idle(), Recorder()
    original = baseline.RealLebaiAdapter._ensure_stop_transaction
    def cancel_at_creation(adapter):
        original(adapter)
        owner = asyncio.current_task()
        adapter._stop_transaction.on_request_started = lambda _method: owner.cancel()
    monkeypatch.setattr(baseline.RealLebaiAdapter, '_ensure_stop_transaction', cancel_at_creation)
    task = asyncio.create_task(run(client, recorder))
    with pytest.raises(asyncio.CancelledError):
        await asyncio.wait_for(task, 2)
    assert client.write_calls == []
    result = next(event for event in recorder.events if event['kind'] == 'idle_stop_baseline_result')
    assert result['sdk_stop_entries'] == 0 and result['local_resources_released']


@pytest.mark.parametrize('cancel', [False, True])
async def test_sdk_lock_wait_cannot_bypass_expiry_or_cancellation(monkeypatch, cancel):
    client, clock, recorder = FakeLebaiClient.idle(), FakeClock(), Recorder()
    locked, release = asyncio.Event(), asyncio.Event()
    holder = None
    original = baseline.RealLebaiAdapter.read_stop_observation
    count = 0
    async def read_then_queue(adapter):
        nonlocal count, holder
        sample = await original(adapter)
        count += 1
        if count == 2:
            async def hold():
                async with adapter._sdk_lock:
                    locked.set()
                    await release.wait()
            holder = asyncio.create_task(hold())
            await locked.wait()
        return sample
    monkeypatch.setattr(baseline.RealLebaiAdapter, 'read_stop_observation', read_then_queue)
    task = asyncio.create_task(run(client, recorder, clock=clock))
    try:
        await asyncio.wait_for(locked.wait(), 1)
        await asyncio.sleep(0)
        if cancel:
            task.cancel()
        else:
            clock.advance_ms(100)
        release.set()
        if cancel:
            with pytest.raises(asyncio.CancelledError):
                await asyncio.wait_for(task, 2)
        else:
            assert not (await asyncio.wait_for(task, 2))['complete']
        assert client.write_calls == []
    finally:
        release.set()
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        if holder:
            await asyncio.gather(holder, return_exceptions=True)


@pytest.mark.parametrize('method', ['move_pvat', 'movej', 'set_claw', 'stop_sys', 'start_sys', 'reset'])
async def test_proxy_never_delegates_other_robot_writes(method):
    client = FakeLebaiClient.idle()
    setattr(client, method, AsyncMock())
    proxy = baseline.StopOnlyClient(client, lambda: None)
    with pytest.raises(BackendCommandError, match='write_forbidden'):
        await getattr(proxy, method)()
    getattr(client, method).assert_not_called()


def valid_sample():
    return {'raw_robot_state': 'IDLE', 'estop': None, 'latched_fault': None, 'running_motion': None,
            'read_requested_ns': 1_000_000_000, 'captured_ns': 1_000_000_000,
            'read_completed_ns': 1_000_000_000, 'actual_qd': [0.0]*6}


@pytest.mark.parametrize('changes,now_ns', [
    ({'read_completed_ns': 1_000_000_001}, 1_000_000_000),
    ({'captured_ns': True}, 1_000_000_000),
    ({'read_completed_ns': None}, 1_000_000_000),
    ({}, 1_100_000_000),
    ({'raw_robot_state': 'PAUSED'}, 1_000_000_000),
    ({'running_motion': 4}, 1_000_000_000),
    ({'estop': 'hard_estop'}, 1_000_000_000),
    ({'actual_qd': [0]*5}, 1_000_000_000),
    ({'actual_qd': [False]*6}, 1_000_000_000),
])
def test_unknown_future_or_invalid_admission_cannot_grant_permission(changes, now_ns):
    with pytest.raises(BackendCommandError):
        baseline.validate_idle_sample({**valid_sample(), **changes}, now_ns)


@pytest.mark.parametrize('key', ['estop', 'latched_fault', 'running_motion'])
def test_missing_health_fields_are_unknown_not_healthy(key):
    sample = valid_sample()
    sample.pop(key)
    with pytest.raises(BackendCommandError):
        baseline.validate_idle_sample(sample, 1_000_000_000)


def test_float_state_code_is_not_accepted_as_integer_idle():
    with pytest.raises(BackendCommandError):
        baseline.validate_idle_sample({**valid_sample(), 'raw_robot_state': 5.0}, 1_000_000_000)


async def test_changed_tcp_after_record_is_rejected_before_sdk(monkeypatch):
    client, recorder = FakeLebaiClient.idle(), Recorder()
    original = recorder.write_critical_event
    async def change(kind, payload, timestamp):
        await original(kind, payload, timestamp)
        if kind == 'idle_stop_admission':
            client.tcp['z'] += .1
    recorder.write_critical_event = change
    result = await run(client, recorder)
    assert not result['complete'] and client.write_calls == []


async def test_recorder_fault_at_sdk_scheduling_boundary_has_zero_writes(monkeypatch):
    client, recorder = FakeLebaiClient.idle(), Recorder()
    original = baseline.RealLebaiAdapter._ensure_stop_transaction
    def fail_at_creation(adapter):
        original(adapter)
        adapter._stop_transaction.on_request_started = lambda _method: setattr(recorder, 'fatal_error', 'disk_failed')
    monkeypatch.setattr(baseline.RealLebaiAdapter, '_ensure_stop_transaction', fail_at_creation)
    result = await run(client, recorder)
    assert not result['complete'] and client.write_calls == []


@pytest.mark.parametrize('moving', [True, False])
async def test_tail_motion_after_successful_stop_verification_invalidates_baseline(monkeypatch, moving):
    client, recorder = FakeLebaiClient.idle(), Recorder()
    original = baseline.RealLebaiAdapter.read_stop_observation
    count = 0
    async def change_on_tail(adapter):
        nonlocal count
        count += 1
        if count == 3:
            if moving:
                client.robot_state = 'MOVING'
            else:
                client.kin_data['actual_joint_speed'][0] = .009587
        return await original(adapter)
    monkeypatch.setattr(baseline.RealLebaiAdapter, 'read_stop_observation', change_on_tail)
    result = await run(client, recorder)
    assert not result['complete'] and result['tail_rejected']
    assert client.write_calls == [('stop_move',)]
    assert any(e['kind'] == 'idle_stop_tail_rejected' and 'sample' in e for e in recorder.events)


async def test_false_authorization_result_is_not_accepted():
    factory = AsyncMock()
    result = await baseline.run_baseline(control_settings(), Recorder(), factory, lambda: False)
    assert not result['complete']
    factory.assert_not_called()


@pytest.mark.parametrize('coordinate', ['joint', 'tcp'])
async def test_tail_position_drift_with_zero_reported_speed_invalidates_baseline(monkeypatch, coordinate):
    client, recorder = FakeLebaiClient.idle(), Recorder()
    original = baseline.RealLebaiAdapter.read_stop_observation
    count = 0
    async def drift_on_tail(adapter):
        nonlocal count
        count += 1
        if count == 3:
            if coordinate == 'joint':
                client.kin_data['actual_joint_pose'][0] += .01
            else:
                client.kin_data['actual_tcp_pose']['x'] += .002
        return await original(adapter)
    monkeypatch.setattr(baseline.RealLebaiAdapter, 'read_stop_observation', drift_on_tail)
    result = await run(client, recorder)
    assert not result['complete'] and result['tail_rejected']
    assert client.write_calls == [('stop_move',)]


async def test_cancel_after_sdk_entry_keeps_unverified_latch_and_does_not_retry():
    client, recorder = FakeLebaiClient.idle(), Recorder()
    entered, block = asyncio.Event(), asyncio.Event()
    original = client.stop_move
    async def pending():
        await original()
        entered.set()
        await block.wait()
    client.stop_move = pending
    task = asyncio.create_task(run(client, recorder))
    try:
        await asyncio.wait_for(entered.wait(), 1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await asyncio.wait_for(task, 2)
        result = next(event for event in recorder.events if event['kind'] == 'idle_stop_baseline_result')
        assert not result['complete'] and result['latched_fault'] == 'stop_unverified'
        assert result['sdk_stop_entries'] == 1 and result['local_resources_released']
        assert client.write_calls == [('stop_move',)]
    finally:
        block.set()
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)


async def test_recorder_close_failure_prevents_complete_without_another_write():
    client, recorder = FakeLebaiClient.idle(), Recorder()
    recorder.close = AsyncMock(side_effect=RuntimeError('synthetic close error'))
    result = await run(client, recorder)
    assert not result['complete'] and result['cleanup_errors'] == ['RuntimeError']
    assert result['sdk_stop_entries'] == 1 and client.write_calls == [('stop_move',)]


async def test_unknown_connection_health_is_not_treated_as_connected():
    client = FakeLebaiClient.idle()
    client.connected = 'unknown'
    result = await run(client)
    assert not result['complete'] and client.write_calls == []
