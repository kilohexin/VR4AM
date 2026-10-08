"""Independent diagnostic clocks, fake SDK only; no live connection/capture."""
import asyncio
import json

import pytest

from app.recording import clock_anchor, commissioning
from app.commissioning import readonly_state
from app.robots import lebai_stop_transaction
from app.robots.lebai_stop_diagnostics import StopDiagnosticSampler
from tests.scripts.test_readonly_state import Reader, run, setup
from tests.robots.fake_lebai import FakeLebaiClient
from app.robots.base import StopReason
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _wait_until


def test_performance_anchor_brackets_wall_without_relabeling_control_clock():
    sample = getattr(clock_anchor, 'sample_performance_anchor', None)
    assert callable(sample), 'independent performance anchor missing'
    values = iter([9000, 9125])
    anchor = sample(lambda: next(values), lambda: 1_000_000)
    assert anchor['clock'] == 'time.perf_counter_ns'
    assert anchor['performance_before_ns'] == 9000
    assert anchor['performance_after_ns'] == 9125
    assert anchor['wall_time_ns'] == 1_000_000
    assert anchor['sampling_span_ns'] == 125
    assert anchor['alignment_verified'] is False
    assert anchor['clock_info']['perf_counter']['resolution'] > 0


@pytest.mark.parametrize('value', [True, -1, float('nan'), None])
def test_invalid_performance_stamp_stays_missing(value):
    sample = getattr(clock_anchor, 'sample_performance_ns', None)
    assert callable(sample), 'safe performance stamp missing'
    assert sample(lambda: value) is None


def test_performance_clock_exception_and_reversed_anchor_are_nonfatal():
    stamp = getattr(clock_anchor, 'sample_performance_ns', None)
    anchor = getattr(clock_anchor, 'sample_performance_anchor', None)
    assert callable(stamp) and callable(anchor)
    def fail():
        raise RuntimeError('private clock error')
    assert stamp(fail) is None
    assert anchor(fail, lambda: 1) is None
    values = iter([9, 8])
    assert anchor(lambda: next(values), lambda: 1) is None


async def test_readonly_sdk_await_uses_independent_clock_with_submillisecond_samples(setup, monkeypatch):
    counter = iter(range(9_000_000_000, 10_000_000_000, 125))
    monkeypatch.setattr(clock_anchor, 'perf_counter_ns', lambda: next(counter), raising=False)
    code, report, connections = await run(setup, Reader(setup[2]))
    assert code == 0 and len(connections) == 1
    assert report['performance_anchor_started']['clock'] == 'time.perf_counter_ns'
    assert report['performance_anchor_completed']['alignment_verified'] is False
    for read in report['reads']:
        timing = read['diagnostic_timing']
        assert timing['clock'] == 'time.perf_counter_ns'
        assert timing['sdk_await_completed_ns'] - timing['sdk_await_started_ns'] == 125
        assert read['completed_ns'] - read['started_ns'] == 1_000_000
    assert report['hardware_stop_verified'] is False
    assert set(r['method'] for r in report['reads']) == set(readonly_state.READ_METHODS)


@pytest.mark.parametrize('failure', ['clock', 'read'])
async def test_diagnostic_clock_failure_does_not_change_read_allowlist_or_rpc_failure(setup, monkeypatch, failure):
    def fail():
        raise RuntimeError('private diagnostic detail')
    monkeypatch.setattr(clock_anchor, 'perf_counter_ns', fail, raising=False)
    reader = Reader(setup[2], fail='get_kin_data' if failure == 'read' else None)
    code, report, _ = await run(setup, reader)
    assert code == (2 if failure == 'read' else 0)
    assert report['performance_anchor_started'] is report['performance_anchor_completed'] is None
    assert all(r['diagnostic_timing']['sdk_await_started_ns'] is None and
               r['diagnostic_timing']['sdk_await_completed_ns'] is None for r in report['reads'])
    assert 'private diagnostic detail' not in setup[1].read_text()
    assert report['reads'][-1]['outcome'] == ('failed' if failure == 'read' else 'returned')


@pytest.mark.parametrize('control_elapsed, performance_elapsed, expected', [
    (1_000, 10_000_000_000, 'returned'), (300_000_000, 125, 'returned_late'),
])
async def test_stop_deadline_depends_only_on_existing_control_clock(monkeypatch, control_elapsed, performance_elapsed, expected):
    control, performance, events = [1000], [9_000_000_000], []
    monkeypatch.setattr(clock_anchor, 'perf_counter_ns', lambda: performance[0], raising=False)
    async def send():
        control[0] += control_elapsed
        performance[0] += performance_elapsed
    rpc = lebai_stop_transaction.StopRpc(1, 1, 'stop_move', send, lambda: control[0], events.append)
    await rpc.task
    if expected == 'returned_late':
        with pytest.raises(TimeoutError):
            await rpc.wait()
    else:
        await rpc.wait()
    event, = events
    assert event['outcome'] == expected
    assert event['deadline_ns'] == 200_001_000
    assert event['diagnostic_timing']['sdk_await_completed_ns'] - event['diagnostic_timing']['sdk_await_started_ns'] == performance_elapsed
    assert event['diagnostic_timing']['anchor_completed']['alignment_verified'] is False
    assert event['sdk_await_completed_ns'] - event['sdk_await_started_ns'] == control_elapsed


async def test_stop_sdk_error_survives_missing_performance_clock(monkeypatch):
    def unavailable():
        raise RuntimeError('private diagnostic detail')
    monkeypatch.setattr(clock_anchor, 'perf_counter_ns', unavailable, raising=False)
    async def send():
        raise ValueError('sdk failure')
    events = []
    rpc = lebai_stop_transaction.StopRpc(1, 1, 'stop_move', send, lambda: 1000, events.append)
    with pytest.raises(ValueError, match='sdk failure'):
        await rpc.wait()
    event, = events
    assert event['outcome'] == 'error'
    assert event['diagnostic_timing']['sdk_await_completed_ns'] is None
    assert event['diagnostic_timing']['anchor_started'] is None
    assert 'private diagnostic detail' not in json.dumps(event)


async def test_sampler_read_timings_are_separate_from_control_clock(monkeypatch):
    counter = iter(range(9000, 99_000, 125))
    monkeypatch.setattr(clock_anchor, 'perf_counter_ns', lambda: next(counter), raising=False)
    sampler = StopDiagnosticSampler(FakeLebaiClient.idle(), 1, lambda: 1000, lambda _: None)
    event = {'reads': []}
    await sampler._read(event)
    assert len(event['reads']) == 3
    for read in event['reads']:
        assert read['started_ns'] == read['completed_ns'] == 1000
        assert read['diagnostic_timing']['sdk_await_completed_ns'] - read['diagnostic_timing']['sdk_await_started_ns'] == 125


async def test_session_anchors_keep_original_event_clock(tmp_path):
    recorder = commissioning.CommissioningRecorder(tmp_path, {}, session_id='performance-offline')
    await recorder.start()
    await recorder.write_event({'kind': 'offline_sample'}, 123)
    await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    for entry in events:
        if entry['kind'] in {'session_started', 'session_ended'}:
            assert entry['performance_anchor']['clock'] == 'time.perf_counter_ns'
            assert entry['performance_anchor']['alignment_verified'] is False
            assert entry['clock_anchor']['clock'] == 'time.monotonic_ns'
        else:
            assert entry['server_mono_ns'] == 123
            assert 'performance_anchor' not in entry


async def test_stop_sdk_lock_wait_has_independent_timing_without_extra_writes(monkeypatch):
    performance = [9000]
    monkeypatch.setattr(clock_anchor, 'perf_counter_ns', lambda: performance[0], raising=False)
    adapter, client, control = await _connected_control_adapter()
    events = []
    async def record(event, _):
        events.append(event)
    adapter._event_callback = record
    await adapter._sdk_lock.acquire()
    stop = asyncio.create_task(adapter.stop(StopReason.GRIP_RELEASED))
    try:
        await _wait_until(lambda: adapter._stop_transaction is not None)
        await asyncio.sleep(0)
        performance[0] += 700_000
        control.advance_ms(7)
        adapter._sdk_lock.release()
        await asyncio.wait_for(stop, 1)
        event = next(e for e in events if e['kind'] == 'stop_diagnostics')
        lock = event['stop_move_lock']
        assert lock['wait_ns'] >= 7_000_000
        assert lock['diagnostic_timing']['clock'] == 'time.perf_counter_ns'
        assert lock['diagnostic_timing']['acquired_ns'] - lock['diagnostic_timing']['requested_ns'] == 700_000
        assert client.write_calls == [('stop_move',)]
    finally:
        if adapter._sdk_lock.locked():
            adapter._sdk_lock.release()
        await asyncio.gather(stop, return_exceptions=True)
        await adapter.disconnect()
