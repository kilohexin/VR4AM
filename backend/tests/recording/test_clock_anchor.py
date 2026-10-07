"""Clock-only diagnostics: injected clocks, no SDK or packet capture."""
import json

import pytest

from app.recording.clock_anchor import sample_clock_anchor
from app.recording.commissioning import CommissioningRecorder
from app.recording import commissioning
from app.commissioning import readonly_state
from tests.scripts.test_readonly_state import Reader, setup


def test_wall_sample_is_bracketed_without_asserting_exact_offset():
    values = iter([100, 108])
    anchor = sample_clock_anchor(lambda: next(values), lambda: 1000)
    assert anchor == {
        'clock': 'time.monotonic_ns', 'wall_clock': 'time.time_ns',
        'monotonic_before_ns': 100, 'wall_time_ns': 1000,
        'monotonic_after_ns': 108, 'sampling_span_ns': 8,
    }


@pytest.mark.parametrize('values', [(10, 9), (True, 10), (10, float('nan'))])
def test_invalid_monotonic_samples_leave_anchor_missing(values):
    samples = iter(values)
    assert sample_clock_anchor(lambda: next(samples), lambda: 1000) is None


@pytest.mark.parametrize('wall', [True, -1, float('nan')])
def test_invalid_wall_sample_leaves_anchor_missing(wall):
    assert sample_clock_anchor(lambda: 10, lambda: wall) is None


def test_diagnostic_clock_failure_does_not_propagate():
    def fail():
        raise RuntimeError('clock_unavailable')
    assert sample_clock_anchor(fail, lambda: 1000) is None


async def test_session_anchors_do_not_relabel_event_monotonic_time(tmp_path):
    recorder = CommissioningRecorder(tmp_path, {}, session_id='clock-only')
    await recorder.start()
    await recorder.write_event({'kind': 'offline_sample'}, 123)
    await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    for kind in ('session_started', 'session_ended'):
        entry = next(e for e in events if e['kind'] == kind)
        assert entry['clock_anchor']['sampling_span_ns'] >= 0
        assert entry['server_mono_ns'] == 0
    sample = next(e for e in events if e['kind'] == 'offline_sample')
    assert sample['server_mono_ns'] == 123
    assert 'clock_anchor' not in sample


async def test_readonly_report_anchors_add_no_sdk_operations(setup):
    config, output, now = setup
    reader = Reader(now)
    async def connect(_):
        return reader
    assert await readonly_state.collect_state(config, output, connect) == 0
    report = json.loads(output.read_text())
    assert report['clock_anchor_started']['clock'] == 'time.monotonic_ns'
    assert report['clock_anchor_completed']['wall_clock'] == 'time.time_ns'
    assert set(reader.calls) == set(readonly_state.READ_METHODS)
    assert report['hardware_stop_verified'] is False


async def test_missing_session_anchor_does_not_break_logging(tmp_path, monkeypatch):
    monkeypatch.setattr(commissioning, 'sample_clock_anchor', lambda: None)
    recorder = CommissioningRecorder(tmp_path, {}, session_id='missing-clock')
    await recorder.start()
    await recorder.close()
    events = [json.loads(line) for line in recorder.jsonl_path.read_text().splitlines()]
    assert [e['clock_anchor'] for e in events] == [None, None]
    assert recorder.fatal_error is None


async def test_missing_readonly_anchor_does_not_change_result_or_allowlist(setup, monkeypatch):
    monkeypatch.setattr(readonly_state, 'sample_clock_anchor', lambda: None)
    config, output, now = setup
    reader = Reader(now)
    async def connect(_):
        return reader
    assert await readonly_state.collect_state(config, output, connect) == 0
    report = json.loads(output.read_text())
    assert report['clock_anchor_started'] is report['clock_anchor_completed'] is None
    assert set(reader.calls) == set(readonly_state.READ_METHODS)
    assert report['hardware_stop_verified'] is False
