import asyncio
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import pytest

from scripts.analyze_session_lifecycle import analyze, analyze_events


MS = 1_000_000


def trace(**changes):
    event = dict(kind="teleop_session_lifecycle", server_mono_ns=200 * MS,
                 session_trace_id="synthetic-private-id", accepted_ns=0,
                 cancel_requested_ns=10 * MS, tasks_joined_ns=85 * MS,
                 disconnect_started_ns=90 * MS, disconnect_completed_ns=190 * MS,
                 owner_released_ns=200 * MS, prior_dropped_events=0)
    event.update(changes)
    return event


def stop(**changes):
    event = dict(kind="stop_diagnostics", reason="disconnect", episode_id=1,
                 started_ns=92 * MS, completed_ns=185 * MS,
                 stop_move_lock=dict(requested_ns=92 * MS, acquired_ns=123 * MS, wait_ns=31 * MS),
                 rpc_calls=[dict(method="stop_move", episode_id=1, request_id=1,
                                 started_ns=123 * MS, request_started_ns=123 * MS, reused=False)])
    event.update(changes)
    return event


def rpc(**changes):
    event = dict(kind="stop_rpc_lifecycle", method="stop_move", episode_id=1, request_id=1,
                 started_ns=123 * MS, deadline_ns=323 * MS, completed_ns=139 * MS,
                 sdk_await_started_ns=123 * MS, sdk_await_completed_ns=139 * MS)
    event.update(changes)
    return event


def first(events, **kwargs):
    return analyze_events(events, **kwargs)["sessions"][0]


def test_complete_chain_is_unique_observed_inference_with_separate_durations():
    result = analyze_events([rpc(), trace(), stop()])
    session = result["sessions"][0]
    assert session["stop_association"] == "unique_observed_candidate"
    assert session["rpc_association"] == "matched_request_identity"
    assert session["timing_ms"] == dict(
        cancel_to_join=75, post_join_gap=5, disconnect_total=100,
        cleanup_to_stop_entry=2, stop_lock_wait=31, cancel_to_rpc_created=113,
        rpc_deadline_window=200, sdk_await=16, sdk_return_after_deadline=0,
    )
    assert result["association_is_inference"] is True
    assert result["hardware_verified"] is False
    assert result["normal_log_drop_count"] is None
    assert "synthetic-private-id" not in json.dumps(result)


def test_latest_cumulative_row_recovers_earlier_boundaries_without_double_counting():
    early = dict(kind="teleop_session_lifecycle", server_mono_ns=10 * MS,
                 session_trace_id="synthetic-private-id", accepted_ns=0, cancel_requested_ns=10 * MS)
    result = first([trace(), early, trace(), stop(), stop(), rpc(), rpc()])
    assert result["trace_record_count"] == 3
    assert result["timing_ms"]["cancel_to_join"] == 75
    assert result["stop_association"] == "unique_observed_candidate"


def test_old_logs_do_not_gain_invented_session_boundaries():
    result = analyze_events([stop(), rpc()])
    assert result["sessions"] == []
    assert result["unassociated_stop_count"] == 1
    assert result["normal_log_drop_count"] is None


def test_partial_session_keeps_independent_durations_but_cannot_associate_stop():
    event = trace()
    del event["disconnect_completed_ns"]
    row = first([event, stop(), rpc()])
    assert row["stop_association"] == "incomplete_or_invalid_disconnect_window"
    assert row["timing_ms"]["cancel_to_join"] == 75
    assert row["timing_ms"]["disconnect_total"] is None
    assert row["timing_ms"]["sdk_await"] is None


@pytest.mark.parametrize("other", [stop(started_ns=95 * MS), stop(reason="fault", started_ns=95 * MS)])
def test_multiple_stop_candidates_or_interleaved_other_stop_are_not_forced(other):
    row = first([trace(), stop(), other, rpc()])
    assert row["stop_association"] == "ambiguous_stop_candidates"
    assert row["timing_ms"]["sdk_await"] is None


def test_non_disconnect_reason_is_not_used_as_session_cleanup_stop():
    row = first([trace(), stop(reason="home"), rpc()])
    assert row["stop_association"] == "no_disconnect_candidate"


@pytest.mark.parametrize("other", [trace(session_trace_id="overlap", accepted_ns=80 * MS),
                                      trace(session_trace_id="open", owner_released_ns=None)])
def test_overlapping_or_open_session_prevents_unique_temporal_association(other):
    rows = analyze_events([trace(), other, stop(), rpc()])["sessions"]
    assert rows[0]["stop_association"] == "ambiguous_session_coverage"


def test_conflicting_cumulative_boundary_is_unknown_instead_of_last_write_wins():
    row = first([trace(), trace(tasks_joined_ns=86 * MS), stop(), rpc()])
    assert row["boundaries_ns"]["tasks_joined_ns"] is None
    assert "conflicting_boundary:tasks_joined_ns" in row["issues"]
    assert row["timing_ms"]["cancel_to_join"] is None


def test_reversed_boundary_is_not_clamped_to_zero():
    row = first([trace(tasks_joined_ns=5 * MS), stop(), rpc()])
    assert row["timing_ms"]["cancel_to_join"] is None
    assert "reversed_interval:cancel_to_join" in row["issues"]


@pytest.mark.parametrize("value", [True, -1, 1.2, "10"])
def test_invalid_timestamp_is_rejected(value):
    with pytest.raises(ValueError, match="invalid_timestamp"):
        analyze_events([trace(cancel_requested_ns=value)])


def test_missing_identity_does_not_fall_back_to_nearest_rpc():
    event = stop()
    del event["rpc_calls"][0]["request_id"]
    row = first([trace(), event, rpc()])
    assert row["rpc_association"] == "missing_request_identity"
    assert row["timing_ms"]["stop_lock_wait"] == 31
    assert row["timing_ms"]["sdk_await"] is None


def test_conflicting_rpc_rows_are_not_silently_selected():
    row = first([trace(), stop(), rpc(), rpc(sdk_await_completed_ns=140 * MS)])
    assert row["rpc_association"] == "ambiguous_rpc_records"
    assert row["timing_ms"]["sdk_await"] is None


def test_reused_request_from_previous_episode_is_not_counted_as_new_rpc():
    event = stop()
    event["rpc_calls"][0].update(reused=True, request_started_ns=1 * MS)
    row = first([trace(), event, rpc(started_ns=1 * MS)])
    assert row["rpc_association"] == "reused_request"
    assert row["timing_ms"]["cancel_to_rpc_created"] is None


def test_request_identity_with_out_of_window_times_is_not_accepted():
    row = first([trace(), stop(), rpc(sdk_await_completed_ns=300 * MS, completed_ns=300 * MS)])
    assert row["rpc_association"] == "rpc_outside_stop_window"
    assert row["timing_ms"]["sdk_await"] is None


def test_conflicting_episode_identity_does_not_join_different_stop_transactions():
    row = first([trace(), stop(episode_id=2), rpc()])
    assert row["rpc_association"] == "conflicting_request_identity"


def test_rpc_cannot_be_created_before_its_stop_lock_was_acquired():
    event = stop(stop_move_lock=dict(requested_ns=92 * MS, acquired_ns=130 * MS, wait_ns=38 * MS))
    row = first([trace(), event, rpc()])
    assert row["rpc_association"] == "rpc_before_lock_acquired"
    assert row["timing_ms"]["sdk_await"] is None


def test_inconsistent_reported_lock_wait_is_unknown():
    event = stop()
    event["stop_move_lock"]["wait_ns"] = 99 * MS
    row = first([trace(), event, rpc()])
    assert row["timing_ms"]["stop_lock_wait"] is None
    assert "conflicting_lock_wait" in row["issues"]


def test_future_cumulative_boundary_is_unknown():
    row = first([trace(owner_released_ns=999 * MS)])
    assert row["boundaries_ns"]["owner_released_ns"] is None
    assert "future_boundary:owner_released_ns" in row["issues"]


def test_late_sdk_return_and_drop_evidence_remain_distinct():
    row = first([trace(server_mono_ns=900 * MS, disconnect_completed_ns=850 * MS,
                       owner_released_ns=900 * MS, prior_dropped_events=3),
                 stop(completed_ns=800 * MS),
                 rpc(sdk_await_completed_ns=732 * MS, completed_ns=732 * MS)], normal_log_drop_count=7)
    assert row["timing_ms"]["sdk_await"] == 609
    assert row["timing_ms"]["sdk_return_after_deadline"] == 409
    assert row["prior_trace_drops_max"] == 3
    assert "recording_loss_observed" in row["issues"]


def test_cli_hash_binding_and_source_preservation(tmp_path):
    path = tmp_path / "synthetic.jsonl"
    raw = "\n".join(json.dumps(e) for e in [trace(), stop(), rpc()]).encode()
    path.write_bytes(raw)
    digest = hashlib.sha256(raw).hexdigest()
    assert analyze(path, expected_sha256=digest)["source_sha256"] == digest
    with pytest.raises(ValueError, match="source_hash_mismatch"):
        analyze(path, expected_sha256="0" * 64)
    script = Path(__file__).resolve().parents[3] / "scripts/analyze_session_lifecycle.py"
    result = subprocess.run([sys.executable, str(script), str(path), "--expected-sha256", digest],
                            capture_output=True, text=True, check=True)
    assert json.loads(result.stdout)["sessions"][0]["timing_ms"]["sdk_await"] == 16
    assert path.read_bytes() == raw


@pytest.mark.parametrize("raw", [b"{bad}\n", b'{"kind":"x","value":NaN}\n', b"[]\n", b'{"kind":[]}\n'])
def test_malformed_jsonl_does_not_emit_a_plausible_partial_report(tmp_path, raw):
    path = tmp_path / "bad.jsonl"
    path.write_bytes(raw)
    with pytest.raises(ValueError):
        analyze(path)


def test_explicit_recording_summary_is_hashed_and_not_assumed_zero(tmp_path):
    path, summary = tmp_path / "session.jsonl", tmp_path / "summary.json"
    path.write_text(json.dumps(trace()))
    summary.write_text(json.dumps(dict(dropped_normal_events=7)))
    result = analyze(path, summary_path=summary)
    assert result["normal_log_drop_count"] == 7
    assert result["summary_sha256"] == hashlib.sha256(summary.read_bytes()).hexdigest()
    assert "recording_loss_observed" in result["sessions"][0]["issues"]


async def test_current_fake_session_jsonl_roundtrips_into_summary(monkeypatch, tmp_path):
    from fastapi import WebSocketDisconnect
    from app.diagnostics.recorder import DiagnosticsRecorder
    from app.diagnostics.store import DiagnosticsStore
    from app.recording.commissioning import CommissioningRecorder
    from tests.api.test_observer_session_lifecycle import session_probe
    from tests.robots.test_lebai_adapter_control import _wait_until

    async with session_probe(monkeypatch) as (probe, adapter, client):
        primary = CommissioningRecorder(tmp_path, {}, session_id="synthetic-roundtrip")
        recorder = DiagnosticsRecorder(primary, DiagnosticsStore())
        probe.control.recorder = recorder
        original_event = adapter._event_callback

        async def record_event(event, timestamp):
            await recorder.write_event(event, timestamp)
            await original_event(event, timestamp)

        adapter._event_callback = record_event
        probe.release_cleanup.set()
        await probe.start()
        probe.observer.clock.advance_ms(7)
        probe.socket.incoming.put_nowait(WebSocketDisconnect())
        await asyncio.wait_for(probe.session, 1)
        await _wait_until(lambda: any(e["kind"] == "stop_rpc_lifecycle" for e in primary._normal))
        await primary.start()
        await primary.close()
        result = analyze(primary.jsonl_path, summary_path=primary.session_dir / "summary.json")
        row = result["sessions"][0]
        assert row["stop_association"] == "unique_observed_candidate"
        assert row["rpc_association"] == "matched_request_identity"
        assert row["timing_ms"]["cancel_to_join"] == 0
        assert row["timing_ms"]["stop_lock_wait"] == 0
        assert row["timing_ms"]["rpc_deadline_window"] == 200
        assert row["issues"] == []
        assert result["normal_log_drop_count"] == 0
        assert result["hardware_verified"] is False
