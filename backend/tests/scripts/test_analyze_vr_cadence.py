import hashlib
import json

import pytest

from scripts.analyze_vr_cadence import analyze, analyze_events


def frame(seq, ns, *, session="synthetic", grip=True, client_ms=None):
    return dict(kind="vr_frame", server_mono_ns=ns,
                frame=dict(session_id=session, seq=seq,
                           client_mono_ms=ns / 1e6 if client_ms is None else client_ms,
                           right=dict(grip=grip)))


def pvat(seq, ns, timing=None):
    return dict(kind="pvat_sent", server_mono_ns=ns, command_id=seq,
                pvat_mode="advance", timing_ms=timing or {})


def test_sorts_receipt_and_emission_times_without_conflating_rates():
    events = [pvat(3, 80_000_000), frame(3, 40_000_000),
              frame(1, 0), pvat(1, 50_000_000), frame(2, 20_000_000, grip=False)]
    report = analyze_events(events, 0, 100_000_000)
    assert report["frames"]["count"] == 3
    assert report["frames"]["grip_true"] == 2
    assert report["frames"]["server_receipt_intervals_ms"]["mean"] == 20
    assert report["frames"]["observed_receipt_rate_hz"] == 50
    assert report["pvat"]["event_intervals_ms"]["mean"] == 30
    assert report["pvat"]["matched_frame_count"] == 2
    assert report["pvat"]["receipt_to_event_ms"]["mean"] == 45
    assert report["normal_log_drop_count"] is None


def test_sequence_gaps_are_not_called_network_packet_loss():
    report = analyze_events([frame(4, 0), frame(7, 30_000_000)], 0, 40_000_000)
    assert report["frames"]["unobserved_sequence_numbers"] == 2
    assert report["frames"]["sequence_delta_counts"] == {"3": 1}
    assert "packet_loss" not in json.dumps(report)


def test_zero_intervals_and_negative_client_deltas_are_retained():
    report = analyze_events([frame(1, 0, client_ms=10), frame(2, 0, client_ms=9)], 0, 1)
    assert report["frames"]["server_receipt_intervals_ms"]["min"] == 0
    assert report["frames"]["client_intervals_ms"]["min"] == -1
    assert report["frames"]["observed_receipt_rate_hz"] is None


@pytest.mark.parametrize("events,reason", [
    ([frame(1, 0), frame(2, 1, session="other")], "multiple_frame_sessions"),
    ([frame(1, 0), frame(1, 1)], "duplicate_frame_sequence"),
    ([frame(2, 0), frame(1, 1)], "nonincreasing_frame_sequence"),
])
def test_rejects_ambiguous_frame_identity(events, reason):
    with pytest.raises(ValueError, match=reason):
        analyze_events(events, 0, 10)


def test_empty_and_missing_fields_remain_unknown():
    report = analyze_events([pvat(99, 1)], 0, 10)
    assert report["frames"]["count"] == 0
    assert report["frames"]["server_receipt_intervals_ms"]["mean"] is None
    assert report["pvat"]["unmatched_frame_count"] == 1
    assert report["pvat"]["receipt_to_event_ms"]["count"] == 0
    assert report["timing_ms"]["snapshot_read"]["mean"] is None


def test_snapshot_lock_wait_is_not_added_twice_and_rpc_start_is_distinct():
    timing = dict(snapshot_read=30, snapshot_lock_wait=10, command_lock_wait=5,
                  candidate_selection=7, send_sdk=8, handler_to_send_complete=50)
    events = [pvat(1, 60_000_000, timing),
              dict(kind="sdk_request_lifecycle", method="move_pvat", command_id=1,
                   server_mono_ns=60_000_000, started_ns=52_000_000, completed_ns=60_000_000)]
    report = analyze_events(events, 0, 100_000_000)
    assert report["timing_ms"]["accounting_residual"]["mean"] == 0
    assert report["sdk_move_pvat"]["await_ms"]["mean"] == 8
    assert report["timing_ms"]["snapshot_lock_wait"]["mean"] == 10


def test_source_hash_and_input_bytes_are_preserved(tmp_path):
    source = tmp_path / "synthetic.jsonl"
    raw = (json.dumps(frame(1, 0)) + "\n").encode()
    source.write_bytes(raw)
    expected = hashlib.sha256(raw).hexdigest()
    assert analyze(source, 0, 10, expected)["source_sha256"] == expected
    assert source.read_bytes() == raw
    with pytest.raises(ValueError, match="source_hash_mismatch"):
        analyze(source, 0, 10, "0" * 64)


def test_rejects_invalid_window():
    with pytest.raises(ValueError, match="invalid_window"):
        analyze_events([], 10, 10)
