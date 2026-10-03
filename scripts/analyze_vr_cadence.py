"""Summarize an explicit frozen JSONL window; stdlib only, no robot access.

VR event times are receipt timestamps logged by the control consumer. PVAT event
times follow local send completion. Neither is a controller execution timestamp.
Only aggregate timing/counts are emitted; no poses, credentials or session IDs.
"""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path


TIMINGS = (
    "snapshot_read", "snapshot_lock_wait", "command_lock_wait",
    "candidate_selection", "send_sdk", "handler_to_send_complete",
    "snapshot_age_at_send",
)


def stats(values):
    values = sorted(values)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite_timing")
    def percentile(fraction):
        index = (len(values) - 1) * fraction
        low = math.floor(index)
        high = math.ceil(index)
        return values[low] + (values[high] - values[low]) * (index - low)
    return dict(count=len(values), min=values[0] if values else None,
                p50=percentile(0.5) if values else None,
                p95=percentile(0.95) if values else None,
                max=values[-1] if values else None,
                mean=sum(values) / len(values) if values else None)


def deltas(values):
    return [b - a for a, b in zip(values, values[1:])]


def rate(timestamps):
    span = timestamps[-1] - timestamps[0] if len(timestamps) > 1 else 0
    return (len(timestamps) - 1) * 1e9 / span if span > 0 else None


def analyze_events(events, start_ns, end_ns):
    if start_ns >= end_ns:
        raise ValueError("invalid_window")
    window = sorted((e for e in events if start_ns <= e["server_mono_ns"] <= end_ns),
                    key=lambda e: e["server_mono_ns"])
    frames = [e for e in window if e["kind"] == "vr_frame"]
    if len({e["frame"]["session_id"] for e in frames}) > 1:
        raise ValueError("multiple_frame_sessions")
    sequence = [e["frame"]["seq"] for e in frames]
    if len(set(sequence)) != len(sequence):
        raise ValueError("duplicate_frame_sequence")
    gaps = deltas(sequence)
    if any(gap <= 0 for gap in gaps):
        raise ValueError("nonincreasing_frame_sequence")
    by_seq = {e["frame"]["seq"]: e for e in frames}
    receipt_times = [e["server_mono_ns"] for e in frames]
    pvat = [e for e in window if e["kind"] == "pvat_sent"]
    pvat_times = [e["server_mono_ns"] for e in pvat]
    matched = [e for e in pvat if e["command_id"] in by_seq]
    timing = {k: stats(e["timing_ms"][k] for e in pvat
                      if e.get("timing_ms", {}).get(k) is not None) for k in TIMINGS}
    parts = ("snapshot_read", "command_lock_wait", "candidate_selection", "send_sdk")
    # snapshot_read already includes snapshot_lock_wait. Never add it twice.
    timing["accounting_residual"] = stats(
        e["timing_ms"]["handler_to_send_complete"] - sum(e["timing_ms"][k] for k in parts)
        for e in pvat if all(e.get("timing_ms", {}).get(k) is not None
                             for k in (*parts, "handler_to_send_complete")))
    rpc = sorted((e for e in window if e["kind"] == "sdk_request_lifecycle"
                  and e.get("method") == "move_pvat" and e.get("started_ns") is not None),
                 key=lambda e: e["started_ns"])
    rpc_times = [e["started_ns"] for e in rpc]
    return dict(
        window=dict(start_ns=start_ns, end_ns=end_ns, duration_s=(end_ns - start_ns) / 1e9,
                    bounds="inclusive", event_count=len(window)),
        normal_log_drop_count=None,
        frames=dict(count=len(frames), grip_true=sum(e["frame"]["right"]["grip"] for e in frames),
                    first_seq=sequence[0] if sequence else None,
                    last_seq=sequence[-1] if sequence else None,
                    unobserved_sequence_numbers=sum(gap - 1 for gap in gaps),
                    sequence_delta_counts={str(k): v for k, v in sorted(Counter(gaps).items())},
                    server_receipt_intervals_ms=stats(v / 1e6 for v in deltas(receipt_times)),
                    client_intervals_ms=stats(deltas([e["frame"]["client_mono_ms"] for e in frames])),
                    observed_receipt_rate_hz=rate(receipt_times)),
        pvat=dict(count=len(pvat), modes=dict(Counter(e.get("pvat_mode", "unknown") for e in pvat)),
                  event_intervals_ms=stats(v / 1e6 for v in deltas(pvat_times)),
                  observed_event_rate_hz=rate(pvat_times),
                  previous_call_gap_ms=stats(e["previous_pvat_gap_ms"] for e in pvat
                                            if e.get("previous_pvat_gap_ms") is not None),
                  matched_frame_count=len(matched), unmatched_frame_count=len(pvat) - len(matched),
                  receipt_to_event_ms=stats((e["server_mono_ns"] - by_seq[e["command_id"]]["server_mono_ns"]) / 1e6
                                           for e in matched)),
        sdk_move_pvat=dict(count=len(rpc), start_intervals_ms=stats(v / 1e6 for v in deltas(rpc_times)),
                           observed_start_rate_hz=rate(rpc_times),
                           await_ms=stats((e["completed_ns"] - e["started_ns"]) / 1e6
                                          for e in rpc if e.get("completed_ns") is not None)),
        timing_ms=timing,
        limitations=[
            "VR frames are consumer-logged receipt timestamps, not every received frame or tick start.",
            "Sequence gaps may involve latest-only coalescing or recording loss; source receipt coverage is unknown.",
            "Command matching requires one observed frame session and unique increasing sequences in this window.",
            "PVAT events and SDK returns do not establish device execution or physical feedback freshness.",
            "Missing timing and normal-log-drop evidence remain unknown; window boundaries can exclude related events.",
        ],
    )


def analyze(path, start_ns, end_ns, expected_sha256=None):
    raw = Path(path).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and digest != expected_sha256:
        raise ValueError("source_hash_mismatch")
    report = analyze_events([json.loads(line) for line in raw.splitlines() if line.strip()], start_ns, end_ns)
    return dict(source_sha256=digest, **report)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--start-ns", type=int, required=True)
    parser.add_argument("--end-ns", type=int, required=True)
    parser.add_argument("--expected-sha256")
    args = parser.parse_args()
    print(json.dumps(analyze(args.session, args.start_ns, args.end_ns, args.expected_sha256),
                     ensure_ascii=False, allow_nan=False, indent=2))
