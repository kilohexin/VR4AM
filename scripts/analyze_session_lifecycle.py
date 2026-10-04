"""Read-only JSONL lifecycle/stop summary. Stdlib only; no robot or network access.

Temporal association is an inference over observed records, never a safety claim.
Raw session IDs, payloads, poses and exception text are not included in output.
"""

import argparse
import hashlib
import json
from pathlib import Path


BOUNDARIES = (
    "accepted_ns", "tasks_started_ns", "cancel_requested_ns", "tasks_joined_ns",
    "disconnect_started_ns", "disconnect_completed_ns", "owner_released_ns",
)
METRICS = (
    "cancel_to_join", "post_join_gap", "disconnect_total", "cleanup_to_stop_entry",
    "stop_lock_wait", "cancel_to_rpc_created", "rpc_deadline_window", "sdk_await",
    "sdk_return_after_deadline",
)
KINDS = {"teleop_session_lifecycle", "stop_diagnostics", "stop_rpc_lifecycle"}


def validate_times(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key.endswith("_ns") and item is not None:
                if type(item) is not int or not 0 <= item < 2**63:
                    raise ValueError("invalid_timestamp")
            validate_times(item)
    elif isinstance(value, list):
        for item in value:
            validate_times(item)


def drop_count(value):
    if value is not None and (type(value) is not int or value < 0):
        raise ValueError("invalid_drop_count")
    return value


def unique(rows):
    return list({json.dumps(row, sort_keys=True, allow_nan=False): row for row in rows}.values())


def duration(start, end, label, issues):
    if start is None or end is None:
        return None
    if end < start:
        issues.append(f"reversed_interval:{label}")
        return None
    return (end - start) / 1e6


def interval(start, end):
    return start is not None and end is not None and start <= end


def merge_trace(rows):
    issues, boundaries = [], {}
    for key in BOUNDARIES:
        values = {row[key] for row in rows if row.get(key) is not None}
        if len(values) > 1:
            issues.append(f"conflicting_boundary:{key}")
        future = any(row.get(key) is not None and row[key] > row["server_mono_ns"] for row in rows)
        if future:
            issues.append(f"future_boundary:{key}")
        boundaries[key] = next(iter(values)) if len(values) == 1 and not future else None
    drops = [drop_count(row.get("prior_dropped_events")) for row in rows]
    return dict(trace_record_count=len(rows), boundaries_ns=boundaries,
                prior_trace_drops_max=max((v for v in drops if v is not None), default=None),
                timing_ms=dict.fromkeys(METRICS), issues=issues,
                stop_association="not_evaluated", rpc_association="not_evaluated")


def request_key(row):
    keys = (row.get("episode_id"), row.get("request_id"))
    if any(type(value) is not int or value < 0 for value in keys):
        return None
    return keys


def attach_rpc(session, stop, rpcs):
    metrics, issues = session["timing_ms"], session["issues"]
    lock = stop.get("stop_move_lock") or {}
    if not isinstance(lock, dict):
        raise ValueError("invalid_lock_record")
    lock_start, lock_end = lock.get("requested_ns"), lock.get("acquired_ns")
    if interval(lock_start, lock_end) and not (
        stop["started_ns"] <= lock_start <= lock_end <= stop["completed_ns"]
    ):
        issues.append("lock_outside_stop_window")
    elif interval(lock_start, lock_end) and lock.get("wait_ns") not in {None, lock_end - lock_start}:
        issues.append("conflicting_lock_wait")
    else:
        metrics["stop_lock_wait"] = duration(lock_start, lock_end, "stop_lock_wait", issues)

    calls = stop.get("rpc_calls", [])
    if not isinstance(calls, list) or any(not isinstance(c, dict) for c in calls):
        raise ValueError("invalid_rpc_calls")
    calls = [c for c in calls if c.get("method") == "stop_move"]
    if len(calls) != 1:
        session["rpc_association"] = "missing_request_identity" if not calls else "ambiguous_request_identity"
        return
    call = calls[0]
    key, started = request_key(call), call.get("request_started_ns")
    if key is None or started is None:
        session["rpc_association"] = "missing_request_identity"
        return
    if type(stop.get("episode_id")) is not int or key[0] != stop["episode_id"]:
        session["rpc_association"] = "conflicting_request_identity"
        return
    if call.get("reused") is True or started < stop["started_ns"]:
        session["rpc_association"] = "reused_request"
        return
    matches = [r for r in rpcs if r.get("method") == "stop_move"
               and request_key(r) == key and r.get("started_ns") == started]
    if len(matches) != 1:
        session["rpc_association"] = "missing_rpc_record" if not matches else "ambiguous_rpc_records"
        return
    rpc = matches[0]
    observed = [rpc.get(k) for k in ("started_ns", "sdk_await_started_ns", "sdk_await_completed_ns", "completed_ns")]
    known = [v for v in observed if v is not None]
    if any(not stop["started_ns"] <= value <= stop["completed_ns"] for value in known):
        session["rpc_association"] = "rpc_outside_stop_window"
        return
    if known != sorted(known):
        session["rpc_association"] = "invalid_rpc_order"
        return
    if lock_end is not None and started < lock_end:
        session["rpc_association"] = "rpc_before_lock_acquired"
        return
    session["rpc_association"] = "matched_request_identity"
    metrics["cancel_to_rpc_created"] = duration(session["boundaries_ns"]["cancel_requested_ns"], started,
                                               "cancel_to_rpc_created", issues)
    metrics["rpc_deadline_window"] = duration(started, rpc.get("deadline_ns"), "rpc_deadline_window", issues)
    metrics["sdk_await"] = duration(rpc.get("sdk_await_started_ns"), rpc.get("sdk_await_completed_ns"), "sdk_await", issues)
    if (rpc.get("sdk_await_completed_ns") is not None and metrics["rpc_deadline_window"] is not None):
        metrics["sdk_return_after_deadline"] = max(0, rpc["sdk_await_completed_ns"] - rpc["deadline_ns"]) / 1e6


def analyze_events(events, *, normal_log_drop_count=None):
    drop_count(normal_log_drop_count)
    groups, stops, rpcs = {}, [], []
    for row in events:
        if not isinstance(row, dict):
            raise ValueError("invalid_event_object")
        if not isinstance(row.get("kind"), str):
            raise ValueError("invalid_event_kind")
        if row["kind"] not in KINDS:
            continue
        validate_times(row)
        if row["kind"] == "teleop_session_lifecycle":
            trace_id = row.get("session_trace_id")
            if not isinstance(trace_id, str) or not 0 < len(trace_id) <= 128:
                raise ValueError("invalid_trace_identity")
            if row.get("server_mono_ns") is None:
                raise ValueError("missing_trace_event_timestamp")
            groups.setdefault(trace_id, []).append(row)
        elif row["kind"] == "stop_diagnostics":
            stops.append(row)
        else:
            rpcs.append(row)
    stops, rpcs = unique(stops), unique(rpcs)
    ordered = sorted(groups.values(), key=lambda rows: min(row["server_mono_ns"] for row in rows))
    sessions = [merge_trace(rows) for rows in ordered]
    used = set()
    for index, session in enumerate(sessions):
        session["session_index"] = index + 1
        b, metrics, issues = session["boundaries_ns"], session["timing_ms"], session["issues"]
        if (normal_log_drop_count or 0) > 0 or (session["prior_trace_drops_max"] or 0) > 0:
            issues.append("recording_loss_observed")
        for label, start, end in (
            ("cancel_to_join", "cancel_requested_ns", "tasks_joined_ns"),
            ("post_join_gap", "tasks_joined_ns", "disconnect_started_ns"),
            ("disconnect_total", "disconnect_started_ns", "disconnect_completed_ns"),
        ):
            metrics[label] = duration(b[start], b[end], label, issues)
        start, end = b["disconnect_started_ns"], b["disconnect_completed_ns"]
        if not interval(start, end):
            session["stop_association"] = "incomplete_or_invalid_disconnect_window"
            continue
        if (not interval(b["accepted_ns"], b["owner_released_ns"])
                or not b["accepted_ns"] <= start <= end <= b["owner_released_ns"]):
            session["stop_association"] = "ambiguous_session_coverage"
            continue
        overlap = False
        for other in sessions:
            if other is session:
                continue
            ob = other["boundaries_ns"]
            if not interval(ob["accepted_ns"], ob["owner_released_ns"]) or (
                ob["accepted_ns"] <= end and ob["owner_released_ns"] >= start
            ):
                overlap = True
        if overlap:
            session["stop_association"] = "ambiguous_session_coverage"
            continue
        if any(s.get("started_ns") is None for s in stops):
            session["stop_association"] = "incomplete_stop_window"
            continue
        candidates = [(i, s) for i, s in enumerate(stops) if s["started_ns"] <= end
                      and (s.get("completed_ns") is None or s["completed_ns"] >= start)]
        if len(candidates) > 1:
            session["stop_association"] = "ambiguous_stop_candidates"
            continue
        if not candidates or candidates[0][1].get("reason") != "disconnect":
            session["stop_association"] = "no_disconnect_candidate"
            continue
        stop_index, stop = candidates[0]
        if not interval(stop.get("started_ns"), stop.get("completed_ns")) or not (
            start <= stop["started_ns"] <= stop["completed_ns"] <= end
        ):
            session["stop_association"] = "incomplete_stop_window"
            continue
        session["stop_association"] = "unique_observed_candidate"
        used.add(stop_index)
        metrics["cleanup_to_stop_entry"] = duration(start, stop["started_ns"], "cleanup_to_stop_entry", issues)
        attach_rpc(session, stop, rpcs)
    return dict(
        source_format="commissioning_jsonl", hardware_verified=False, association_is_inference=True,
        normal_log_drop_count=normal_log_drop_count, sessions=sessions,
        observed_stop_count=len(stops), unassociated_stop_count=len(stops) - len(used),
        observed_rpc_count=len(rpcs),
        limitations=[
            "Only observed records from one recording file are considered; absent records may hide other candidates.",
            "Stop association uses interval and reason, not a session foreign key. Unique observed is not certain attribution.",
            "Task times are callback observations. Post-join gap is not exclusively rehearsal time.",
            "RPC identity includes episode, request and creation time; reused requests are not new session RPCs.",
            "Missing/conflicting data stays null. Source freshness, remote cancellation and physical stop are unverified.",
        ],
    )


def read_json(raw):
    def reject_constant(_value):
        raise ValueError("nonfinite_json")
    return json.loads(raw, parse_constant=reject_constant)


def analyze(path, *, expected_sha256=None, summary_path=None):
    raw = Path(path).read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if expected_sha256 is not None and expected_sha256 != digest:
        raise ValueError("source_hash_mismatch")
    rows = [read_json(line) for line in raw.splitlines() if line.strip()]
    drops, summary_hash = None, None
    if summary_path is not None:
        summary_raw = Path(summary_path).read_bytes()
        summary = read_json(summary_raw)
        if not isinstance(summary, dict):
            raise ValueError("invalid_recording_summary")
        drops = drop_count(summary.get("dropped_normal_events"))
        summary_hash = hashlib.sha256(summary_raw).hexdigest()
    return dict(source_sha256=digest, summary_sha256=summary_hash,
                **analyze_events(rows, normal_log_drop_count=drops))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--recording-summary", type=Path)
    args = parser.parse_args()
    try:
        report = analyze(args.session, expected_sha256=args.expected_sha256,
                         summary_path=args.recording_summary)
    except (ValueError, OSError):
        parser.exit(2, "Invalid or unreadable input; no report generated.\n")
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
