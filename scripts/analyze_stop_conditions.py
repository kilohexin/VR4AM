"""Offline stop RPC comparison; never a motion or stop authorization."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def timestamp(value):
    return value if type(value) is int and value >= 0 else None


def elapsed(start, end):
    start, end = timestamp(start), timestamp(end)
    return (end - start) / 1e6 if start is not None and end is not None and end >= start else None


def pre_state(events, reference_ns):
    if timestamp(reference_ns) is None:
        return None
    candidates = []
    for event in events:
        if event.get('kind') != 'robot_kinematics':
            continue
        start = timestamp(event.get('server_mono_ns'))
        if start is None or start >= reference_ns:
            continue
        if 'snapshot_read_completed_ns' in event:
            end = timestamp(event['snapshot_read_completed_ns'])
            # Equal coarse clock values do not establish event order.
            if end is None or not start <= end < reference_ns:
                continue
            basis, sampled_ns, age = 'completed_read', end, (reference_ns - end) / 1e6
        else:
            basis, sampled_ns, age = 'legacy_read_start_only', start, None
        velocity = event.get('actual_qd')
        max_qd = (max(abs(value) for value in velocity)
                  if isinstance(velocity, list) and len(velocity) == 6
                  and all(type(value) in (int, float) and math.isfinite(value) for value in velocity)
                  else None)
        candidates.append((sampled_ns, {
            'raw_robot_state': event.get('raw_robot_state'), 'max_abs_qd': max_qd,
            'basis': basis, 'sampled_ns': sampled_ns, 'reference_ns': reference_ns,
            'read_age_ms': age,
        }))
    if not candidates:
        return None
    newest_ns = max(item[0] for item in candidates)
    newest = [item[1] for item in candidates if item[0] == newest_ns]
    if all(item == newest[0] for item in newest):
        return newest[0]
    return {'raw_robot_state': None, 'max_abs_qd': None,
            'basis': 'ambiguous_same_timestamp', 'sampled_ns': newest_ns,
            'reference_ns': reference_ns, 'read_age_ms': None,
            'candidate_count': len(newest)}


def analyze_session(path):
    path = Path(path)
    raw = path.read_bytes()
    try:
        events = [json.loads(line) for line in raw.decode('utf-8-sig').splitlines() if line.strip()]
    except (UnicodeError, json.JSONDecodeError):
        raise ValueError('invalid JSONL evidence') from None
    if not all(isinstance(event, dict) for event in events):
        raise ValueError('invalid JSONL event')
    headers = [event for event in events if event.get('kind') == 'session_started']
    metadata = headers[0].get('metadata', {}) if len(headers) == 1 else {}
    if (not isinstance(metadata, dict) or metadata.get('backend') != 'LEBAI'
        or metadata.get('real_robot_mode') != 'control'
        or metadata.get('runtime') == 'SIMULATION' or metadata.get('hardware_verified') is False):
        raise ValueError('session is not eligible for field comparison')
    diagnostics = [event for event in events if event.get('kind') == 'stop_diagnostics']
    unique, duplicates = {}, 0
    for event in events:
        if event.get('kind') != 'stop_rpc_lifecycle' or event.get('method') != 'stop_move':
            continue
        identity = (event.get('episode_id'), event.get('request_id'))
        if not all(type(value) is int and value > 0 for value in identity):
            raise ValueError('invalid stop RPC identity')
        if identity in unique:
            if unique[identity] != event:
                raise ValueError('conflicting stop RPC lifecycle')
            duplicates += 1
        else:
            unique[identity] = event
    rows = []
    for (episode_id, request_id), event in unique.items():
        related = [item for item in diagnostics if item.get('episode_id') == episode_id]
        started = timestamp(event.get('started_ns'))
        initial = [item for item in related if timestamp(item.get('started_ns')) is not None
                   and started is not None and item['started_ns'] <= started]
        diagnostic = min(initial, key=lambda item: item['started_ns']) if initial else {}
        reference_ns = diagnostic.get('started_ns', started)
        timing = event.get('diagnostic_timing', {})
        if not isinstance(timing, dict):
            timing = {}
        rows.append({
            'episode_id': episode_id, 'request_id': request_id, 'outcome': event.get('outcome'),
            'reason': diagnostic.get('reason'), 'diagnostic_count_for_episode': len(related),
            'sdk_await_ms': elapsed(event.get('sdk_await_started_ns'), event.get('sdk_await_completed_ns')),
            'rpc_elapsed_ms': elapsed(started, event.get('completed_ns')),
            'sdk_await_qpc_ms': (elapsed(timing.get('sdk_await_started_ns'), timing.get('sdk_await_completed_ns'))
                                 if timing.get('clock') == 'time.perf_counter_ns' else None),
            'stop_entry': diagnostic.get('stop_entry'),
            'pre_state': pre_state(events, reference_ns),
        })
    return {
        'source_path': str(path.resolve()), 'source_sha256': hashlib.sha256(raw).hexdigest(),
        'source_bytes': len(raw), 'event_count': len(events),
        'rpc_count': len(rows), 'diagnostic_count': len(diagnostics),
        'duplicate_lifecycle_count': duplicates, 'rpcs': rows, 'hardware_stop_verified': False,
    }


def analyze_sessions(paths):
    sessions, seen, aliases = [], set(), 0
    for path in paths:
        session = analyze_session(path)
        if session['source_sha256'] in seen:
            aliases += 1
            continue
        seen.add(session['source_sha256'])
        sessions.append(session)
    return {'schema': 1, 'offline_only': True, 'hardware_stop_verified': False,
            'sessions': sessions, 'session_count': len(sessions),
            'duplicate_source_count': aliases,
            'rpc_count': sum(item['rpc_count'] for item in sessions),
            'diagnostic_count': sum(item['diagnostic_count'] for item in sessions),
            'limitations': ['Raw state is recorded telemetry, not physical standstill.',
                            'Legacy read start alone does not prove read completed before stop.',
                            'Different sessions, software and time sources are not controlled causal comparisons.']}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sessions', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output exists; evidence will not be overwritten')
    result = analyze_sessions(args.sessions)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in ('session_count', 'rpc_count', 'diagnostic_count', 'duplicate_source_count')}))


if __name__ == '__main__':
    main()
