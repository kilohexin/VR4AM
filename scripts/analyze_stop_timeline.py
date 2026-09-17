"""Offline JSONL evidence extraction. No SDK imports or device access."""
import argparse
import hashlib
import json
from pathlib import Path


def deadline_evidence(events, start):
    request = next((e for e in events if e['kind'] == 'stop_rpc_lifecycle'
                    and e['method'] == 'stop_move'), None)
    if request is None:
        return None
    deadline = request['deadline_ns']
    reads = sorted((e for e in events if e['kind'] == 'stop_diagnostic_read'
                    and e.get('outcome') == 'sample'
                    and start <= e['started_ns'] <= e['completed_ns'] <= deadline),
                   key=lambda e: e['completed_ns'])
    states = [e['raw_robot_state'] for e in reads]
    return dict(
        deadline_ms=(deadline-start)/1e6,
        completed_reads=len(reads), raw_states=states,
        observed_span_ms=(reads[-1]['completed_ns']-reads[0]['completed_ns'])/1e6 if reads else None,
        settled_state_reads=sum(s in {'IDLE', 'STOP', 'PAUSED', 5, 6, 12} for s in states),
        max_abs_qd=max((abs(v) for e in reads for v in
                       e['raw_kinematics']['actual_joint_speed']), default=None),
        supports_deferred_escalation=None,
        scope='Completed diagnostic reads only; missing other safety fields are not presumed safe.')


def analyze(path):
    raw = path.read_bytes()
    events = [json.loads(line) for line in raw.splitlines() if line.strip()]
    start = next(e['server_mono_ns'] for e in events if e['kind'] == 'stop_requested')
    observations = [e for e in events if e['kind'] == 'stop_observation_sample']
    if not observations:
        raise ValueError('No observation samples; cannot calculate displacement')
    x0 = observations[0]['state']['actual_tcp']['p'][0]
    rows = []
    for e in observations:
        s = e['state']
        raw_kin = s.get('raw_kinematics', {})
        rows.append(dict(
            read_started_ms=(e['read_started_ns'] - start) / 1e6,
            read_finished_ms=(e['read_finished_ns'] - start) / 1e6,
            state=s['raw_robot_state'],
            dx_from_first_observation_mm=(s['actual_tcp']['p'][0] - x0) * 1000,
            max_abs_qd=max(abs(v) for v in s['actual_qd']),
            actual_torque=raw_kin.get('actual_joint_torque'),
            target_torque=raw_kin.get('target_joint_torque')))
    return dict(
        source_sha256=hashlib.sha256(raw).hexdigest(),
        event_count=len(events), observation_count=len(rows),
        stop_move_deadline_evidence=deadline_evidence(events, start),
        baseline='first stop_observation_sample actual_tcp.x; NOT pre-motion',
        final_dx_mm=rows[-1]['dx_from_first_observation_mm'],
        min_dx_mm=min(r['dx_from_first_observation_mm'] for r in rows),
        max_dx_mm=max(r['dx_from_first_observation_mm'] for r in rows),
        rpc=[dict(method=e['method'], outcome=e['outcome'],
                  start_ms=(e['started_ns'] - start) / 1e6,
                  completion_ms=(e['completed_ns'] - start) / 1e6,
                  duration_ms=(e['completed_ns'] - e['started_ns']) / 1e6)
             for e in events if e['kind'] == 'stop_rpc_lifecycle'],
        first_samples=rows[:15], last_sample=rows[-1],
        limitations=['Sequential SDK reads are not atomic.',
                     'Torque units/freshness and physical cause not established.',
                     'Recorded RPC completion is not physical stop confirmation.'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.session), ensure_ascii=False, indent=2))
