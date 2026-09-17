"""Offline audit must never use future feedback to justify a past decision."""
import importlib.util
import json
from pathlib import Path


spec = importlib.util.spec_from_file_location(
    'stop_timeline_audit', Path(__file__).resolve().parents[3] / 'scripts/analyze_stop_timeline.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def test_deadline_audit_excludes_inflight_and_future_reads(tmp_path):
    events = [dict(kind='stop_requested', server_mono_ns=0)]
    for start, end, state in [(0, 16, 'MOVING'), (125, 141, 'MOVING'),
                               (190, 220, 'STOPPING'), (250, 282, 'STOPPING')]:
        events.append(dict(kind='stop_diagnostic_read', started_ns=start*1000000,
                           completed_ns=end*1000000, outcome='sample',
                           raw_robot_state=state, raw_kinematics={'actual_joint_speed': [0.]*6}))
    events.append(dict(kind='stop_rpc_lifecycle', method='stop_move', started_ns=0,
                       deadline_ns=200000000, completed_ns=625000000, outcome='returned_late'))
    events.append(dict(kind='stop_observation_sample', read_started_ns=400000000,
                       read_finished_ns=420000000, state=dict(raw_robot_state='STOPPING',
                       actual_tcp={'p': [0., 0., 0.]}, actual_qd=[0.]*6)))
    path = tmp_path / 'session.jsonl'
    path.write_text('\n'.join(json.dumps(e) for e in events))
    result = audit.analyze(path)['stop_move_deadline_evidence']
    assert result['completed_reads'] == 2
    assert result['raw_states'] == ['MOVING', 'MOVING']
    assert result['observed_span_ms'] == 125
    assert result['settled_state_reads'] == 0
    assert result['max_abs_qd'] == 0
    assert result['supports_deferred_escalation'] is None  # Audit is not a safety policy.
