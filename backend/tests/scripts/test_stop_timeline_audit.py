"""Offline audit must never use future feedback to justify a past decision."""
import importlib.util
import json
from pathlib import Path

import pytest


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


def finite_events():
    ms = 1_000_000
    return [
        dict(kind='session_started', metadata={'event_clock': 'time.perf_counter_ns'}),
        dict(kind='robot_kinematics', snapshot_read_requested_ns=80*ms,
             snapshot_read_started_ns=81*ms,snapshot_read_completed_ns=90*ms,
             actual_q=[0.]*6,actual_qd=[0.]*6,actual_qdd=[0.]*6,
             actual_tcp={'p': [0., 0., .4]}),
        dict(kind='pvat_sent', p=[.00013, 0., 0., 0., 0., 0.], horizon_s=.08,
             point_inputs={'snapshot_read_requested_ns':80*ms,'snapshot_read_started_ns':81*ms,
                 'snapshot_read_completed_ns': 90*ms, 'actual_q': [0.]*6,
                 'actual_qd':[0.]*6,'actual_qdd':[0.]*6}),
        dict(kind='stop_diagnostics', episode_id=1, started_ns=180*ms),
        dict(kind='stop_rpc_lifecycle', episode_id=1, method='stop_move',
             started_ns=180*ms, deadline_ns=380*ms, completed_ns=398*ms,
             outcome='returned_late', diagnostic_timing={
                 'clock': 'time.perf_counter_ns', 'sdk_await_started_ns': 180*ms,
                 'sdk_await_completed_ns': 398*ms}),
        dict(kind='stop_observation_sample', read_started_ns=400*ms,
             read_finished_ns=420*ms, state=dict(raw_robot_state='IDLE',
             actual_tcp={'p': [0., 0., .4]}, actual_q=[0.]*6, actual_qd=[0.]*6)),
        dict(kind='finite_scope_result', event_clock='time.perf_counter_ns', pvat_entries=1,
             stop_confirmed=False, latched_fault='stop_unverified', pvat_call={
                 'native_started_ns': 100*ms, 'outcome': 'returned'}),
    ]


def write_session(tmp_path, events):
    path=tmp_path/'session.jsonl'
    path.write_text('\n'.join(json.dumps(e) for e in events), encoding='utf-8')
    return path


def test_finite_session_without_stop_requested_has_explicit_anchor(tmp_path):
    report=audit.analyze(write_session(tmp_path,finite_events()))
    assert report['stop_anchor_source']=='stop_diagnostics.started_ns'
    assert report['rpc'][0]['duration_ms']==218
    assert report['rpc'][0]['sdk_await_ms']==218
    assert report['finite_motion_evidence']['max_observed_tcp_delta_mm']==0
    assert report['finite_motion_evidence']['command_max_joint_delta_rad']==.00013
    assert report['finite_motion_evidence']['within_reference_samples']==0
    assert report['finite_motion_evidence']['first_feedback_started_after_native_ms']==300
    assert report['finite_motion_evidence']['physical_motion_verified'] is False
    assert report['finite_motion_evidence']['stop_confirmed'] is False


def test_feedback_window_must_be_wholly_inside_reference(tmp_path):
    events=finite_events()
    for begin,end in ((110,120),(170,190),(90,110)):
        events.insert(-1,dict(kind='stop_observation_sample',
            read_started_ns=begin*1_000_000,read_finished_ns=end*1_000_000,
            state=dict(raw_robot_state='MOVING',actual_tcp={'p':[0.,0.,.40005]},
                       actual_q=[.00013,0.,0.,0.,0.,0.],actual_qd=[0.]*6)))
    evidence=audit.analyze(write_session(tmp_path,events))['finite_motion_evidence']
    assert evidence['within_reference_samples']==1
    assert evidence['max_observed_tcp_delta_mm']==pytest.approx(.05)
    assert evidence['physical_motion_verified'] is False  # Telemetry is not site acceptance.


@pytest.mark.parametrize('change', ['clock','missing_baseline','future_baseline','different_q',
                                  'multiple_pvat','missing_timestamp','short_vector','invalid_horizon',
                                  'different_requested','different_qd','different_qdd'])
def test_finite_audit_refuses_ambiguous_evidence(tmp_path,change):
    events=finite_events()
    if change=='clock': events[-1]['event_clock']='time.monotonic_ns'
    elif change=='missing_baseline':events.pop(1)
    elif change=='future_baseline':
        events[1]['snapshot_read_completed_ns']=110_000_000
        events[2]['point_inputs']['snapshot_read_completed_ns']=110_000_000
    elif change=='different_q':events[1]['actual_q'][0]=.01
    elif change=='missing_timestamp':events[2]['point_inputs'].pop('snapshot_read_completed_ns')
    elif change=='short_vector':events[2]['p']=[0.]
    elif change=='invalid_horizon':events[2]['horizon_s']=float('nan')
    elif change=='different_requested':events[1]['snapshot_read_requested_ns']=79_000_000
    elif change=='different_qd':events[1]['actual_qd'][0]=.01
    elif change=='different_qdd':events[1]['actual_qdd'][0]=.01
    else:events.insert(3,dict(events[2]))
    evidence=audit.analyze(write_session(tmp_path,events))['finite_motion_evidence']
    assert evidence['assessment']=='insufficient_or_ambiguous_evidence'
    assert evidence['physical_motion_verified'] is False


def test_finite_no_tail_still_reports_stop_failure(tmp_path):
    events=[e for e in finite_events() if e['kind']!='stop_observation_sample']
    report=audit.analyze(write_session(tmp_path,events))
    assert report['observation_count']==0
    assert report['final_dx_mm'] is None
    assert report['finite_motion_evidence']['observed_samples']==0


def test_missing_stop_anchor_is_clear_error(tmp_path):
    with pytest.raises(ValueError,match='stop anchor'):
        audit.analyze(write_session(tmp_path,[]))


def test_different_stop_episodes_are_not_merged(tmp_path):
    events=finite_events()
    events.append(dict(kind='stop_diagnostics',episode_id=2,started_ns=500_000_000))
    with pytest.raises(ValueError,match='Multiple stop episodes'):
        audit.analyze(write_session(tmp_path,events))


def test_unidentified_legacy_stop_requests_are_not_merged(tmp_path):
    events=[dict(kind='stop_requested',server_mono_ns=t) for t in (100,200)]
    with pytest.raises(ValueError,match='Multiple stop episodes'):
        audit.analyze(write_session(tmp_path,events))
