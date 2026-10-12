import json

import pytest

from scripts import analyze_stop_conditions as audit


def session(tmp_path, events, **metadata):
    path = tmp_path / 'session.jsonl'
    header = {'kind': 'session_started', 'metadata': {
        'backend': 'LEBAI', 'real_robot_mode': 'control', **metadata}}
    path.write_text('\n'.join(json.dumps(row) for row in [header, *events]), encoding='utf-8')
    return path


def rpc(**changes):
    return {'kind': 'stop_rpc_lifecycle', 'episode_id': 1, 'request_id': 1,
            'method': 'stop_move', 'started_ns': 200_000_000,
            'sdk_await_started_ns': 200_000_000, 'sdk_await_completed_ns': 810_000_000,
            'completed_ns': 810_000_000, 'outcome': 'returned_late', **changes}


def diagnostic(**changes):
    return {'kind': 'stop_diagnostics', 'episode_id': 1, 'started_ns': 200_000_000,
            'reason': 'grip_released', 'stop_entry': {'pvat_pending': False}, **changes}


def telemetry(**changes):
    return {'kind': 'robot_kinematics', 'server_mono_ns': 170_000_000,
            'snapshot_read_completed_ns': 180_000_000, 'raw_robot_state': 'MOVING',
            'actual_qd': [0.01, 0, 0, 0, 0, 0], **changes}


def test_reused_stop_diagnostics_are_not_counted_as_independent_rpcs(tmp_path):
    path = session(tmp_path, [telemetry(), diagnostic(), rpc(),
                             diagnostic(started_ns=900_000_000, reason='shutdown')])
    result = audit.analyze_session(path)
    assert result['rpc_count'] == 1 and result['diagnostic_count'] == 2
    row = result['rpcs'][0]
    assert row['diagnostic_count_for_episode'] == 2
    assert row['reason'] == 'grip_released' and row['sdk_await_ms'] == 610
    assert row['pre_state']['basis'] == 'completed_read'
    assert row['pre_state']['raw_robot_state'] == 'MOVING'
    assert not result['hardware_stop_verified']


def test_episode_ids_are_scoped_and_exact_duplicate_lifecycles_are_deduplicated(tmp_path):
    path = session(tmp_path, [diagnostic(), rpc(), rpc(),
        diagnostic(episode_id=2, started_ns=900_000_000),
        rpc(episode_id=2, started_ns=900_000_000,
            sdk_await_started_ns=900_000_000, sdk_await_completed_ns=910_000_000)])
    result = audit.analyze_session(path)
    assert result['rpc_count'] == 2
    assert result['duplicate_lifecycle_count'] == 1


def test_conflicting_lifecycle_is_rejected(tmp_path):
    with pytest.raises(ValueError, match='conflicting'):
        audit.analyze_session(session(tmp_path, [rpc(), rpc(outcome='returned')]))


@pytest.mark.parametrize('metadata', [
    {'backend': 'LEBAI_FAKE'}, {'backend': 'SIMULATOR'},
    {'runtime': 'SIMULATION'}, {'hardware_verified': False},
    {'real_robot_mode': 'readonly'},
])
def test_fake_and_noncontrol_sessions_cannot_enter_field_comparison(tmp_path, metadata):
    with pytest.raises(ValueError, match='field'):
        audit.analyze_session(session(tmp_path, [rpc()], **metadata))


def test_future_and_inflight_reads_cannot_describe_state_before_stop(tmp_path):
    path = session(tmp_path, [telemetry(),
        telemetry(server_mono_ns=190_000_000, snapshot_read_completed_ns=220_000_000, raw_robot_state='IDLE'),
        telemetry(server_mono_ns=210_000_000, snapshot_read_completed_ns=215_000_000, raw_robot_state='IDLE'),
        diagnostic(), rpc()])
    row = audit.analyze_session(path)['rpcs'][0]
    assert row['pre_state']['raw_robot_state'] == 'MOVING'
    assert row['pre_state']['read_age_ms'] == 20


def test_legacy_read_start_is_reported_without_claiming_read_completed(tmp_path):
    old = telemetry()
    old.pop('snapshot_read_completed_ns')
    row = audit.analyze_session(session(tmp_path, [old, diagnostic(), rpc()]))['rpcs'][0]
    assert row['pre_state']['basis'] == 'legacy_read_start_only'
    assert row['pre_state']['read_age_ms'] is None


def test_equal_legacy_time_and_missing_telemetry_are_unknown(tmp_path):
    old = telemetry(server_mono_ns=200_000_000)
    old.pop('snapshot_read_completed_ns')
    row = audit.analyze_session(session(tmp_path, [old, diagnostic(), rpc()]))['rpcs'][0]
    assert row['pre_state'] is None


def test_state_is_selected_before_stop_entry_not_after_lock_wait(tmp_path):
    row = audit.analyze_session(session(tmp_path, [telemetry(), diagnostic(),
        telemetry(server_mono_ns=205_000_000, snapshot_read_completed_ns=210_000_000, raw_robot_state='IDLE'),
        rpc(started_ns=220_000_000, sdk_await_started_ns=220_000_000)]))['rpcs'][0]
    assert row['pre_state']['raw_robot_state'] == 'MOVING'
    assert row['pre_state']['reference_ns'] == 200_000_000


def test_performance_clock_interval_is_kept_separate_from_monotonic(tmp_path):
    timing = {'clock': 'time.perf_counter_ns', 'sdk_await_started_ns': 9_000_000_000,
              'sdk_await_completed_ns': 9_613_000_000}
    row = audit.analyze_session(session(tmp_path, [rpc(diagnostic_timing=timing)]))['rpcs'][0]
    assert row['sdk_await_ms'] == 610 and row['sdk_await_qpc_ms'] == 613


@pytest.mark.parametrize('timing', [
    {'clock': 'other', 'sdk_await_started_ns': 0, 'sdk_await_completed_ns': 100},
    {'clock': 'time.perf_counter_ns', 'sdk_await_started_ns': True, 'sdk_await_completed_ns': 100},
    {'clock': 'time.perf_counter_ns', 'sdk_await_started_ns': 200, 'sdk_await_completed_ns': 100},
])
def test_bad_or_unknown_clock_intervals_are_not_zero_filled(tmp_path, timing):
    row = audit.analyze_session(session(tmp_path, [rpc(diagnostic_timing=timing)]))['rpcs'][0]
    assert row['sdk_await_qpc_ms'] is None


def test_missing_timing_is_unknown_even_for_returned_outcome(tmp_path):
    row = rpc(outcome='returned')
    row.pop('sdk_await_started_ns')
    assert audit.analyze_session(session(tmp_path, [row]))['rpcs'][0]['sdk_await_ms'] is None


def test_invalid_json_is_rejected_without_echoing_original_content(tmp_path):
    path = tmp_path / 'session.jsonl'
    path.write_text('secret-invalid-json', encoding='utf-8')
    with pytest.raises(ValueError, match='invalid JSONL') as error:
        audit.analyze_session(path)
    assert 'secret-invalid-json' not in str(error.value)


def test_equal_completed_read_time_cannot_prove_order_with_coarse_clock(tmp_path):
    old = telemetry()
    tied = telemetry(server_mono_ns=190_000_000, snapshot_read_completed_ns=200_000_000,
                     raw_robot_state='IDLE')
    row = audit.analyze_session(session(tmp_path, [old, tied, diagnostic(), rpc()]))['rpcs'][0]
    assert row['pre_state']['raw_robot_state'] == 'MOVING'
    assert row['pre_state']['read_age_ms'] == 20


def test_identical_file_copies_do_not_duplicate_trials(tmp_path):
    first = session(tmp_path, [rpc()])
    copy = tmp_path / 'copy.jsonl'
    copy.write_bytes(first.read_bytes())
    result = audit.analyze_sessions([first, copy])
    assert result['session_count'] == 1 and result['rpc_count'] == 1
    assert result['duplicate_source_count'] == 1


def test_separate_session_ids_do_not_merge_same_episode_and_request_id(tmp_path):
    first = session(tmp_path, [rpc()])
    second = tmp_path / 'second.jsonl'
    data = first.read_text().replace('810000000', '811000000')
    second.write_text(data)
    assert audit.analyze_sessions([first, second])['rpc_count'] == 2


def test_existing_output_is_preserved_before_any_input_read(tmp_path, monkeypatch):
    output = tmp_path / 'result.json'
    output.write_bytes(b'previous')
    monkeypatch.setattr(audit, 'analyze_sessions', lambda *_: pytest.fail('no input read allowed'))
    with pytest.raises(SystemExit) as error:
        audit.main(['missing.jsonl', '--output', str(output)])
    assert error.value.code == 2 and output.read_bytes() == b'previous'


@pytest.mark.parametrize('velocity', [[0]*5, [0]*5+[True], [0]*5+[float('nan')], [0]*5+[float('inf')]])
def test_invalid_velocity_is_unknown_without_implying_zero(tmp_path, velocity):
    row = audit.analyze_session(session(tmp_path, [telemetry(actual_qd=velocity), diagnostic(), rpc()]))['rpcs'][0]
    assert row['pre_state']['max_abs_qd'] is None


@pytest.mark.parametrize('reverse', [False, True])
def test_conflicting_latest_read_ties_are_ambiguous_regardless_of_file_order(tmp_path, reverse):
    events = [telemetry(), telemetry(raw_robot_state='IDLE', actual_qd=[0]*6)]
    if reverse:
        events.reverse()
    row = audit.analyze_session(session(tmp_path, [*events, diagnostic(), rpc()]))['rpcs'][0]
    assert row['pre_state']['basis'] == 'ambiguous_same_timestamp'
    assert row['pre_state']['raw_robot_state'] is None
    assert row['pre_state']['max_abs_qd'] is None


def test_identical_latest_read_ties_do_not_create_a_false_conflict(tmp_path):
    row = audit.analyze_session(session(tmp_path, [telemetry(), telemetry(), diagnostic(), rpc()]))['rpcs'][0]
    assert row['pre_state']['basis'] == 'completed_read'
    assert row['pre_state']['raw_robot_state'] == 'MOVING'
