"""Speed acceptance must reject excess/missing evidence despite stable arrival."""
import json
import math

import pytest

from app.commissioning.joint_move_speed import JointMoveSpeedAudit, main


def sample(speed, *, purpose='joint_move', started=100, completed=200):
    return dict(kind='robot_kinematics', snapshot_read_purpose=purpose,
                snapshot_read_started_ns=started, snapshot_read_completed_ns=completed,
                actual_qd=[0, 0, 0, 0, speed, 0], target_qd=[0, 0, 0, 0, .05, 0])


@pytest.mark.parametrize(('speed', 'status'), [
    (.049, 'within_observed_limit'), (.05, 'within_observed_limit'),
    (.050000000000006, 'within_observed_limit'),
    (.050001, 'exceeded'), (.10546117916713782, 'exceeded'), (-.105, 'exceeded'),
])
def test_audit_uses_actual_absolute_velocity_instead_of_planned_velocity(speed, status):
    audit = JointMoveSpeedAudit(.05)
    audit.observe(sample(speed))
    report = audit.report()
    assert report['status'] == status
    assert report['actual_peak_radps'] == pytest.approx(abs(speed))
    assert report['target_peak_radps'] == .05
    assert report['actual_peak_joint'] == 5
    assert report['physical_speed_verified'] is False
    assert report['motion_time_stop_verified'] is False


def test_one_overspeed_is_not_erased_by_later_standstill():
    audit = JointMoveSpeedAudit(.05)
    audit.observe(sample(.105))
    audit.observe(sample(0, started=300, completed=400))
    assert audit.report()['status'] == 'exceeded'
    assert audit.report()['over_limit_samples'] == 1
    assert audit.report()['valid_samples'] == 2


def test_other_read_purposes_do_not_satisfy_move_observation():
    audit = JointMoveSpeedAudit(.05)
    audit.observe(sample(0, purpose='preflight'))
    assert audit.report()['status'] == 'incomplete'
    assert audit.report()['valid_samples'] == 0


@pytest.mark.parametrize('field,value', [
    ('actual_qd', None), ('actual_qd', [0]*5), ('actual_qd', [math.nan]*6),
    ('actual_qd', [True]*6), ('actual_qd', ['0']*6), ('target_qd', [math.inf]*6),
    ('snapshot_read_started_ns', None), ('snapshot_read_started_ns', -1),
    ('snapshot_read_started_ns', True), ('snapshot_read_completed_ns', 99),
])
def test_invalid_sample_prevents_within_limit_claim(field, value):
    audit = JointMoveSpeedAudit(.05)
    bad = sample(0)
    bad[field] = value
    audit.observe(bad)
    audit.observe(sample(.04, started=300, completed=400))
    assert audit.report()['status'] == 'incomplete'
    assert audit.report()['invalid_samples'] == 1


def test_clock_regression_is_incomplete_not_a_new_motion_window():
    audit = JointMoveSpeedAudit(.05)
    audit.observe(sample(.04, started=300, completed=400))
    audit.observe(sample(.04))
    assert audit.report()['status'] == 'incomplete'


def test_impossibly_large_velocity_is_invalid_evidence_instead_of_crashing():
    audit = JointMoveSpeedAudit(.05)
    audit.observe(sample(10**1000))
    assert audit.report()['status'] == 'incomplete'
    assert audit.report()['invalid_samples'] == 1


def test_completion_clock_regression_prevents_within_limit_claim():
    audit = JointMoveSpeedAudit(.05)
    audit.observe(sample(0, started=100, completed=300))
    audit.observe(sample(0, started=200, completed=250))
    assert audit.report()['status'] == 'incomplete'


def test_cli_with_no_motion_snapshots_returns_incomplete(tmp_path):
    source = tmp_path/'session.jsonl'
    source.write_text(json.dumps({'kind': 'session_started'})+'\n')
    output = tmp_path/'report.json'
    assert main(['--session', str(source), '--limit-radps', '.05', '--output', str(output)]) == 2
    assert json.loads(output.read_text())['status'] == 'incomplete'


def test_overspeed_in_any_joint_is_checked():
    audit = JointMoveSpeedAudit(.05)
    event = sample(0)
    event['actual_qd'][1] = -.1
    audit.observe(event)
    assert audit.report()['actual_peak_joint'] == 2
    assert audit.report()['status'] == 'exceeded'


@pytest.mark.parametrize('limit', [0, -.05, math.nan, math.inf, True])
def test_invalid_limit_is_rejected(limit):
    with pytest.raises(ValueError, match='invalid_joint_speed_limit'):
        JointMoveSpeedAudit(limit)


def test_cli_replays_saved_log_and_refuses_to_overwrite(tmp_path):
    source = tmp_path/'session.jsonl'
    source.write_text(json.dumps(sample(.105))+'\n'+json.dumps(sample(0, started=300, completed=400))+'\n')
    before = source.read_bytes()
    output = tmp_path/'audit.json'
    argv = ['--session', str(source), '--limit-radps', '.05', '--output', str(output)]
    assert main(argv) == 2
    report = json.loads(output.read_text())
    assert report['status'] == 'exceeded'
    assert report['valid_samples'] == 2
    assert len(report['session_sha256']) == 64
    assert source.read_bytes() == before
    with pytest.raises(FileExistsError):
        main(argv)
    assert json.loads(output.read_text()) == report


def test_cli_rejects_truncated_json_without_editing_source(tmp_path):
    source = tmp_path/'session.jsonl'
    source.write_text('{"kind":')
    before = source.read_bytes()
    with pytest.raises(ValueError, match='invalid_session_jsonl'):
        main(['--session', str(source), '--limit-radps', '.05', '--output', str(tmp_path/'out.json')])
    assert source.read_bytes() == before
