import importlib.util
from pathlib import Path
import subprocess

import pytest

spec = importlib.util.spec_from_file_location(
    'docker_pose_follow_probe', Path(__file__).resolve().parents[3] / 'scripts/docker_pose_follow_probe.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def trace():
    lines = ['POSE_FOLLOW_BEGIN\t.3\t0\t.4']
    for seq in range(1, 71):
        x = .301 if seq < 30 else .3
        lines.append(f'POSE_FOLLOW_POINT\t{seq}\t{seq*80}\t.303\t{x}\t0\t.4\t.001\t45\t10')
    lines += ['POSE_FOLLOW_RELEASE\t5700\t70', 'POSE_FOLLOW_QUIET\t6000\t.0001',
              'POSE_FOLLOW_DONE\ttrue\tnil\t70']
    return '\n'.join(lines)


def test_complete_trace_requires_progress_freshness_and_natural_quiet_tail():
    result = m.analyze(trace())
    assert result['passed'] and result['points'] == 70
    assert result['max_send_gap_ms'] == 80
    assert result['x_peak_progress_m'] == pytest.approx(.001)
    assert result['release_to_quiet_ms'] == 300


@pytest.mark.parametrize('change', [
    lambda s: s.replace('POSE_FOLLOW_DONE\ttrue', 'POSE_FOLLOW_DONE\tfalse'),
    lambda s: s + '\nPOSE_FOLLOW_ABORT\ttrue\ttrue',
    lambda s: s.replace('POSE_FOLLOW_DONE\ttrue\tnil\t70', 'POSE_FOLLOW_DONE'),
    lambda s: s.replace('POINT\t70\t', 'POINT\t69\t'),
    lambda s: s.replace('\t0.301\t', '\t0.3\t'),
    lambda s: s.replace('\t45', '\t100'),
    lambda s: s.replace('QUIET\t6000\t.0001', 'QUIET\t6000\t.001'),
    lambda s: s.replace('QUIET\t6000\t.0001', 'QUIET\t6000\tnan'),
    lambda s: s.replace('\t45', '\tnan'),
    lambda s: s.replace('QUIET\t6000\t.0001', 'QUIET\t5701\t.0001'),
])
def test_failures_missing_output_no_progress_and_stale_result_do_not_pass(change):
    assert not m.analyze(change(trace()))['passed']


def test_program_embeds_real_mailbox_and_pvat_generator_without_host_target():
    source = m.program()
    assert '__POSE_MAILBOX_MODULE__' not in source and '__POSE_TRACKER_MODULE__' not in source
    assert 'kinematics_inverse(sample.pose,seed)' in source
    assert 'lebai:move_pvat(p,v,a,dt)' in source
    assert 'wait_all(source,receiver,planner,motion)' in source
    assert 'for tick=1,70 do' in source
    for forbidden in ('stop_sys', 'connect_real', 'set_claw'):
        assert forbidden not in source


def test_timeout_retains_unknown_without_motion_retry_or_cleanup(monkeypatch):
    calls = []

    def timeout(*args, **kwargs):
        calls.append(args)
        raise subprocess.TimeoutExpired('docker', 40, output=b'POSE_FOLLOW_BEGIN\t.3\t0\t.4\n')

    monkeypatch.setattr(m.subprocess, 'run', timeout)
    result = m.execute({'container_id': 'isolated'})
    assert len(calls) == 1 and not result['passed'] and not result['completed']
    assert result['error'] == 'transport_timeout_execution_unknown'
    assert result['stop_unverified']
    assert 'POSE_FOLLOW_BEGIN' in result['raw_output']


@pytest.mark.parametrize('abort,expected', [
    ('POSE_FOLLOW_ABORT\ttrue\ttrue', False),
    ('POSE_FOLLOW_ABORT\ttrue\tfalse', True),
    ('POSE_FOLLOW_ABORT\tfalse\ttrue', True),
    ('', True),
])
def test_failed_motion_only_reports_confirmed_stop_with_both_return_and_observation(abort, expected):
    raw = '\n'.join(['POSE_FOLLOW_MOTION_ENTER\tmovej', abort, 'POSE_FOLLOW_DONE\tfalse\terror\t0'])
    result = m.analyze(raw)
    assert not result['passed'] and result['stop_unverified'] == expected
