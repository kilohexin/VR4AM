import importlib.util
from pathlib import Path
import subprocess

import pytest

spec = importlib.util.spec_from_file_location(
    'docker_pose_transport_probe', Path(__file__).resolve().parents[3] / 'scripts/docker_pose_transport_probe.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def item():
    return dict(Id='isolated-id', Image='sha256:image', State={'Running': True}, Mounts=[],
                Config={'Image': m.IMAGE + ':latest', 'Cmd': ['/data/startup.sh']},
                HostConfig=dict(NetworkMode='none', Privileged=False, Devices=[], PortBindings={}),
                NetworkSettings={'Ports': {}})


def trace():
    return '\n'.join(['POSE_PROBE_CHECK\t' + name for name in sorted(m.CONTRACT_CHECKS)] +
                     [f'POSE_PROBE_POINT\t{i}\t5\t6\t1\t14' for i in range(1, 21)] +
                     ['POSE_PROBE_DONE\ttrue\tnil'])


def test_admits_only_non_networked_official_instance():
    assert m.validate_isolated(item())['container_id'] == 'isolated-id'


@pytest.mark.parametrize('path,value', [
    (('State', 'Running'), False), (('Config', 'Image'), 'other:latest'),
    (('Config', 'Cmd'), ['bash']), (('HostConfig', 'NetworkMode'), 'bridge'),
    (('HostConfig', 'Privileged'), True), (('HostConfig', 'Devices'), ['device']),
    (('HostConfig', 'PortBindings'), {'3030/tcp': []}), (('Mounts',), ['host-path']),
    (('NetworkSettings', 'Ports'), {'3030/tcp': None}),
])
def test_rejects_physical_routes_and_unverified_runtime(path, value):
    row = item()
    target = row
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ValueError, match='not_admitted'):
        m.validate_isolated(row)


def test_refuses_remote_docker_before_any_process(monkeypatch):
    monkeypatch.setenv('DOCKER_HOST', 'tcp://remote:1234')
    monkeypatch.setattr(m.subprocess, 'run', lambda *a, **k: pytest.fail('must not inspect remote Docker'))
    with pytest.raises(ValueError, match='not_admitted'):
        m.admission('anything')


def test_build_contains_decoder_and_independent_rotation_expectations_without_motion_calls():
    code = m.program()
    assert '__POSE_MAILBOX_MODULE__' not in code and '__ROTATION_CASES__' not in code
    assert 'rotation_conventions' in code and 'kinematics_inverse' in code
    for name in ('move_pvat(', 'move_pvt(', 'movej(', 'stop_move(', 'start_sys(', 'set_claw('):
        assert name not in code


def test_only_complete_trace_meeting_native_budget_passes():
    result = m.analyze(trace())
    assert result['passed'] and result['samples'] == 20 and result['max_total_ms'] == 14
    assert result['max_read_ms'] == 6 and result['motion_calls'] == 0


@pytest.mark.parametrize('change', [
    lambda s: s.replace('POSE_PROBE_DONE\ttrue\tnil', 'POSE_PROBE_DONE'),
    lambda s: s.replace('POSE_PROBE_DONE\ttrue', 'POSE_PROBE_DONE\tfalse'),
    lambda s: s.replace('POSE_PROBE_CHECK\trollback', 'POSE_PROBE_CHECK\tempty'),
    lambda s: s.replace('POSE_PROBE_POINT\t20\t', 'POSE_PROBE_POINT\t19\t'),
    lambda s: s.replace('\t14', '\t80'),
    lambda s: s.replace('\t14', '\tnan'),
    lambda s: s.replace('\t14', '\t-1'),
    lambda s: s.replace('\t5\t6\t1\t14', '\t5\t6\t1'),
    lambda s: s + '\nPOSE_PROBE_DONE\ttrue\tnil',
])
def test_missing_checks_stale_timing_and_partial_output_never_pass(change):
    assert not m.analyze(change(trace()))['passed']


def test_transport_timeout_preserves_evidence_and_never_retries(monkeypatch):
    calls = []

    def timeout(*args, **kwargs):
        calls.append(args)
        raise subprocess.TimeoutExpired('docker', 25, output=b'POSE_PROBE_POINT\t1\t1\t2\t3\t6\n')

    monkeypatch.setattr(m.subprocess, 'run', timeout)
    result = m.execute({'container_id': 'isolated-id'})
    assert not result['passed'] and not result['completed'] and len(calls) == 1
    assert result['error'] == 'transport_timeout_execution_unknown'
    assert 'POSE_PROBE_POINT' in result['raw_output']
