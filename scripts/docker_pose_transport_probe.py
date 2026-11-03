"""No-motion full-pose/Lua/IK probe in an isolated official Docker container.

Default preview; --run never creates/restarts containers or connects an SDK.
Only a running network=none container without ports, mounts or devices is
admitted. Signal writes occur inside that container; motion APIs are absent.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
IMAGE = 'registry.cn-shanghai.aliyuncs.com/lebai/l-master'
CONTRACT_CHECKS = {
    'empty', 'latest', 'torn_each_field', 'other_bank_does_not_starve',
    'expiry_permanent', 'rollback', 'immutable_duplicate', 'revocation_permanent',
    'clock_wrap', 'quaternion_rejected', 'rotation_conventions',
}


def validate_isolated(item):
    config, host = item['Config'], item['HostConfig']
    if (item['State']['Running'] is not True or not config['Image'].startswith(IMAGE + ':')
            or config['Cmd'] != ['/data/startup.sh'] or host['NetworkMode'] != 'none'
            or host['Privileged'] or host.get('Devices') or item.get('Mounts')
            or host.get('PortBindings') or item['NetworkSettings']['Ports']):
        raise ValueError('isolated_docker_not_admitted')
    return dict(container_id=item['Id'], image_id=item['Image'], network='none', ports=[])


def admission(container):
    if os.environ.get('DOCKER_HOST'):
        raise ValueError('isolated_docker_not_admitted')
    endpoint = json.loads(subprocess.run(
        ['docker', 'context', 'inspect', '--format', '{{json .Endpoints.docker.Host}}'],
        capture_output=True, text=True, check=True, timeout=5).stdout)
    if not isinstance(endpoint, str) or not endpoint.startswith(('npipe:////./pipe/', 'unix:///')):
        raise ValueError('isolated_docker_not_admitted')
    rows = json.loads(subprocess.run(['docker', 'inspect', container], capture_output=True,
                                    text=True, check=True, timeout=5).stdout)
    return validate_isolated(rows[0])


def program():
    from scipy.spatial.transform import Rotation
    from app.control.vr_pose_mailbox import pack
    from app.schemas.messages import Pose
    module = (ROOT / 'scripts/vr_pose_mailbox.lua').read_text(encoding='utf-8')
    harness = (ROOT / 'scripts/docker_pose_transport_probe.lua').read_text(encoding='utf-8')
    cases = []
    for angles in ((0, 0, 0), (0, 70, 0), (30, 90, 40), (10, -90, -30),
                   (180, 0, 0), (0, 0, 180), (20, 30, -40), (1, 89.9, 2)):
        rotation = Rotation.from_euler('xyz', angles, degrees=True)
        pose = Pose(p=(.1234567, -.2345678, .3456789), q=tuple(rotation.as_quat()))
        values = ','.join(str(v) for v in pack(pose))
        matrix = ','.join(repr(float(v)) for v in rotation.as_matrix().ravel())
        cases.append('{values={' + values + '},rotation={' + matrix + '}}')
    return harness.replace('__POSE_MAILBOX_MODULE__', module).replace(
        '__ROTATION_CASES__', '{' + ','.join(cases) + '}')


def analyze(raw):
    rows = [line.strip().split('\t') for line in raw.splitlines()]
    done = [r for r in rows if r[0] == 'POSE_PROBE_DONE']
    points = [r for r in rows if r[0] == 'POSE_PROBE_POINT']
    checks = [r[1] for r in rows if r[0] == 'POSE_PROBE_CHECK']
    report = dict(completed=bool(done), controller_passed=bool(len(done) == 1 and len(done[0]) >= 3
                  and done[0][1] == 'true'), passed=False,
                  samples=len(points), contract_checks=checks, raw_output=raw, motion_calls=0)
    try:
        if any(len(row) != 6 for row in points):
            return report
        times = [tuple(float(v) for v in row[2:6]) for row in points]
        if any(not math.isfinite(v) or v < 0 for row in times for v in row):
            return report
        for i, name in enumerate(('write', 'read', 'ik', 'total')):
            report['max_' + name + '_ms'] = max((row[i] for row in times), default=None)
        report['passed'] = bool(report['controller_passed'] and len(points) == 20
                                and [int(r[1]) for r in points] == list(range(1, 21))
                                and set(checks) == CONTRACT_CHECKS
                                and len(checks) == len(CONTRACT_CHECKS)
                                and report['max_total_ms'] < 80)
    except (ValueError, IndexError):
        pass
    return report


def execute(admitted):
    bridge = ('exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; '
              'while IFS= read -r -t 20 line <&3; do printf "%s\\n" "$line"; '
              'case "$line" in *POSE_PROBE_DONE*) break;; esac; done')
    try:
        response = subprocess.run(['docker', 'exec', '-i', admitted['container_id'], 'bash', '-c', bridge],
                                  input=program(), text=True, encoding='utf-8', capture_output=True, timeout=25)
        report = analyze(response.stdout)
        report.update(transport_returncode=response.returncode, transport_stderr=response.stderr)
        report['passed'] = report['passed'] and response.returncode == 0
    except subprocess.TimeoutExpired as error:
        raw = error.stdout or b''
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', errors='replace')
        report = dict(passed=False, completed=False, raw_output=raw,
                      error='transport_timeout_execution_unknown', motion_calls=0)
    return dict(report, admission=admitted)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    admitted = admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False, admission=admitted, motion_calls=0,
                              signal_range=[240, 255], samples=20, purpose='full_pose_read_ik_timing')))
        return 0
    report = execute(admitted)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in report.items() if k not in ('raw_output', 'admission')}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
