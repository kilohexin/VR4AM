"""Bounded full-pose IK/PVAT experiment, isolated network=none Docker only.

Default preview. One prepare movej then 70 PVAT intervals at 80ms, generated
hand targets (not a headset). A failed/unknown run is retained, never retried.
No physical selector, SDK connection, browser registration or gripper control.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from docker_pose_transport_probe import admission


def program():
    code = (ROOT / 'scripts/docker_pose_follow_probe.lua').read_text(encoding='utf-8')
    for marker, file in (('__POSE_MAILBOX_MODULE__', 'vr_pose_mailbox.lua'),
                         ('__POSE_TRACKER_MODULE__', 'vr_pose_tracker.lua')):
        code = code.replace(marker, (ROOT / 'scripts' / file).read_text(encoding='utf-8'))
    return code


def analyze(raw):
    rows = [line.strip().split('\t') for line in raw.splitlines()]
    done = [r for r in rows if r[0] == 'POSE_FOLLOW_DONE']
    points = [r for r in rows if r[0] == 'POSE_FOLLOW_POINT']
    begin = [r for r in rows if r[0] == 'POSE_FOLLOW_BEGIN']
    release = [r for r in rows if r[0] == 'POSE_FOLLOW_RELEASE']
    quiet = [r for r in rows if r[0] == 'POSE_FOLLOW_QUIET']
    aborts = [r for r in rows if r[0] == 'POSE_FOLLOW_ABORT']
    entered = any(r[0] == 'POSE_FOLLOW_MOTION_ENTER' for r in rows) or bool(begin or points)
    success = len(done) == 1 and len(done[0]) == 4 and done[0][1] == 'true'
    confirmed_abort = len(aborts) == 1 and aborts[0][1:] == ['true', 'true']
    result = dict(passed=False, completed=bool(done), points=len(points), raw_output=raw,
                  aborted=bool(aborts), stop_unverified=bool(
                      (aborts and not confirmed_abort) or (entered and not success and not confirmed_abort)))
    try:
        if (len(done) != 1 or len(done[0]) != 4 or len(begin) != 1 or len(release) != 1 or len(quiet) != 1
                or any(len(r) != 10 for r in points) or len(points) != 70):
            return result
        numeric = [tuple(float(x) for x in r[1:]) for r in points]
        if any(not math.isfinite(v) for row in numeric for v in row):
            return result
        result['max_send_gap_ms'] = max(b[1] - a[1] for a, b in zip(numeric, numeric[1:]))
        result['strict_interval_cadence'] = all(0 < b[1] - a[1] < 80 for a, b in zip(numeric, numeric[1:]))
        result['max_input_age_at_write_ms'] = max(r[7] for r in numeric)
        result['max_pvat_rpc_ms'] = max(r[8] for r in numeric)
        result['x_peak_progress_m'] = max(r[3] - float(begin[0][1]) for r in numeric)
        result['final_tcp_error_m'] = float(quiet[0][2])
        result['release_to_quiet_ms'] = float(quiet[0][1]) - float(release[0][1])
        result['passed'] = bool(done[0][1] == 'true' and int(done[0][3]) == 70
                                and [int(r[0]) for r in numeric] == list(range(1, 71))
                                and not result['aborted'] and not result['stop_unverified']
                                and all(0 < b[1] - a[1] < 160 for a, b in zip(numeric, numeric[1:]))
                                and 0 <= result['max_input_age_at_write_ms'] < 100
                                and result['x_peak_progress_m'] > .0005
                                and 0 <= result['final_tcp_error_m'] < .0007
                                and 200 <= result['release_to_quiet_ms'] < 3000)
    except (ValueError, IndexError):
        pass
    return result


def execute(admitted):
    bridge = ('exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; '
              'while IFS= read -r -t 30 line <&3; do printf "%s\\n" "$line"; '
              'case "$line" in *POSE_FOLLOW_DONE*) break;; esac; done')
    try:
        response = subprocess.run(['docker', 'exec', '-i', admitted['container_id'], 'bash', '-c', bridge],
                                  input=program(), text=True, encoding='utf-8', capture_output=True, timeout=40)
        result = analyze(response.stdout)
        result.update(transport_returncode=response.returncode, transport_stderr=response.stderr)
        result['passed'] = result['passed'] and response.returncode == 0
    except subprocess.TimeoutExpired as error:
        raw = error.stdout or b''
        if isinstance(raw, bytes):
            raw = raw.decode('utf-8', errors='replace')
        result = dict(passed=False, completed=False, raw_output=raw,
                      error='transport_timeout_execution_unknown', stop_unverified=True)
    return dict(result, admission=admitted)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container', required=True)
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    admitted = admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False, admission=admitted, prepare_movej=1,
                              pvat_points=70, period_ms=80, tcp_target_mm=3,
                              reference_velocity_cap_radps=.06, input_expiry_ms=100)))
        return 0
    result = execute(admitted)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('raw_output', 'admission')}))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
