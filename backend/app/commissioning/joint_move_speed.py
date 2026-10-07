"""Offline and commissioning joint velocity evidence; no SDK/device access.

This checks observed telemetry after arrival. It is not a motion supervisor,
continuous physical speed guarantee, or proof of motion-time stopping.
"""
import argparse
import hashlib
import json
import math
from collections.abc import Mapping
from pathlib import Path


def _finite_number(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _velocity(value):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        return None
    if not all(_finite_number(v) for v in value):
        return None
    return tuple(abs(v) for v in value)


class JointMoveSpeedAudit:
    """Constant-memory reduction of joint_move snapshots only."""
    def __init__(self, limit_radps):
        if not _finite_number(limit_radps) or limit_radps <= 0:
            raise ValueError('invalid_joint_speed_limit')
        self.limit_radps = limit_radps
        self.valid_samples = 0
        self.invalid_samples = 0
        self.over_limit_samples = 0
        self.actual_peaks = [0.] * 6
        self.target_peaks = [0.] * 6
        self.first_started_ns = None
        self.last_completed_ns = None
        self.last_started_ns = None
        self.max_start_gap_ns = 0
        self.max_read_duration_ns = 0

    def observe(self, event):
        if not isinstance(event, Mapping) or event.get('kind') != 'robot_kinematics':
            return
        if event.get('snapshot_read_purpose') != 'joint_move':
            return
        actual = _velocity(event.get('actual_qd'))
        target = _velocity(event.get('target_qd'))
        started = event.get('snapshot_read_started_ns')
        completed = event.get('snapshot_read_completed_ns')
        if (actual is None or target is None or type(started) is not int or
                type(completed) is not int or started < 0 or completed < started or
                (self.last_started_ns is not None and started < self.last_started_ns) or
                (self.last_completed_ns is not None and completed < self.last_completed_ns)):
            self.invalid_samples += 1
            return
        self.valid_samples += 1
        # Only floating representation tolerance, not a physical speed allowance.
        if any(v > self.limit_radps and not math.isclose(v, self.limit_radps,
                                                       rel_tol=1e-12, abs_tol=1e-12)
               for v in actual):
            self.over_limit_samples += 1
        self.actual_peaks = [max(a, b) for a, b in zip(self.actual_peaks, actual)]
        self.target_peaks = [max(a, b) for a, b in zip(self.target_peaks, target)]
        if self.first_started_ns is None:
            self.first_started_ns = started
        if self.last_started_ns is not None:
            self.max_start_gap_ns = max(self.max_start_gap_ns, started-self.last_started_ns)
        self.last_started_ns = started
        self.last_completed_ns = completed
        self.max_read_duration_ns = max(self.max_read_duration_ns, completed-started)

    def report(self):
        status = ('exceeded' if self.over_limit_samples else
                  'incomplete' if not self.valid_samples or self.invalid_samples else
                  'within_observed_limit')
        peak = max(self.actual_peaks) if self.valid_samples else None
        return {
            'status': status,
            'requested_limit_radps': self.limit_radps,
            'valid_samples': self.valid_samples, 'invalid_samples': self.invalid_samples,
            'over_limit_samples': self.over_limit_samples,
            'actual_peak_radps': peak,
            'actual_peak_joint': self.actual_peaks.index(peak)+1 if peak is not None else None,
            'actual_peaks_radps': list(self.actual_peaks) if self.valid_samples else None,
            'target_peak_radps': max(self.target_peaks) if self.valid_samples else None,
            'target_peaks_radps': list(self.target_peaks) if self.valid_samples else None,
            'first_read_started_ns': self.first_started_ns,
            'last_read_completed_ns': self.last_completed_ns,
            'max_read_start_gap_ms': self.max_start_gap_ns/1e6,
            'max_snapshot_duration_ms': self.max_read_duration_ns/1e6,
            'clock_basis': 'client_sequential_read_boundaries_not_controller_sample_time',
            'scope': 'joint_move_snapshot_reported_velocity_after_arrival',
            'physical_speed_verified': False, 'motion_time_stop_verified': False,
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', type=Path, required=True)
    parser.add_argument('--limit-radps', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    audit = JointMoveSpeedAudit(args.limit_radps)
    raw = args.session.read_bytes()
    for line in raw.splitlines():
        try:
            event = json.loads(line)
        except (ValueError, UnicodeError):
            raise ValueError('invalid_session_jsonl') from None
        if not isinstance(event, dict):
            raise ValueError('invalid_session_jsonl')
        audit.observe(event)
    report = audit.report()
    report.update(session_sha256=hashlib.sha256(raw).hexdigest(),
                  session_bytes=len(raw), offline_only=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as output:
        output.write(json.dumps(report, allow_nan=False, indent=2)+'\n')
    print(json.dumps(report, allow_nan=False))
    return 0 if report['status'] == 'within_observed_limit' else 2
