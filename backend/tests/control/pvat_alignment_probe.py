"""TEST ONLY: conditional reference comparison, never execution/arrival proof."""
from dataclasses import dataclass

import numpy as np

from app.robots.command_admission import CommandRef
from app.robots.motion_view import JointObservation
from tests.control.pvat_observation_probe import QualifiedObservation
from tests.robots.pvat_interpolation_probe import QuinticSegment


@dataclass(frozen=True)
class ReferenceWindow:
    segment: QuinticSegment
    origin_ns: int
    owner_generation: int
    command: CommandRef


@dataclass(frozen=True)
class SyntheticTiming:
    execution_origin_earliest_ns: int
    execution_origin_latest_ns: int
    sample_within_read: bool = False
    synchronous_pva: bool = False


@dataclass(frozen=True)
class ReferenceCompatibility:
    observation: QualifiedObservation
    possible_reference_time_s: tuple[float, float]
    witness_time_s: float
    scope: str = 'synthetic_time_compatibility'


def _reference(reference):
    if (not isinstance(reference, ReferenceWindow)
            or type(reference.origin_ns) is not int or reference.origin_ns < 0
            or type(reference.owner_generation) is not int or reference.owner_generation < 0
            or not isinstance(reference.command, CommandRef)
            or any(type(x) is not int or x < 0 for x in (
                reference.command.command_id, reference.command.generation, reference.command.submission_id))
            or not isinstance(reference.segment, QuinticSegment)):
        raise ValueError('invalid_alignment_reference')
    segment = reference.segment
    try:
        coefficients = np.asarray(segment.coefficients, dtype=float)
        if (coefficients.shape != (6, 6) or not np.all(np.isfinite(coefficients))
                or not np.isfinite(segment.duration_s) or segment.duration_s <= 0
                or round(segment.duration_s * 1e9) <= 0):
            raise ValueError
    except (ValueError, TypeError, OverflowError):
        raise ValueError('invalid_alignment_reference') from None
    return reference.origin_ns + round(segment.duration_s * 1e9)


def compare_reference(reference, observation, *, timing=None, tolerances=(1e-4, 1e-3, .02)):
    """Find one common PVA time under explicit synthetic sampling assumptions.

    Root partition is a floating-point feasibility probe, not certified interval
    arithmetic. A witness is conditional compatibility, never a measured time.
    """
    deadline = _reference(reference)
    if (not isinstance(observation, QualifiedObservation)
            or not isinstance(observation.sample, JointObservation)
            or type(observation.owner_generation) is not int
            or not isinstance(observation.command, CommandRef)
            or any(type(x) is not int for x in (observation.command.command_id,
                observation.command.generation, observation.command.submission_id, observation.sample.generation))
            or observation.owner_generation != reference.owner_generation
            or observation.command != reference.command
            or observation.sample.generation != reference.command.generation):
        raise ValueError('pvat_alignment_binding')
    sample = observation.sample
    times = (sample.captured_ns, sample.read_completed_ns, observation.checked_ns)
    try:
        actual = np.asarray((sample.actual_q, sample.actual_qd, sample.actual_qdd), dtype=float)
        if (any(type(x) is not int or x < 0 for x in times)
                or not times[0] <= times[1] <= times[2]
                or actual.shape != (3, 6) or not np.all(np.isfinite(actual))
                or observation.age_ns != times[2]-times[0]
                or observation.read_span_ns != times[1]-times[0]):
            raise ValueError
    except (ValueError, TypeError):
        raise ValueError('invalid_alignment_observation') from None
    if (not isinstance(timing, SyntheticTiming) or timing.sample_within_read is not True
            or timing.synchronous_pva is not True):
        raise ValueError('pvat_alignment_time_unknown')
    origins = (timing.execution_origin_earliest_ns, timing.execution_origin_latest_ns)
    if any(type(x) is not int or x < 0 for x in origins) or origins[0] > origins[1]:
        raise ValueError('invalid_alignment_timing')
    try:
        tolerance = np.asarray(tolerances, dtype=float)
        if tolerance.shape != (3,) or not np.all(np.isfinite(tolerance)) or np.any(tolerance <= 0):
            raise ValueError
    except (ValueError, TypeError):
        raise ValueError('invalid_alignment_tolerances') from None
    segment = reference.segment
    earliest = sample.captured_ns-origins[1]
    latest = sample.read_completed_ns-origins[0]
    if (observation.checked_ns < reference.origin_ns or observation.checked_ns >= deadline
            or earliest < 0 or latest/1e9 > segment.duration_s):
        raise ValueError('pvat_alignment_coverage')
    lo, hi = earliest/1e9/segment.duration_s, latest/1e9/segment.duration_s
    curves = []
    boundaries = [lo, hi]
    for derivative in range(3):
        for joint, row in enumerate(segment.coefficients):
            error = np.polynomial.polynomial.polyder(row, m=derivative)/segment.duration_s**derivative
            error[0] -= actual[derivative, joint]
            curves.append((error, tolerance[derivative]))
            for sign in (-1., 1.):
                boundary = error.copy(); boundary[0] += sign*tolerance[derivative]
                for root in np.polynomial.polynomial.polyroots(boundary):
                    if abs(root.imag) <= 1e-10 and lo-1e-12 <= root.real <= hi+1e-12:
                        boundaries.append(float(np.clip(root.real, lo, hi)))
    boundaries = sorted(set(boundaries))
    candidates = sorted(boundaries + [(a+b)/2 for a, b in zip(boundaries, boundaries[1:])])
    for fraction in candidates:
        if all(abs(np.polynomial.polynomial.polyval(fraction, error)) <= bound+1e-12
               for error, bound in curves):
            return ReferenceCompatibility(observation, (earliest/1e9, latest/1e9),
                                          fraction*segment.duration_s)
    raise ValueError('pvat_alignment_incompatible')


class TerminalFeedbackProbe:
    def __init__(self, reference, target, *, settle_samples=2, min_span_ns=20_000_000,
                 max_gap_ns=30_000_000):
        self.deadline_ns = _reference(reference)
        target = np.asarray(target, dtype=float)
        if (target.shape != (6,) or not np.all(np.isfinite(target))
                or type(settle_samples) is not int or settle_samples < 2
                or type(min_span_ns) is not int or min_span_ns <= 0
                or type(max_gap_ns) is not int or max_gap_ns <= 0
                or not np.allclose(reference.segment.state(reference.segment.duration_s),
                                   (target, (0.,)*6, (0.,)*6), rtol=0, atol=1e-12)):
            raise ValueError('invalid_terminal_feedback')
        self.reference = reference
        self.target = tuple(float(x) for x in target)
        self.settle_samples, self.min_span_ns, self.max_gap_ns = settle_samples, min_span_ns, max_gap_ns
        self.phase, self.fault = 'awaiting_feedback', None
        self.good_samples = 0
        self.first_capture_ns = self.last_capture_ns = self.last_completed_ns = None
        self.last_checked_ns = reference.origin_ns

    def _reject(self, reason):
        if self.fault is None:
            self.fault = reason
        self.phase = 'fault'
        self.good_samples = 0
        return self.fault

    def check(self, now_ns):
        if self.fault is not None:
            return self.fault
        if self.phase == 'revoked':
            return None
        if type(now_ns) is not int or now_ns < self.last_checked_ns:
            return self._reject('pvat_reference_clock_reversed')
        self.last_checked_ns = now_ns
        if now_ns >= self.deadline_ns:
            return self._reject('pvat_window_expired')
        return None

    def observe(self, observation, *, timing=None):
        if self.phase == 'revoked':
            return self.phase
        if not isinstance(observation, QualifiedObservation):
            return self._reject('invalid_alignment_observation')
        fault = self.check(observation.checked_ns)
        if fault is not None:
            return fault
        try:
            compare_reference(self.reference, observation, timing=timing)
        except ValueError as error:
            return self._reject(str(error))
        sample = observation.sample
        if (self.last_capture_ns is not None and (sample.captured_ns <= self.last_capture_ns
                                                 or sample.captured_ns < self.last_completed_ns)):
            return self.phase
        close = (max(abs(a-b) for a, b in zip(sample.actual_q, self.target)) <= 1e-4
                 and max(abs(x) for x in sample.actual_qd) <= 1e-3
                 and max(abs(x) for x in sample.actual_qdd) <= .02)
        contiguous = (self.last_capture_ns is not None
                      and sample.captured_ns-self.last_capture_ns <= self.max_gap_ns)
        if not close or not contiguous:
            self.good_samples = 0
            self.first_capture_ns = None
        self.last_capture_ns, self.last_completed_ns = sample.captured_ns, sample.read_completed_ns
        if close:
            if self.first_capture_ns is None:
                self.first_capture_ns = sample.captured_ns
            self.good_samples += 1
        self.phase = ('feedback_consistent' if self.good_samples >= self.settle_samples
                      and sample.captured_ns-self.first_capture_ns >= self.min_span_ns else 'awaiting_feedback')
        return self.phase

    def retarget(self, target, now_ns):
        if self.fault is not None:
            return self.fault
        if self.phase == 'revoked':
            return self.phase
        fault = self.check(now_ns)
        if fault is not None:
            return fault
        values = np.asarray(target, dtype=float)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            return self._reject('invalid_terminal_target')
        if tuple(values) != self.target:
            return self._reject('pvat_terminal_target_changed')
        return self.phase

    def revoke(self):
        self.good_samples = 0
        self.first_capture_ns = self.last_capture_ns = self.last_completed_ns = None
        if self.fault is None:
            self.phase = 'revoked'
