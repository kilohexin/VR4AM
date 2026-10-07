"""TEST ONLY: phase-local endpoint encoding under declared quintic semantics.

There is no dispatcher, controller timing model, SDK call or feedback here.
Packet start times describe a mathematical partition, not receipt/execution.
Retargeting uses covered reference predictions, never measured robot state.
"""
from dataclasses import dataclass
from math import ceil, comb

import numpy as np

from tests.robots.pvat_interpolation_probe import QuinticSegment, quintic_segment
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import (
    SettlingLimits, _position_range, build_settling_plan,
)


@dataclass(frozen=True)
class EncodedWindow:
    start_s: float
    segment: QuinticSegment

    @property
    def duration_s(self):
        return self.segment.duration_s

    def state(self, time_s):
        return self.segment.state(time_s)

    def sdk_arguments(self):
        """Inert endpoint fields only; initial PVA remains explicit in segment."""
        q, v, a = self.state(self.duration_s)
        return {'p': list(q), 'v': list(v), 'a': list(a), 't': self.duration_s}


def _motion_limits(limits):
    try:
        minimum = np.asarray(limits.joint_min, dtype=float)
        maximum = np.asarray(limits.joint_max, dtype=float)
        values = (limits.duration_s, limits.speed_cap, limits.acceleration_cap)
        valid = (minimum.shape == maximum.shape == (6,)
                 and np.all(np.isfinite((minimum, maximum)))
                 and np.all(minimum < maximum)
                 and np.all(np.isfinite(values)) and min(values) > 0)
    except (TypeError, ValueError):
        valid = False
    if not valid:
        raise ValueError('invalid_motion_limits')
    return minimum, maximum


def _check_curve(segment, limits, minimum, maximum):
    speed, acceleration = segment.peak_abs_derivatives()
    lower, upper = _position_range(segment)
    if (max(speed) > limits.speed_cap+1e-10
            or max(acceleration) > limits.acceleration_cap+1e-10
            or np.any(lower < minimum-1e-12)
            or np.any(upper > maximum+1e-12)):
        raise ValueError('pvat_window_interpolation_limit')


def _restriction(segment, start, duration):
    offset, scale = start/segment.duration_s, duration/segment.duration_s
    rows = tuple(tuple(sum(row[j]*comb(j,k)*offset**(j-k)*scale**k
                           for j in range(k,6)) for k in range(6))
                 for row in segment.coefficients)
    return QuinticSegment(rows, duration)


def encode_phases(segments, limits=None, *, min_duration_s=.02,
                  max_duration_s=.08, max_points=2048):
    """All-or-reject bounded encoding; balanced windows never cross a phase."""
    limits = limits or RampLimits()
    minimum, maximum = _motion_limits(limits)
    if (not np.all(np.isfinite((min_duration_s, max_duration_s)))
            or not 0 < min_duration_s <= max_duration_s
            or type(max_points) is not int or max_points <= 0):
        raise ValueError('invalid_encoding_limits')
    segments = tuple(segments)
    if not segments or len(segments) > max_points:
        raise ValueError('invalid_phases' if not segments else 'pvat_encoding_budget')
    counts = []
    for segment in segments:
        rows = np.asarray(segment.coefficients, dtype=float)
        if (rows.shape != (6,6) or not np.all(np.isfinite(rows))
                or not np.isfinite(segment.duration_s) or segment.duration_s <= 0):
            raise ValueError('invalid_phases')
        count = ceil(segment.duration_s/max_duration_s)
        if sum(counts)+count > max_points:
            raise ValueError('pvat_encoding_budget')
        if segment.duration_s/count < min_duration_s:
            raise ValueError('pvat_phase_too_short')
        counts.append(count)
    for left, right in zip(segments, segments[1:]):
        if not np.allclose(left.state(left.duration_s),right.state(0.),atol=1e-10,rtol=0):
            raise ValueError('pvat_reference_discontinuous')
    packets, origin = [], 0.
    for segment, count in zip(segments, counts):
        _check_curve(segment, limits, minimum, maximum)
        duration = segment.duration_s/count
        for index in range(count):
            reference = _restriction(segment, index*duration, duration)
            # Reconstruct from the actual endpoint fields, not an asserted fit.
            fitted = quintic_segment(*reference.state(0.),
                                     *reference.state(duration), duration)
            _check_curve(fitted, limits, minimum, maximum)
            packets.append(EncodedWindow(origin+index*duration, fitted))
        origin += segment.duration_s
    return tuple(packets)


def retarget_window(packet, elapsed_s, target, limits=None, **encoding_options):
    """Rebuild from covered predicted PVA; bounded settling family may reject."""
    if not np.isfinite(elapsed_s) or not 0 <= elapsed_s < packet.duration_s:
        raise ValueError('pvat_retarget_window_expired')
    limits = limits or RampLimits()
    _motion_limits(limits)
    plan = build_settling_plan(*packet.state(elapsed_s),target,SettlingLimits(motion=limits))
    return encode_phases((plan.segment,),limits,**encoding_options)
