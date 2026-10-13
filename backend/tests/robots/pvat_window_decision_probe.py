"""Test-only mathematical decision; no dispatcher, robot stop or SDK calls.

Caller ages/generation/elapsed/feedback are explicit assumptions, not measured
execution evidence. Matching PVA does not establish freshness or controller
interpolation. Discarding a reference cannot retract queued motion or verify
physical stopping. No production code imports this experiment.
"""
from dataclasses import dataclass

import numpy as np

from tests.robots.pvat_continuous_arrival_candidate import build_continuous_arrival
from tests.robots.pvat_phase_encoding_probe import (
    EncodedWindow, _check_curve, _motion_limits, encode_phases,
)
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import SettlingLimits


@dataclass(frozen=True)
class WindowDecision:
    packets: tuple
    discard_reference: bool
    reason: str | None


def retarget_arrival_window(packet, elapsed_s, target, *, issued_generation,
                            current_generation, frame_age_ns, observed_pva,
                            limits=None, tolerances=(1e-10,1e-10,1e-10)):
    def discard(reason):return WindowDecision((),True,reason)
    if (type(issued_generation) is not int or type(current_generation) is not int
            or min(issued_generation,current_generation)<0 or issued_generation!=current_generation):
        return discard('pvat_reference_generation_changed')
    if type(frame_age_ns) is not int or not 0<=frame_age_ns<100_000_000:
        return discard('pvat_vr_frame_expired')
    if not isinstance(packet,EncodedWindow):return discard('invalid_issued_reference')
    try:
        coefficients=np.asarray(packet.segment.coefficients,dtype=float)
        valid=(coefficients.shape==(6,6) and np.all(np.isfinite(coefficients))
               and np.isfinite(packet.duration_s) and .02<=packet.duration_s<=.08)
    except (TypeError,ValueError,AttributeError):
        valid=False
    if not valid:return discard('invalid_issued_reference')
    try:
        covered=(type(elapsed_s) is not bool and np.isfinite(elapsed_s)
                 and 0<=elapsed_s<packet.duration_s)
    except (TypeError,ValueError):
        covered=False
    if not covered:return discard('pvat_issued_window_expired')
    try:
        tolerance=np.asarray(tolerances,dtype=float)
        valid=(tolerance.shape==(3,) and np.all(np.isfinite(tolerance)) and np.all(tolerance>=0))
    except (TypeError,ValueError):valid=False
    if not valid:return discard('invalid_reference_tolerances')
    try:
        observed=np.asarray(observed_pva,dtype=float)
        valid=observed.shape==(3,6) and np.all(np.isfinite(observed))
    except (TypeError,ValueError):valid=False
    if not valid:return discard('pvat_reference_feedback_unknown')
    try:
        target_vector=np.asarray(target,dtype=float)
        valid=target_vector.shape==(6,) and np.all(np.isfinite(target_vector))
    except (TypeError,ValueError):valid=False
    if not valid:return discard('invalid_reference_target')
    limits=RampLimits() if limits is None else limits
    if not isinstance(limits,RampLimits):return discard('invalid_motion_limits')
    try:
        minimum,maximum=_motion_limits(limits)
        _check_curve(packet.segment,limits,minimum,maximum)
        expected=packet.state(float(elapsed_s))
        if np.any(np.max(np.abs(observed-np.array(expected)),axis=1)>tolerance):
            return discard('pvat_reference_feedback_mismatch')
        # Use supplied feedback, not a predicted state disguised as observation.
        # The test caller must independently establish its actual provenance.
        plan=build_continuous_arrival(*observed,target_vector,SettlingLimits(motion=limits))
        packets=encode_phases(plan.segments,limits)
    except ValueError as error:
        return discard(str(error))
    return WindowDecision(packets,False,None)
