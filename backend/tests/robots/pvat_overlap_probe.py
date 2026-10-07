"""TEST ONLY: covered PVA renewal; no controller/SDK/plant timing model.

Full plans are desired references, issued windows are declared predictions.
Neither establishes actual robot state or controller acceptance. Brake
branches only prove this polynomial family at renewal and terminal states,
not safe stops at every intermediate instant or physical stopping.
"""
from dataclasses import dataclass

import numpy as np

from tests.robots.pvat_interpolation_probe import QuinticSegment,quintic_segment
from tests.robots.pvat_phase_encoding_probe import EncodedWindow,_check_curve,_motion_limits
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import SettlingLimits,_vector,build_settling_plan


@dataclass(frozen=True)
class OverlapReference:
    plan: object
    reference_origin_ns: int
    origin_ns: int
    packet: EncodedWindow
    limits: RampLimits
    brake_reserve: tuple[QuinticSegment,...]


def build_brake_reserve(q, v, a, limits=None):
    """Neutralize a linearly, then reduce all v smoothly to zero."""
    limits=limits or RampLimits()
    minimum,maximum=_motion_limits(limits)
    q,v,a=(_vector(value) for value in (q,v,a))
    duration=limits.duration_s
    zero=np.zeros(6)
    rows=np.stack((q,v*duration,.5*a*duration**2,-a*duration**2/6,zero,zero),axis=1)
    neutral=QuinticSegment(tuple(tuple(float(x) for x in row) for row in rows),duration)
    q1,v1,_=neutral.state(duration);q1,v1=np.array(q1),np.array(v1)
    braking=max(duration,1.5*float(np.max(np.abs(v1)))/limits.acceleration_cap)
    rows=np.stack((q1,v1*braking,zero,-v1*braking,.5*v1*braking,zero),axis=1)
    decelerate=QuinticSegment(tuple(tuple(float(x) for x in row) for row in rows),braking)
    try:
        for segment in (neutral,decelerate):_check_curve(segment,limits,minimum,maximum)
    except ValueError:
        raise ValueError('pvat_brake_reserve_limit') from None
    return neutral,decelerate


def _packet(plan,reference_origin_ns,now_ns,initial,limits):
    duration=limits.duration_s
    reference_elapsed=(now_ns-reference_origin_ns)/1e9
    end=plan.state(reference_elapsed+duration)
    segment=quintic_segment(*initial,*end,duration)
    minimum,maximum=_motion_limits(limits)
    _check_curve(segment,limits,minimum,maximum)
    reserve=build_brake_reserve(*segment.state(duration),limits)
    return OverlapReference(plan,reference_origin_ns,now_ns,
                            EncodedWindow(reference_elapsed,segment),limits,reserve)


def start_overlap(plan, origin_ns, limits=None):
    if type(origin_ns) is not int or origin_ns<0:raise ValueError('pvat_overlap_time')
    limits=limits or RampLimits()
    _motion_limits(limits)
    initial=plan.state(0.)
    build_brake_reserve(*initial,limits)
    return _packet(plan,origin_ns,origin_ns,initial,limits)


def renew_overlap(previous, now_ns, target=None):
    if (type(now_ns) is not int or now_ns<=previous.origin_ns
            or now_ns>=previous.origin_ns+int(round(previous.packet.duration_s*1e9))):
        raise ValueError('pvat_overlap_time')
    elapsed=(now_ns-previous.origin_ns)/1e9
    initial=previous.packet.state(elapsed)
    build_brake_reserve(*initial,previous.limits)
    desired=previous.plan.target if target is None else tuple(float(x) for x in _vector(target))
    plan,reference_origin=previous.plan,previous.reference_origin_ns
    if desired!=plan.target:
        plan=build_settling_plan(*initial,desired,SettlingLimits(motion=previous.limits))
        reference_origin=now_ns
    return _packet(plan,reference_origin,now_ns,initial,previous.limits)
