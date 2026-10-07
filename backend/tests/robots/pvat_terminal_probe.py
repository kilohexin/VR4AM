"""TEST ONLY: monotonic finite terminal reference, not physical arrival.

No SDK calls, feedback estimation, dispatcher, completion acknowledgement or
fault clearing. At terminal expiry the existing window guard still faults.
"""
from dataclasses import dataclass,replace

import numpy as np

from tests.robots.pvat_interpolation_probe import QuinticSegment,quintic_segment
from tests.robots.pvat_overlap_probe import build_brake_reserve
from tests.robots.pvat_phase_encoding_probe import EncodedWindow,_check_curve,_motion_limits,_restriction
from tests.robots.pvat_settling_candidate import _position_range,_vector


@dataclass(frozen=True)
class TerminalReference:
    segment: QuinticSegment
    target: tuple[float,...]
    reference_origin_ns: int
    deadline_ns: int
    origin_ns: int
    checked_ns: int
    packet: EncodedWindow
    limits: object
    min_duration_s: float
    brake_reserve: tuple[QuinticSegment,...]
    refresh: bool


def _monotonic(segment,initial_q,target):
    q0,q1=np.array(initial_q),np.array(target)
    lower,upper=_position_range(segment)
    rows=tuple(tuple(np.pad(np.polynomial.polynomial.polyder(row)/segment.duration_s,(0,1)))
               for row in segment.coefficients)
    vmin,vmax=_position_range(QuinticSegment(rows,segment.duration_s))
    direction=np.sign(q1-q0)
    if (np.any(lower<np.minimum(q0,q1)-1e-12) or np.any(upper>np.maximum(q0,q1)+1e-12)
            or np.any((direction>0)&(vmin < -1e-12))
            or np.any((direction<0)&(vmax > 1e-12))
            or np.any((direction==0)&((vmin < -1e-12)|(vmax > 1e-12)))):
        raise ValueError('pvat_terminal_not_monotonic')


def _packet(previous,now_ns):
    elapsed=(now_ns-previous.reference_origin_ns)/1e9
    remaining=(previous.deadline_ns-now_ns)/1e9
    duration=min(previous.limits.duration_s,remaining)
    if duration<previous.min_duration_s:raise ValueError('pvat_terminal_short_window')
    restricted=_restriction(previous.segment,elapsed,duration)
    fitted=quintic_segment(*restricted.state(0.),*restricted.state(duration),duration)
    minimum,maximum=_motion_limits(previous.limits)
    _check_curve(fitted,previous.limits,minimum,maximum)
    _monotonic(fitted,restricted.state(0.)[0],previous.target)
    reserve=build_brake_reserve(*fitted.state(duration),previous.limits)
    return replace(previous,origin_ns=now_ns,checked_ns=now_ns,
                   packet=EncodedWindow(elapsed,fitted),brake_reserve=reserve,refresh=True)


def start_terminal(previous, now_ns, *, min_duration_s=.02):
    if (type(now_ns) is not int or now_ns<=previous.origin_ns
            or now_ns>=previous.origin_ns+round(previous.packet.duration_s*1e9)):
        raise ValueError('pvat_terminal_time')
    minimum,maximum=_motion_limits(previous.limits)
    if not np.isfinite(min_duration_s) or not 0<min_duration_s<=previous.limits.duration_s:
        raise ValueError('invalid_terminal_limits')
    deadline=previous.reference_origin_ns+round(previous.plan.duration_s*1e9)
    duration=(deadline-now_ns)/1e9
    if not min_duration_s<=duration<=2*previous.limits.duration_s:
        raise ValueError('pvat_terminal_region')
    initial=previous.packet.state((now_ns-previous.origin_ns)/1e9)
    target=tuple(float(x) for x in _vector(previous.plan.target))
    segment=quintic_segment(*initial,target,(0.,)*6,(0.,)*6,duration)
    _monotonic(segment,initial[0],target)
    _check_curve(segment,previous.limits,minimum,maximum)
    build_brake_reserve(*initial,previous.limits)
    draft=TerminalReference(segment,target,now_ns,deadline,now_ns,now_ns,
                            previous.packet,previous.limits,float(min_duration_s),(),False)
    return _packet(draft,now_ns)


def renew_terminal(previous, now_ns, target=None):
    if (type(now_ns) is not int or now_ns<=previous.checked_ns
            or now_ns>=previous.deadline_ns
            or now_ns>=previous.origin_ns+round(previous.packet.duration_s*1e9)):
        raise ValueError('pvat_terminal_time')
    if target is not None and tuple(float(x) for x in _vector(target))!=previous.target:
        raise ValueError('pvat_terminal_target_changed')
    initial=previous.packet.state((now_ns-previous.origin_ns)/1e9)
    build_brake_reserve(*initial,previous.limits)
    if previous.origin_ns+round(previous.packet.duration_s*1e9)==previous.deadline_ns:
        # Final window is still covered; never re-fit it into a longer hold.
        return replace(previous,checked_ns=now_ns,refresh=False)
    return _packet(previous,now_ns)
