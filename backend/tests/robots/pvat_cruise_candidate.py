"""Test-only rest-to-rest piecewise reference; never a wire PVAT command."""
from dataclasses import dataclass
from math import comb

import numpy as np

from tests.robots.pvat_interpolation_probe import QuinticSegment,quintic_segment
from tests.robots.pvat_ramp_candidate import RampLimits
from tests.robots.pvat_settling_candidate import _position_range


@dataclass(frozen=True)
class ReferenceWindow:
    segments: tuple[QuinticSegment,...]

    @property
    def duration_s(self):
        return sum(segment.duration_s for segment in self.segments)

    def state(self,time_s):
        if not np.isfinite(time_s) or time_s < 0 or time_s > self.duration_s+1e-12:
            raise ValueError('invalid_sample_time')
        remaining=min(float(time_s),self.duration_s)
        for segment in self.segments:
            if remaining <= segment.duration_s:
                return segment.state(remaining)
            remaining-=segment.duration_s
        return self.segments[-1].state(self.segments[-1].duration_s)


@dataclass(frozen=True)
class CruisePlan:
    segments: tuple[QuinticSegment,...]
    target: tuple[float,...]

    @property
    def duration_s(self):
        return sum(segment.duration_s for segment in self.segments)

    def state(self,time_s):
        if not np.isfinite(time_s) or time_s < 0:
            raise ValueError('invalid_sample_time')
        if time_s >= self.duration_s:
            return self.target,(0.,)*6,(0.,)*6
        return ReferenceWindow(self.segments).state(time_s)

    def window(self,elapsed_s,horizon_s):
        if not np.isfinite(elapsed_s) or elapsed_s < 0:
            raise ValueError('invalid_sample_time')
        if not np.isfinite(horizon_s) or horizon_s <= 0:
            raise ValueError('invalid_duration')
        if elapsed_s >= self.duration_s:
            raise ValueError('plan_complete')
        remaining=min(float(horizon_s),self.duration_s-elapsed_s)
        cursor=float(elapsed_s);pieces=[]
        for index,segment in enumerate(self.segments):
            boundary_roundoff=1e-12 if index<len(self.segments)-1 else 0.
            if cursor >= segment.duration_s-boundary_roundoff:
                cursor=max(0.,cursor-segment.duration_s)
                continue
            duration=min(remaining,segment.duration_s-cursor)
            start,scale=cursor/segment.duration_s,duration/segment.duration_s
            rows=tuple(tuple(sum(row[j]*comb(j,k)*start**(j-k)*scale**k for j in range(k,6))
                             for k in range(6)) for row in segment.coefficients)
            pieces.append(QuinticSegment(rows,duration))
            remaining-=duration;cursor=0.
            if remaining <= 1e-15:break
        return ReferenceWindow(tuple(pieces))


def build_cruise_plan(initial,target,limits=None,*,initial_velocity=(0.,)*6,initial_acceleration=(0.,)*6):
    """Zero initial v/a only. Do not silently zero observed moving states."""
    limits=limits or RampLimits()
    velocity0,acceleration0=_vector(initial_velocity),_vector(initial_acceleration)
    if np.any(velocity0!=0) or np.any(acceleration0!=0):raise ValueError('cruise_requires_rest')
    initial,target,minimum,maximum=(_vector(x) for x in (initial,target,limits.joint_min,limits.joint_max))
    if (not np.all(np.isfinite((limits.duration_s,limits.speed_cap,limits.acceleration_cap)))
            or min(limits.duration_s,limits.speed_cap,limits.acceleration_cap)<=0 or not np.all(minimum<maximum)):
        raise ValueError('invalid_limits')
    if np.any(initial<minimum) or np.any(initial>maximum):raise ValueError('initial_joint_limit')
    if np.any(target<minimum) or np.any(target>maximum):raise ValueError('target_joint_limit')
    delta=target-initial;distance=float(np.max(np.abs(delta)))
    zero=np.zeros(6)
    def segment(q,c1,c3,c4,duration):
        rows=np.stack((q,c1,zero,c3,c4,zero),axis=1)
        return QuinticSegment(tuple(tuple(float(x) for x in row) for row in rows),float(duration))
    if distance==0:
        return CruisePlan((segment(initial,zero,zero,zero,limits.duration_s),),tuple(float(x) for x in target))
    direction=delta/distance
    velocity=min(limits.speed_cap,np.sqrt(distance*limits.acceleration_cap/1.5))
    ramp=1.5*velocity/limits.acceleration_cap
    cruise=max(0.,distance/velocity-ramp)
    displacement=direction*velocity*ramp
    segments=[segment(initial,zero,displacement,-.5*displacement,ramp)]
    middle=np.array(segments[-1].state(ramp)[0])
    if cruise>1e-15:
        segments.append(segment(middle,direction*velocity*cruise,zero,zero,cruise))
        middle=np.array(segments[-1].state(cruise)[0])
    segments.append(segment(middle,displacement,-displacement,.5*displacement,ramp))
    np.testing.assert_allclose(segments[-1].state(ramp),(target,zero,zero),atol=1e-10,rtol=0)
    return CruisePlan(tuple(segments),tuple(float(x) for x in target))


def _vector(value):
    vector=np.asarray(value,dtype=float)
    if vector.shape!=(6,) or not np.all(np.isfinite(vector)):raise ValueError('invalid_vector')
    return vector


def fit_reference_window(plan,elapsed_s,horizon_s,limits=None):
    """Declared quintic endpoint model only, not LEBAI interpolation semantics."""
    limits=limits or RampLimits()
    window=plan.window(elapsed_s,horizon_s)
    fit=quintic_segment(*window.state(0.),*window.state(window.duration_s),window.duration_s)
    speed,acceleration=fit.peak_abs_derivatives();minimum,maximum=_position_range(fit)
    if (max(speed)>limits.speed_cap+1e-12 or max(acceleration)>limits.acceleration_cap+1e-12
            or np.any(minimum<np.array(limits.joint_min)-1e-12) or np.any(maximum>np.array(limits.joint_max)+1e-12)):
        raise ValueError('pvat_window_interpolation_limit')
    return fit
