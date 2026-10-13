"""Test-only continuous reference, not a robot command or stopping proof.

Neutralizes initial acceleration before comparing settling and brake/cruise
families. Braking here is a reference construction, not a robot stop. The
bridge is a fixed-duration, deliberately limited family: rejecting it does
not prove no other trajectory exists. Target reversal can require overshoot.
PVA bounds do not constrain jerk, Cartesian error, collision or controller
interpolation. Full-plan predictions never extend issued-window coverage.
"""
from dataclasses import dataclass, replace

import numpy as np

from tests.robots.pvat_cruise_candidate import CruisePlan, build_cruise_plan
from tests.robots.pvat_interpolation_probe import QuinticSegment
from tests.robots.pvat_settling_candidate import (
    SettlingLimits, _limits, _position_range, _vector, build_settling_plan,
)


@dataclass(frozen=True)
class ContinuousArrivalPlan:
    reference: CruisePlan
    limits: SettlingLimits

    @property
    def target(self):
        return self.reference.target

    @property
    def duration_s(self):
        return self.reference.duration_s

    @property
    def segments(self):
        return self.reference.segments

    def state(self, elapsed_s):
        return self.reference.state(elapsed_s)

    def window(self, elapsed_s, horizon_s):
        return self.reference.window(elapsed_s, horizon_s)


def build_continuous_arrival(q0, v0, a0, target, limits=SettlingLimits()):
    q0,v0,a0,target=(_vector(x) for x in (q0,v0,a0,target))
    limits=_limits(limits)
    motion=limits.motion
    if np.max(np.abs(v0))>motion.speed_cap+1e-12:
        raise ValueError('initial_speed_limit')
    if np.max(np.abs(a0))>motion.acceleration_cap+1e-12:
        raise ValueError('initial_acceleration_limit')
    minimum,maximum=np.array(motion.joint_min),np.array(motion.joint_max)
    if np.any(q0<minimum) or np.any(q0>maximum):raise ValueError('initial_joint_limit')
    if np.any(target<minimum) or np.any(target>maximum):raise ValueError('target_joint_limit')
    if np.all(a0==0):
        return _finish((),(q0,v0,a0),target,limits)
    t=motion.duration_s
    # a(s)=a0*(1-s/T); integrate twice instead of using terminal v*T.
    rows=np.stack((q0,v0*t,.5*a0*t*t,-a0*t*t/6,np.zeros(6),np.zeros(6)),axis=1)
    if not np.all(np.isfinite(rows)):raise ValueError('no_feasible_acceleration_bridge')
    bridge=QuinticSegment(tuple(tuple(float(x) for x in row) for row in rows),float(t))
    speed,acceleration=bridge.peak_abs_derivatives()
    lower,upper=_position_range(bridge)
    if (max(speed)>motion.speed_cap+1e-12 or max(acceleration)>motion.acceleration_cap+1e-12
            or np.any(lower<minimum-1e-12) or np.any(upper>maximum+1e-12)):
        raise ValueError('no_feasible_acceleration_bridge')
    # Use the exact evaluated end PVA, including roundoff; do not reset feedback.
    return _finish((bridge,),bridge.state(t),target,limits)


def _finish(prefix,state,target,limits):
    motion=limits.motion
    remaining=limits.max_duration_s-sum(s.duration_s for s in prefix)
    if remaining<=0:raise ValueError('no_feasible_continuous_arrival')
    candidates=[]
    if remaining>=motion.duration_s:
        try:
            settled=build_settling_plan(*state,target,replace(limits,max_duration_s=remaining))
        except ValueError as error:
            if str(error)!='no_feasible_settling_plan':raise
        else:
            candidates.append(CruisePlan((*prefix,settled.segment),settled.target))
    q,v,_=(np.array(x) for x in state)
    braking=()
    if np.any(v!=0):
        duration=max(motion.duration_s,1.5*float(np.max(np.abs(v)))/motion.acceleration_cap)
        # v(s)=v0*(1-3s²+2s³), a at both ends analytically zero.
        # The prior bridge's terminal a is also analytically zero; its tiny
        # evaluator residual is roundoff, never an observed state reset.
        rows=np.stack((q,v*duration,np.zeros(6),-v*duration,.5*v*duration,np.zeros(6)),axis=1)
        segment=QuinticSegment(tuple(tuple(float(x) for x in row) for row in rows),float(duration))
        speed,acceleration=segment.peak_abs_derivatives()
        lower,upper=_position_range(segment)
        if (max(speed)<=motion.speed_cap+1e-12 and max(acceleration)<=motion.acceleration_cap+1e-12
                and np.all(lower>=np.array(motion.joint_min)-1e-12)
                and np.all(upper<=np.array(motion.joint_max)+1e-12)):
            braking=(segment,)
            q=q+.5*v*duration
        else:
            if not candidates:raise ValueError('no_feasible_continuous_arrival')
            return ContinuousArrivalPlan(min(candidates,key=lambda p:p.duration_s),limits)
    try:
        cruise=build_cruise_plan(q,target,motion)
    except ValueError as error:
        # The alternative's derived start can fail its stricter boundary
        # check. Keep an independently valid settling plan without clipping
        # observed state, expanding bounds or swallowing unrelated failures.
        if str(error)!='initial_joint_limit':raise
        if not candidates:raise ValueError('no_feasible_continuous_arrival') from error
        return ContinuousArrivalPlan(min(candidates,key=lambda p:p.duration_s),limits)
    accelerated=CruisePlan((*prefix,*braking,*cruise.segments),cruise.target)
    if accelerated.duration_s<=limits.max_duration_s+1e-12:candidates.append(accelerated)
    if not candidates:raise ValueError('no_feasible_continuous_arrival')
    return ContinuousArrivalPlan(min(candidates,key=lambda p:p.duration_s),limits)


def retarget_continuous_arrival(plan, elapsed_s, target, limits=None):
    state=plan.state(elapsed_s)
    target=_vector(target)
    selected=_limits(plan.limits if limits is None else limits)
    if tuple(float(x) for x in target)==plan.target and selected==plan.limits:
        return plan,float(elapsed_s)
    return build_continuous_arrival(*state,target,selected),0.
