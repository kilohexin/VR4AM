"""Offline reference + braking-tail admission. No actuator or transport.

A future controller MUST own the entire accepted tail before starting its
reference segment. This synchronous mathematical supervisor is not that
controller, a deadline guarantee, or proof of physical stopping.
"""
from dataclasses import dataclass,field

import numpy as np

from app.control.cartesian_pvat_reference import (JointReference,OfflineCartesianFollower,
    PvatSegment,_evaluate,_range,plan_cartesian_braking)

KNOWN_REJECTIONS=frozenset({
    'cartesian_reference_ik_unreachable','cartesian_reference_ik_branch','cartesian_reference_singular',
    'cartesian_reference_rate_1','cartesian_reference_rate_2','cartesian_reference_rate_3',
    'cartesian_reference_rate_reserve','cartesian_reference_task_rate','cartesian_reference_joint_bounds',
    'cartesian_reference_corridor','cartesian_reference_unresolved_interval',
    'cartesian_reference_optimizer_infeasible','cartesian_brake_no_feasible_plan',
    'cartesian_reference_bundle_workspace','cartesian_reference_bundle_workspace_unresolved',
})


@dataclass(frozen=True)
class ReferenceBundle:
    segment: PvatSegment
    braking_tail: tuple[PvatSegment,...]
    mode: str
    demand_scale: float
    reason: str|None=None
    position_bound_m: float|None=None
    optimizer_iterations: int|None=None
    native_execution_verified: bool=field(default=False,init=False)


def _workspace_checked(segment,model,lower,upper):
    c=segment.coefficients();stack=[(0.,1.,0)]
    while stack:
        lo,hi,depth=stack.pop();mid=(lo+hi)/2;q=_evaluate(c,mid)
        p=np.array(model.pose(q).p);ql,qh=_range(c,lo,hi)
        if np.any(p<lower) or np.any(p>upper):raise ValueError('cartesian_reference_bundle_workspace')
        radius=model.position_lipschitz@np.maximum(np.abs(ql-q),np.abs(qh-q))
        radius+=256*np.finfo(float).eps*(1+np.linalg.norm(p))
        if np.all(p-radius>=lower) and np.all(p+radius<=upper):continue
        if depth>=12:raise ValueError('cartesian_reference_bundle_workspace_unresolved')
        stack.extend(((lo,mid,depth+1),(mid,hi,depth+1)))


class OfflineReferenceSupervisor:
    def __init__(self,model,start,*,workspace_span=.35,scales=(1.,.5,.25,0.),**follower_options):
        if (not np.isfinite(workspace_span) or workspace_span<=0 or not scales
                or any(not np.isfinite(s) or not 0<=s<=1 for s in scales)):
            raise ValueError('cartesian_reference_bundle_configuration')
        if not isinstance(start,JointReference) or max(abs(x) for x in start.v+start.a)>1e-12:
            raise ValueError('cartesian_reference_bundle_requires_quiet_origin')
        self.model=model;self.state=start;self.options=dict(follower_options);self.scales=tuple(scales)
        base=np.array(model.pose(start.q).p);self.lower=base-workspace_span;self.upper=base+workspace_span
        self._binding=(model.tool,tuple(model.lower),tuple(model.upper),tuple(model.position_lipschitz))
        seed=OfflineCartesianFollower(model,start,**self.options)
        self._period=seed.period;self._validation=dict(seed.validation)
        initial,_=plan_cartesian_braking(model,start,period=self._period,**self._validation)
        self._tail=initial.segments;self._tail_checks=initial.checks;self.phase='following';self.reason=None
        for segment in self._tail:_workspace_checked(segment,model,self.lower,self.upper)

    def _check(self):
        if self._binding!=(self.model.tool,tuple(self.model.lower),tuple(self.model.upper),tuple(self.model.position_lipschitz)):
            self.phase='fault';raise ValueError('cartesian_reference_bundle_model_changed')
        if self.phase in ('ready','fault'):raise ValueError('cartesian_reference_bundle_requires_capture')

    def finish(self,reason='release'):
        self._check()
        if self.phase=='following':self.phase='finishing';self.reason=reason
        return self._consume_tail()

    def _consume_tail(self):
        if not self._tail:
            self.phase='fault';raise ValueError('cartesian_reference_bundle_missing_tail')
        segment=self._tail[0]
        if segment.start!=self.state:
            self.phase='fault';raise ValueError('cartesian_reference_bundle_chain')
        check=self._tail_checks[0]
        self._tail=self._tail[1:];self._tail_checks=self._tail_checks[1:];self.state=segment.end
        if not self._tail:self.phase='ready'
        return ReferenceBundle(segment,self._tail,'braking',0.,self.reason,check['position_bound_m'])

    def advance(self,target):
        self._check()
        if self.phase=='finishing':return self._consume_tail()
        for scale in self.scales:
            options=dict(self.options)
            if scale:
                options['lookahead_position']=options.get('lookahead_position',.02)*scale
                options['lookahead_rotation']=options.get('lookahead_rotation',.08)*scale
                requested=target
            else:requested=self.model.pose(self.state.q)
            candidate=OfflineCartesianFollower(self.model,self.state,**options)
            try:
                segment,check=candidate.advance(requested)
                brake,_=plan_cartesian_braking(self.model,segment.end,period=self._period,**self._validation)
                chain=(segment,)+brake.segments
                if any(a.end!=b.start for a,b in zip(chain,chain[1:])):
                    raise ValueError('cartesian_reference_bundle_chain')
                for item in chain:_workspace_checked(item,self.model,self.lower,self.upper)
            except ValueError as error:
                if str(error) not in KNOWN_REJECTIONS:
                    self.phase='fault';raise
                continue  # Pure mathematical backoff, no SDK or command retry.
            except Exception:
                self.phase='fault';raise
            self.state=segment.end;self._tail=brake.segments;self._tail_checks=brake.checks
            return ReferenceBundle(segment,self._tail,'following',float(scale),
                position_bound_m=check['position_bound_m'],optimizer_iterations=check['optimizer_iterations'])
        return self.finish('no_candidate_with_braking_tail')
