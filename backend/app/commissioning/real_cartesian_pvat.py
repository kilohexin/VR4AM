"""Transport-free finite physical-trial contract and receipt ownership.

This is NOT a hardware launcher or authorization. It never opens a socket,
imports an SDK, clears a historical fault, or certifies arrival from returns.
A future native executor must additionally enforce live feedback, arrival,
stop verification and the independently approved physical scope.
"""
from dataclasses import dataclass, field
from hashlib import sha256
import json
import math
import re

import numpy as np
from scipy.spatial.transform import Rotation

from app.commissioning.real_pvat_reference import build_empty_trial
from app.control.cartesian_pvat_reference import (JointReference, PvatSegment, _range,
    MdhReferenceModel, _curve_from_reference, validate_segment)
from app.control.cartesian_tool_binding import tool_digest
from app.control.finite_position_types import COARSE_GRASP_ARRIVAL
from app.schemas.messages import Pose


def _digest(value):
    if type(value) is not str or re.fullmatch('[a-f0-9]{64}', value) is None:
        raise ValueError('pvat_identity')
    return value


def _vector(value, size):
    if len(value) != size or any(type(v) not in (float, int) or not math.isfinite(v) for v in value):
        raise ValueError('pvat_vector')
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class FinitePvatContract:
    origin: JointReference
    points: tuple[PvatSegment, ...]
    joint_lower: tuple
    joint_upper: tuple
    workspace_lower: tuple
    workspace_upper: tuple
    tool_sha256: str
    config_sha256: str
    source_sha256: str
    tool_report_sha256: str
    model_tool: object
    motion_admitted: bool = field(default=False, init=False)

    def __post_init__(self):
        for name in ('tool_sha256', 'config_sha256', 'source_sha256', 'tool_report_sha256'):
            _digest(getattr(self, name))
        for name, size in (('joint_lower',6),('joint_upper',6),('workspace_lower',3),('workspace_upper',3)):
            object.__setattr__(self, name, _vector(getattr(self,name),size))
        object.__setattr__(self,'points',tuple(self.points))
        if (type(self.origin) is not JointReference or any(self.origin.v+self.origin.a)
                or len(self.points)!=20 or any(type(p) is not PvatSegment or p.duration!=.08 for p in self.points)
                or self.points[0].start!=self.origin
                or any(a.end!=b.start for a,b in zip(self.points,self.points[1:]))
                or any(self.points[-1].end.v+self.points[-1].end.a)):
            raise ValueError('pvat_finite_chain')
        for lo,hi in ((self.joint_lower,self.joint_upper),(self.workspace_lower,self.workspace_upper)):
            if any(a>=b for a,b in zip(lo,hi)):raise ValueError('pvat_bounds')
        # Direct construction/dataclasses.replace must preserve the same
        # validated scope as the factory; a digest alone proves no geometry.
        model=MdhReferenceModel(self.model_tool,self.joint_lower,self.joint_upper)
        if tool_digest(self.model_tool)!=self.tool_sha256:raise ValueError('pvat_tool_binding')
        base=model.pose(self.origin.q)
        goal=base.model_copy(update={'p':(base.p[0],base.p[1],base.p[2]+.002)})
        end=model.pose(self.points[-1].end.q)
        if (math.dist(end.p,goal.p)>1e-9 or
                (Rotation.from_quat(end.q).inv()*Rotation.from_quat(goal.q)).magnitude()>1e-9):
            raise ValueError('pvat_endpoint_scope')
        curve=_curve_from_reference(model,self.origin,goal,1.6)
        checks=[validate_segment(point,model,curve,i*.08,speed=.05,acceleration=.2)
            for i,point in enumerate(self.points)]
        lo,hi=_range(curve.coefficients);margin=max(c['position_bound_m'] for c in checks)
        if any(a-margin<l or b+margin>u for a,b,l,u in
                zip(lo[:3],hi[:3],self.workspace_lower,self.workspace_upper)):
            raise ValueError('pvat_workspace')

    def document(self):
        def reference(r):return dict(q=r.q,v=r.v,a=r.a)
        return dict(schema=1,axis='base_z',distance_m=.002,duration_s=1.6,
            origin=reference(self.origin),points=[dict(start=reference(p.start),end=reference(p.end),
                duration=p.duration) for p in self.points],joint_lower=self.joint_lower,joint_upper=self.joint_upper,
            workspace_lower=self.workspace_lower,workspace_upper=self.workspace_upper,
            tool_sha256=self.tool_sha256,config_sha256=self.config_sha256,source_sha256=self.source_sha256,
            tool_report_sha256=self.tool_report_sha256,motion_admitted=False,
            native_interpolation_verified=False)

    @property
    def sha256(self):
        return sha256(json.dumps(self.document(),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def build_contract(model,start,*,workspace_lower,workspace_upper,config_sha256,source_sha256,tool_report_sha256):
    """Validate a fixed +Z candidate without moving/recentering the supplied box."""
    for value in (config_sha256,source_sha256,tool_report_sha256):_digest(value)
    lower=np.array(_vector(workspace_lower,3));upper=np.array(_vector(workspace_upper,3))
    if np.any(lower>=upper):raise ValueError('pvat_workspace')
    trial=build_empty_trial(model,start,axis='z')
    # The model checker bounds the WHOLE FK curve's distance from the
    # reference. Enclose that reference polynomial, then add its error ball.
    lo,hi=_range(trial.plan.curve.coefficients)
    margin=max(c['position_bound_m'] for c in trial.plan.checks)
    if np.any(lo[:3]-margin<lower) or np.any(hi[:3]+margin>upper):
        raise ValueError('pvat_workspace')
    return FinitePvatContract(start,trial.plan.segments,tuple(map(float,model.lower)),tuple(map(float,model.upper)),
        tuple(map(float,lower)),tuple(map(float,upper)),trial.tool_sha256,config_sha256,source_sha256,
        tool_report_sha256,model.tool)


class FinitePvatWindow:
    """Single-use receipt ledger; returned points are not permission to send.

    Caller must enforce the physical scope and continuous feedback separately.
    Monotonic nanoseconds are local observations; controller clock validation
    belongs to the native bridge. No timeout/late reply can restore authority.
    """
    def __init__(self,contract,connection,generation):
        if type(contract) is not FinitePvatContract or connection is None:
            raise ValueError('pvat_contract')
        if type(generation) is not int or generation<0:raise ValueError('pvat_generation')
        self.contract=contract;self.connection=connection;self.generation=generation
        self.phase='new';self.fault=None;self.entered=0;self.returned_count=0
        self.pending_index=None;self.pending_since=None;self.last_enter=None;self.epoch=None;self.last_clock=None
        self.motion_unknown=False;self.stop_unverified=False;self.stop_entered_ns=None;self.stop_returned_ns=None

    @property
    def passed(self):
        # This component deliberately cannot pass a physical trial.
        return False

    def revoke(self,reason):
        if type(reason) is not str or not reason:raise ValueError('pvat_reason')
        self.fault=self.fault or reason;self.phase='fault'
        self.motion_unknown |= self.pending_index is not None
        self.stop_unverified |= self.entered>0

    def _reject(self,reason):
        self.revoke(reason)
        raise ValueError(reason)

    def _clock(self,now):
        if type(now) is not int or now<0 or (self.last_clock is not None and now<self.last_clock):
            self._reject('pvat_clock')
        self.last_clock=now

    def _owner(self,connection,generation):
        if connection is not self.connection or type(generation) is not int or generation!=self.generation:
            self._reject('pvat_owner')

    def admit(self,connection,generation,contract_sha256,tool_sha256,actual,target,state,captured_ns,now_ns):
        self._clock(now_ns);self._owner(connection,generation)
        if self.fault or self.phase!='new':self._reject('pvat_reentry')
        if contract_sha256!=self.contract.sha256 or tool_sha256!=self.contract.tool_sha256:
            self._reject('pvat_binding')
        if type(captured_ns) is not int or not 0<=captured_ns<=now_ns or now_ns-captured_ns>=100_000_000:
            self._reject('pvat_snapshot_age')
        if (state!='IDLE' or any(type(r) is not JointReference for r in (actual,target))
                or any(actual.v+actual.a+target.v+target.a)
                or any(abs(q-o)>1e-6 for r in (actual,target) for q,o in zip(r.q,self.contract.origin.q))):
            self._reject('pvat_origin')
        self.phase='admitted';self.admitted_ns=now_ns;self.origin_captured_ns=captured_ns

    def enter(self,connection,generation,index,now_ns):
        self._clock(now_ns);self._owner(connection,generation)
        if self.fault or self.phase not in ('admitted','sending'):self._reject('pvat_not_active')
        if self.pending_index is not None:self._reject('pvat_write_pending')
        if type(index) is not int or index!=self.entered+1 or not 1<=index<=20:self._reject('pvat_index')
        if self.epoch is None:
            if now_ns-self.origin_captured_ns>=100_000_000:self._reject('pvat_snapshot_age')
            self.epoch=now_ns
        if self.last_enter is not None and now_ns-self.last_enter>=80_000_000:self._reject('pvat_cadence')
        # At most two nominal intervals ahead. This is a scheduling bound,
        # NOT a claim about the unknown controller queue implementation.
        if index*80_000_000-(now_ns-self.epoch)>160_000_000:self._reject('pvat_queue_lead')
        self.entered=index;self.pending_index=index;self.pending_since=now_ns
        self.last_enter=now_ns;self.phase='sending'
        return self.contract.points[index-1]

    def returned(self,index,now_ns):
        self._clock(now_ns)
        if type(index) is not int or self.pending_index!=index:self._reject('pvat_unmatched_return')
        if now_ns-self.pending_since>=60_000_000:self.revoke('pvat_reply_deadline')
        self.pending_index=None;self.pending_since=None;self.returned_count+=1
        if not self.fault and self.returned_count==20:self.phase='awaiting_arrival'

    def claim_exception_stop(self,connection,generation,now_ns):
        self._clock(now_ns);self._owner(connection,generation)
        if not self.fault:self._reject('pvat_stop_without_failure')
        if not self.entered:return False
        if self.stop_entered_ns is not None:self._reject('pvat_stop_already_consumed')
        self.stop_entered_ns=now_ns;self.stop_unverified=True
        return True

    def stop_returned(self,now_ns):
        self._clock(now_ns)
        if self.stop_entered_ns is None or self.stop_returned_ns is not None:self._reject('pvat_stop_receipt')
        self.stop_returned_ns=now_ns
        # 200 ms reply, 300 ms stable and 500 ms verification remain the
        # external stop verifier's requirements. A return alone clears NONE.


class FinitePvatFeedback:
    """Sampled empty +Z arrival evidence, never a physical stop certificate.

    Uses the already selected coarse arrival policy, anchored stability and
    tail observations. It does not prove unsampled physical motion, native
    interpolation, or load/contact performance. It cannot clear a window fault.
    """
    hardware_verified=False

    def __init__(self,window,model):
        c=window.contract
        if (tool_digest(model.tool)!=c.tool_sha256 or tuple(model.lower)!=c.joint_lower
                or tuple(model.upper)!=c.joint_upper):raise ValueError('pvat_feedback_model')
        self.window=window;self.model=model;self.contract=c
        self.base=model.pose(c.origin.q);self.goal=model.pose(c.points[-1].end.q)
        self.previous_ns=window.admitted_ns if window.phase=='admitted' else None
        if self.previous_ns is None:raise ValueError('pvat_feedback_before_admission')
        self.stable=None;self.arrival_ns=None;self.arrival=None;self.tail_complete=False
        self.acceleration_exceedances=0;self.samples=0

    @property
    def sampled_passed(self):
        return bool(self.tail_complete and not self.window.fault and not self.window.motion_unknown)

    def _angle(self,a,b):
        return float((Rotation.from_quat(a.q).inv()*Rotation.from_quat(b.q)).magnitude())

    def observe(self,connection,generation,actual,target,tcp,state,captured_ns,now_ns):
        w=self.window;c=self.contract;p=COARSE_GRASP_ARRIVAL
        w._clock(now_ns);w._owner(connection,generation)
        if w.fault:w._reject('pvat_feedback_after_fault')
        if (type(captured_ns) is not int or not 0<=captured_ns<=now_ns
                or now_ns-captured_ns>=100_000_000
                or captured_ns<=self.previous_ns or now_ns-self.previous_ns>=100_000_000):
            w._reject('pvat_feedback_gap')
        if (type(actual) is not JointReference or type(target) is not JointReference
                or type(tcp) is not Pose or state not in ('IDLE','MOVING')
                or any(not math.isfinite(x) for x in tcp.p+tcp.q)
                or abs(math.hypot(*tcp.q)-1)>1e-6):w._reject('pvat_feedback_shape')
        self.previous_ns=captured_ns;self.samples+=1
        if self.samples>512:w._reject('pvat_feedback_budget')
        for r in (actual,target):
            if any(q<lo or q>hi for q,lo,hi in zip(r.q,c.joint_lower,c.joint_upper)):
                w._reject('pvat_feedback_joint_bounds')
            if any(abs(v)>.05+1e-10 for v in r.v):w._reject('pvat_feedback_speed')
        if any(abs(a)>.2+1e-10 for a in target.a):w._reject('pvat_target_acceleration')
        peak=max(map(abs,actual.a))
        self.acceleration_exceedances=self.acceleration_exceedances+1 if peak>.2 else 0
        if peak>.4 or self.acceleration_exceedances>=3:w._reject('pvat_actual_acceleration')
        if any(x<lo or x>hi for x,lo,hi in zip(tcp.p,c.workspace_lower,c.workspace_upper)):
            w._reject('pvat_feedback_workspace')
        # Whole candidate is a fixed vertical line; observed target and
        # actual TCP must stay in its 1 mm corridor, not just at its endpoint.
        for observed in (tcp,self.model.pose(actual.q),self.model.pose(target.q)):
            nearest=(self.base.p[0],self.base.p[1],min(self.goal.p[2],max(self.base.p[2],observed.p[2])))
            if math.dist(observed.p,nearest)>.001 or self._angle(observed,self.base)>p.orientation_error_rad:
                w._reject('pvat_feedback_corridor')
        actual_pose=self.model.pose(actual.q);target_pose=self.model.pose(target.q)
        if math.dist(tcp.p,actual_pose.p)>.001 or self._angle(tcp,actual_pose)>p.orientation_error_rad:
            w._reject('pvat_feedback_model_disagreement')
        if w.epoch is None:return
        if self.arrival_ns is None and now_ns-w.epoch>=p.completion_ns:w._reject('pvat_arrival_deadline')
        end=c.points[-1].end
        endpoint=(math.dist(tcp.p,self.goal.p)<=p.endpoint_error_m
            and self._angle(tcp,self.goal)<=p.orientation_error_rad
            and tcp.p[2]-self.base.p[2]>=.001
            and math.dist(target_pose.p,self.goal.p)<=p.endpoint_error_m
            and self._angle(target_pose,self.goal)<=p.orientation_error_rad
            and all(abs(q-goal)<=math.radians(.1) for r in (actual,target) for q,goal in zip(r.q,end.q)))
        if self.arrival_ns is not None:
            _,anchor,tcp_anchor=self.arrival
            if (not endpoint or state!='IDLE' or any(abs(v)>p.tail_velocity_radps for v in actual.v+target.v)
                    or math.dist(tcp.p,tcp_anchor.p)>p.tail_tcp_drift_m
                    or any(abs(q-a)>p.tail_joint_drift_rad for q,a in zip(actual.q,anchor.q))):
                w._reject('pvat_tail')
            if captured_ns-self.arrival_ns>=2_000_000_000:self.tail_complete=True
            return
        quiet=(w.phase=='awaiting_arrival' and endpoint and state=='IDLE'
            and all(abs(v)<=.002 for v in actual.v+target.v))
        if not quiet:self.stable=None;return
        if self.stable is not None:
            since,anchor,tcp_anchor=self.stable
            if (math.dist(tcp.p,tcp_anchor.p)>p.stable_tcp_drift_m
                    or any(abs(q-a)>p.stable_joint_drift_rad for q,a in zip(actual.q,anchor.q))):self.stable=None
        if self.stable is None:self.stable=(captured_ns,actual,tcp.model_copy(deep=True))
        if captured_ns-self.stable[0]>=p.stable_span_ns:
            self.arrival_ns=captured_ns;self.arrival=(captured_ns,actual,tcp.model_copy(deep=True))
