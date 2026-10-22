"""Explicit Fake-injected finite movej candidate. No native connection factory."""
import asyncio
import hashlib
import json
import math
from dataclasses import asdict
from pathlib import Path
from app.control.finite_position_types import *
from app.robots.base import StopReason,BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.lebai_codec import pose_to_lebai,pose_from_lebai,joint_vector
from app.robots.finite_position_plan import build_position_target,audit_position_path,evaluate_position_arrival,FRACTIONS,angle
from app.robots.finite_position_acceleration import PositionAccelerationMonitor
from app.schemas.messages import Pose,BackendState


def digest(value):return hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()


class _FinitePositionCore(RealLebaiAdapter):
    """Shared executor. Requires an explicit injected factory; no native loader."""
    position_candidate=False
    _stop_blocking_motion_methods=frozenset({'movej','move_pvat','set_claw'})
    _immediate_stop_latch_methods=frozenset({'movej','move_pvat'})

    def __init__(self,settings,*,client_factory=None,backend_label=None,position_model=None,arrival_policy='strict',**kwargs):
        if client_factory is None or position_model is None:
            raise ValueError('position_explicit_injection_required')
        self.position_model=position_model;self.position_binding=None
        self.position_arrival_policy=position_arrival_policy(arrival_policy)
        self._position_policy_binding=self.position_arrival_policy
        self._position_plan=None;self._position_plan_observer=None
        self.position_busy=False;self.position_aborted=False
        self._position_acceleration=PositionAccelerationMonitor(min(.2,settings.control.max_joint_acceleration_radps2))
        self._position_config=digest(asdict(settings))
        self._position_sources={Path(__file__),Path(__file__).with_name('finite_position_plan.py'),
            Path(__file__).with_name('finite_position_acceleration.py'),
            Path(__file__).parents[1]/'control/finite_position_types.py'}
        self._position_identity=self._source_identity()
        self._position_metadata=self._source_metadata()
        super().__init__(settings,client_factory=client_factory,backend_label=backend_label,**kwargs)

    def _source_identity(self):
        return tuple((str(p),p.stat().st_size,p.stat().st_mtime_ns,hashlib.sha256(p.read_bytes()).hexdigest())
            for p in sorted(self._position_sources))

    def _source_metadata(self):
        return tuple((str(p),s.st_size,s.st_mtime_ns,s.st_ino) for p in sorted(self._position_sources) for s in (p.stat(),))

    def bind_position_generation(self,generation):
        if self.position_busy or self._latched_fault or self._motion_calls.unknown_methods:
            raise ValueError('position_binding_unavailable')
        self.position_binding=PositionBinding(self._client,generation,self._position_config,
            digest(asdict(self.settings.expected_tcp)))
        self.position_aborted=False
        self._position_acceleration.reset()
        return self.position_binding

    def _position_health(self,binding):
        if (self.position_arrival_policy is not self._position_policy_binding
                or binding is not self.position_binding or binding.connection is not self._client
                or self._client is None or self._disconnecting or self._latched_fault
                or self.position_aborted or self._snapshot_calls.closed
                or self._motion_calls.unknown_methods or self._snapshot_calls.unknown_methods
                or digest(asdict(self.settings))!=self._position_config
                or self._source_metadata()!=self._position_metadata):
            raise ValueError('position_binding_unavailable')
        self._check_known_snapshot_fault(self._client,self._pump.generation)
        if self._snapshot is not None:
            fault=self._motion_boundary_fault(self._snapshot)
            if fault:raise ValueError('position_'+fault)

    def _position_feedback_limits(self,s):
        fault=self._motion_boundary_fault(s)
        if fault:raise ValueError('position_'+fault)
        if not self._joints_inside_limits(s.target_q):raise ValueError('position_joint_outside_soft_limits')
        limits=self.settings.control
        if (any(abs(x)>limits.max_joint_speed_radps for v in (s.actual_qd,s.target_qd) for x in v)
                or any(abs(x)>self._position_acceleration.limit for x in s.target_qdd)):
            raise ValueError('position_reported_rate_limit')
        self._position_acceleration.observe(s.actual_qdd,captured_ns=s.captured_ns,now_ns=self._clock())

    def _position_entry_snapshot(self,plan):
        latest=self._snapshot
        if latest is None:raise ValueError('position_feedback_missing')
        self._position_feedback_limits(latest)
        if (self._position_plan_observer is None
                or not self._position_same_inputs(self._position_plan_observer,latest)
                or self.position_model.model_sha256!=plan.evidence.model_sha256
                or tuple(latest.actual_q)!=plan.start.actual_q or tuple(latest.target_q)!=plan.start.target_q
                or tuple(latest.actual_tcp.p)!=plan.start.tcp_p or tuple(latest.actual_tcp.q)!=plan.start.tcp_q
                or latest.robot_state!=BackendState.IDLE
                or any(abs(x)>1e-8 for v in (latest.actual_qd,latest.actual_qdd,latest.target_qd,latest.target_qdd) for x in v)):
            raise ValueError('position_observer_changed')

    @staticmethod
    def _position_same_inputs(before,after):
        # Timing may change; every controller input to this fixed plan must not.
        names=('robot_state','estop','actual_q','actual_qd','actual_qdd','target_q',
            'target_qd','target_qdd','actual_tcp','target_tcp','actual_flange',
            'tcp_setting','running_motion','raw_running_motion','motion_state','gripper')
        return all(getattr(before,name)==getattr(after,name) for name in names)

    def _position_sample(self,s,task_state=None):
        return PositionSample(self.position_binding,self._snapshot_read_sequence,s.captured_ns,
            s.actual_q,s.target_q,s.actual_qd,s.actual_qdd,s.target_qd,s.target_qdd,
            tuple(s.actual_tcp.p),tuple(s.actual_tcp.q),s.robot_state,s.running_motion,
            task_state if task_state is not None else s.motion_state,s.estop)

    async def _position_math(self,name,operation,authorize):
        async with self._sdk_lock:
            authorize();self._position_health(self.position_binding)
            return await self._snapshot_calls.call(name,operation,.2,revoked=lambda:self.position_aborted)

    async def plan_step(self,intent,*,authorize):
        binding=self.position_binding;self._position_health(binding);authorize()
        if self.position_busy:raise ValueError('position_busy')
        self._position_plan=None;self._position_plan_observer=None
        began=self._clock();model_identity=self.position_model.model_sha256
        def live():
            authorize();self._position_health(binding)
            if self.position_model.model_sha256!=model_identity:raise ValueError('position_model_changed')
            if not 0<=self._clock()-began<PLAN_NS:raise ValueError('position_planning_expired')
        try:
            async with asyncio.timeout(PLAN_NS/1e9):
                s=await self._read_snapshot(read_purpose='position_plan');live()
                if not self._preflight_from_snapshot(s).ready:raise ValueError('position_preflight')
                if not 0<=self._clock()-s.captured_ns<INPUT_NS:raise ValueError('position_snapshot_expired')
                if any(abs(x)>1e-8 for v in (s.actual_qd,s.actual_qdd,s.target_qd,s.target_qdd) for x in v):
                    raise ValueError('position_initial_not_stable')
                start=self._position_sample(s);p,q=build_position_target(start,intent)
                raw=await self._position_math('kinematics_inverse',lambda:self._client.kinematics_inverse(
                    pose_to_lebai(Pose(p=p,q=q)),list(s.actual_q)),live)
                if raw is None:raise ValueError('position_ik_unreachable')
                goal=joint_vector(raw,'position_ik_invalid');fk=[]
                for fraction in FRACTIONS:
                    joints=[a+(b-a)*fraction for a,b in zip(start.actual_q,goal)]
                    raw=await self._position_math('kinematics_forward',lambda:self._client.kinematics_forward(joints),live)
                    pose=pose_from_lebai(raw);fk.append((tuple(pose.p),tuple(pose.q)))
                if math.dist(fk[-1][0],p)>1e-5 or angle(fk[-1][1],q)>1e-4:
                    raise ValueError('position_ik_endpoint_mismatch')
                audit=await asyncio.to_thread(audit_position_path,start,goal,fk_samples=tuple(fk),
                    settings=self.settings,model=self.position_model)
                live()
                await self._emit_event(dict(kind='finite_position_plan',input_seq=intent.seq,
                    start_tcp=list(start.tcp_p),target_tcp=list(p),goal_joints=list(goal),audit=asdict(audit),
                    planning_timeout_ms=PLAN_NS/1e6,entry_policy='fresh_identical_start_after_planning'))
                live()
                fresh=await self._read_snapshot(read_purpose='position_revalidate');live()
                if not self._position_same_inputs(s,fresh):raise ValueError('position_observer_changed')
                if not self._preflight_from_snapshot(fresh).ready:raise ValueError('position_preflight')
                self._position_feedback_limits(fresh)
                if not 0<=self._clock()-fresh.captured_ns<INPUT_NS:raise ValueError('position_snapshot_expired')
                # The renewed timestamp is backed by a new read, never assigned to old feedback.
                self._position_plan_observer=fresh
                self._position_plan=PositionPlan(binding,intent.seq,self._position_sample(fresh),p,q,goal,
                    audit.joint_speed,audit.joint_acceleration,self._clock(),audit,self.position_arrival_policy)
                return self._position_plan
        except TimeoutError:
            raise ValueError('position_planning_expired') from None

    async def execute_step(self,plan,*,before_entry,monitor_authorize,on_entered):
        entered=False;owned=False;entry_ns=None;motion_id=None;final=None;reason=None;stable=None;anchor=None;good=0;last=None
        binding=plan.binding
        try:
            if (plan is not self._position_plan or self.position_busy
                    or plan.arrival_policy is not self.position_arrival_policy):raise ValueError('position_plan_obsolete')
            self._position_plan=None;self._position_health(binding);before_entry()
            self.position_busy=True;owned=True
            async with self._sdk_lock:
                async def send():
                    nonlocal entered,entry_ns
                    self._position_health(binding);before_entry()
                    self._position_entry_snapshot(plan)
                    if not 0<=self._clock()-plan.start.captured_ns<INPUT_NS:raise ValueError('position_snapshot_expired')
                    on_entered();self._begin_motion_epoch();entered=True;entry_ns=self._clock()
                    self._motion_accepted=self._motion_write_stop_required=True
                    return await self._client.movej(list(plan.goal_joints),plan.joint_acceleration,plan.joint_speed,0.,0.)
                motion_id=await self._motion_calls.call('movej',send,.2,
                    revoked=lambda:self.position_aborted,entered=lambda:entered and self._position_native_entered(),
                    native_result_unknown=self._position_native_unknown)
            if type(motion_id) is not int or motion_id<=0:raise ValueError('position_invalid_motion_id')
            policy=plan.arrival_policy;end=entry_ns+policy.completion_ns
            while self._clock()<end:
                monitor_authorize();self._position_health(binding)
                s=await self._read_snapshot(read_purpose='position_progress')
                monitor_authorize();self._position_health(binding)
                state=await self._position_math('get_motion_state',lambda:self._client.get_motion_state(motion_id),monitor_authorize)
                if self._clock()>=end:raise ValueError('position_completion_timeout')
                final=self._position_sample(s,str(state).upper())
                if not 0<=self._clock()-final.captured_ns<INPUT_NS:raise ValueError('position_feedback_expired')
                self._position_feedback_limits(s)
                if s.running_motion not in (None,motion_id):raise ValueError('position_foreign_task')
                if str(state).upper() not in ('RUNNING','FINISHED'):raise ValueError('position_task_failed')
                check=evaluate_position_arrival(plan,final,motion_id=motion_id)
                if check is None:
                    if (last is None or final.captured_ns-last>=INPUT_NS or anchor is None
                            or math.dist(final.tcp_p,anchor.tcp_p)>policy.stable_tcp_drift_m
                            or max(abs(a-b) for a,b in zip(final.actual_q,anchor.actual_q))>policy.stable_joint_drift_rad):
                        stable=final.captured_ns;good=0;anchor=final
                    if stable is None:stable=final.captured_ns;anchor=final
                    good+=1
                    if good>=3 and final.captured_ns-stable>=policy.stable_span_ns:
                        self._motion_accepted=self._motion_write_stop_required=False
                        return PositionResult(True,motion_id,True,False,final,None)
                else:
                    stable=None;good=0;reason=check
                    settling=check in ('position_endpoint_error','position_joint_endpoint','position_not_stable')
                    if check=='position_endpoint_error':
                        delta=tuple(b-a for a,b in zip(plan.start.tcp_p,plan.target_p))
                        length=math.hypot(*delta)
                        progress=sum((p-a)*d for p,a,d in zip(final.tcp_p,plan.start.tcp_p,delta))/length
                        settling=progress<=length+1e-12  # Trailing the goal, never excess overshoot.
                    # FINISHED describes the controller task, not physical
                    # arrival. Give known trailing/settling feedback the rest
                    # of the fixed policy window; all boundaries still run each
                    # iteration and no additional target is submitted.
                    if final.task_state=='FINISHED' and check!='position_not_finished' and not settling:
                        raise ValueError(check)
                last=final.captured_ns
                await self._sleep(.02)
            raise ValueError(reason or 'position_completion_timeout')
        except BaseException as error:
            if entered and not self._position_native_entered():
                entered=False;self._motion_accepted=self._motion_write_stop_required=False
            reason=str(error) if str(error).startswith('position_') else 'position_step_failed'
            unknown=bool(self._motion_calls.unknown_methods)
            if entered and binding.connection is self._client:
                try:await asyncio.shield(self.stop(StopReason.FAULT))
                except BaseException:pass
                if unknown:self._latched_fault='stop_unverified'
            if isinstance(error,asyncio.CancelledError):raise
            return PositionResult(False,motion_id if type(motion_id) is int else None,entered,unknown,final,
                self._latched_fault or reason)
        finally:
            if owned:
                self.position_busy=False
                self._schedule_sdk_diagnostic(dict(kind='finite_position_step',input_seq=plan.intent_seq,
                    entered=entered,entry_ns=entry_ns,motion_id=motion_id if type(motion_id) is int else None,
                    reason=reason,unknown=bool(self._motion_calls.unknown_methods)))

    async def prepare(self,*args,**kwargs):
        if self.position_busy:raise BackendCommandError('position_busy')
        return await super().prepare(*args,**kwargs)

    async def stop(self,reason):
        self.position_aborted=True
        return await super().stop(reason)

    async def home(self,*args,**kwargs):
        if self.position_busy:raise BackendCommandError('position_busy')
        return await super().home(*args,**kwargs)

    async def set_gripper(self,*args,**kwargs):
        if self.position_busy:raise BackendCommandError('position_busy')
        return await super().set_gripper(*args,**kwargs)

    def _position_native_entered(self):return True

    def _position_native_unknown(self):return None


class FinitePositionAdapter(_FinitePositionCore):
    """Public demo selector remains Fake-only; no production registration."""
    position_candidate=True

    def __init__(self,settings,*,client_factory=None,backend_label=None,position_model=None,**kwargs):
        if (settings.ip!='digital-twin' or backend_label!='LEBAI_FAKE'
                or client_factory is None or position_model is None):
            raise ValueError('position_offline_injection_required')
        super().__init__(settings,client_factory=client_factory,backend_label=backend_label,
            position_model=position_model,**kwargs)
