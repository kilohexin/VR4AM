"""Unregistered, injected real-arm trial port. No SDK loader or connection.

The caller must supply a fresh, scope-bound authorization callback. Production
web/VR selectors cannot construct this adapter. Two +Z segments are the entire
trial; reversal is deliberately a separate future admission.
"""
import math
from pathlib import Path
from app.robots.finite_position_web import RealFinitePositionAdapter
from app.robots.blended_position_docker import DockerBlendPort
from app.robots.lebai_codec import pose_from_lebai
from app.robots.finite_position_plan import angle


class TrialBudget:
    def __init__(self,authorize,clock):
        self.authorize,self.clock=authorize,clock
        self.started=clock();self.entries=0;self.unknown=False

    def check(self):
        self.authorize()
        if self.unknown:raise ValueError('blend_trial_unknown')
        if not 0<=self.clock()-self.started<20_000_000_000:raise ValueError('blend_trial_expired')

    def enter(self):
        self.check()
        if self.entries>=2:raise ValueError('blend_trial_consumed')
        self.entries+=1


class RealBlendTrialAdapter(RealFinitePositionAdapter):
    """Only explicit injected construction, retaining physical stop deadlines."""
    _stop_blocking_motion_methods=RealFinitePositionAdapter._stop_blocking_motion_methods|{'movel'}
    _immediate_stop_latch_methods=RealFinitePositionAdapter._immediate_stop_latch_methods|{'movel'}

    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._position_sources.add(Path(__file__))
        self._position_identity=self._source_identity();self._position_metadata=self._source_metadata()


class RealBlendTrialPort(DockerBlendPort):
    policy='real_blended_trial'

    def __init__(self,adapter,binding,*,authorize):
        if type(adapter) is not RealBlendTrialAdapter or binding is not adapter.position_binding:
            raise ValueError('blend_trial_adapter_required')
        self.adapter,self.binding,self.authorize=adapter,binding,authorize
        self.client=adapter._client;self.generation=adapter._pump.generation
        self.ids=set();self.sample=None;self.orientation=None;self.plan=None
        self.model_sha=adapter.position_model.model_sha256;self.closed=False
        self.budget=TrialBudget(authorize,adapter._clock);self.origin=None
        self.observations=[]

    async def read(self,pending):
        feedback=await super().read(pending)
        self.observations.append(dict(captured_ns=feedback.captured_ns,
            position=feedback.position,idle=feedback.idle,stationary=feedback.stationary,
            finished=feedback.finished,actual_q=self.sample.actual_q,
            actual_qd=self.sample.actual_qd,target_qd=self.sample.target_qd,
            actual_qdd=self.sample.actual_qdd,target_qdd=self.sample.target_qdd))
        return feedback

    def begin_cycle(self):self.check()
    def end_cycle(self):pass

    def check(self):
        self.budget.check()
        super().check()

    def _initialize_reference(self,s):self.origin=tuple(s.actual_tcp.p)

    def _target(self,target):
        if self.origin is None:raise ValueError('blend_trial_start_missing')
        delta=tuple(p-a for p,a in zip(target.position,self.origin))
        if (target.direction!=(0.,0.,1.) or abs(delta[0])>1e-9 or abs(delta[1])>1e-9
                or not 0<delta[2]<=.008+1e-9):raise ValueError('blend_trial_target_rejected')

    async def audit(self,target):
        self._target(target)
        plan=await super().audit(target)
        # Actual controller FK must independently agree at every sampled joint
        # configuration; the theoretical MDH model alone never admits hardware.
        for q in plan.joints:
            native=await self.adapter._position_math('kinematics_forward',
                lambda:self.client.kinematics_forward(list(q)),self.check)
            pose=pose_from_lebai(native);p,r=self.adapter.position_model.forward(q)
            if math.dist(p,pose.p)>.001 or angle(r,pose.q)>.001:
                self.plan=None;raise ValueError('blend_trial_native_fk_mismatch')
        return plan

    async def submit(self,plan,*,entering):
        self._target(plan.target)
        def entry():
            # Budget and external source/config authorization are checked at
            # native entry, after all IK/FK and fresh-feedback work.
            self.budget.check()
            if self.budget.entries>=2:raise ValueError('blend_trial_consumed')
            entering();self.budget.enter()
        try:return await super().submit(plan,entering=entry)
        except BaseException:
            if self.client.move_unknown or self.adapter._motion_calls.unknown_methods:self.budget.unknown=True
            raise
