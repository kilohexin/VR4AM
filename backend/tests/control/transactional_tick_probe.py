"""TEST ONLY: original RobotControl.tick, with transactional motion dispatch.

Uses the production TargetStrategy seam. Preparation copies candidate history;
sampling records its actual pose before dispatch. None commits no optimistic
target; completion collection advances history separately. No production module
imports this policy and no websocket session is replaced.
"""

from app.control.robot_control import RobotControl
from app.control.safety import SafetyViolation
from app.control.target_strategy import TargetPlan, TargetUpdate
from tests.control.braking_approach_probe import InfeasibleApproach
from tests.control.candidate_commit_probe import CandidateCoordinator
from tests.control.catch_up_probe import CatchUpProbe
from tests.robots.test_vr_profile_experiments import limiter


class TransactionalStrategy:
    def __init__(self, owner):
        self.owner = owner

    def prepare(self, previous, requested, dt, *, limiter):
        owner = self.owner
        owner.latest_requested = requested.model_copy(deep=True)
        if owner.pursuit is not None:
            if owner.pursuit.pending is not None or owner.pursuit._check() != 'pursuing':
                return TargetPlan('wait', None)
            return TargetPlan('catch_up', owner.adapter._last_sent_tcp.model_copy(deep=True))
        if owner.coordinator._pending is not None:
            return TargetPlan('wait', None)
        owner.candidate_inputs.append(requested.model_copy(deep=True))
        try:
            draft = owner.coordinator.prepare(requested, dt)
        except InfeasibleApproach as error:
            raise SafetyViolation('target_braking_infeasible') from error
        return TargetPlan('candidate', draft.pose, draft)

    async def dispatch(self, plan, command_id, *, backend):
        await self.owner.dispatch(plan, command_id)
        return None

    def collect(self, context):
        return self.owner.collect(context)

    def on_anchor(self, pose, context):
        self.owner.capture_epoch(pose)

    def on_revoke(self, generation):
        self.owner.latest_requested = None
        if self.owner.pursuit is not None:
            self.owner.pursuit.state = 'authority_revoked'


class TransactionalControl(RobotControl):
    def __init__(self, *, backend, **kwargs):
        self.adapter = backend
        self.coordinator = None
        self.pursuit = None
        self.latest_requested = None
        self.requests = []
        self.candidate_inputs = []
        self.exits = []
        self.halted = False
        strategy = TransactionalStrategy(self)
        super().__init__(backend=backend, limiter=limiter(),
            target_strategy=strategy, target_lifecycle=strategy, **kwargs)

    def input_authorized(self):
        return self._target_context().authorized

    def pump_current(self, generation):
        return self.adapter._pump.is_current(generation) and self.adapter.pump_fault is None

    def capture_epoch(self, pose):
        if self.coordinator is None:
            self.coordinator = CandidateCoordinator(self.adapter, self.limiter,
                pose,
                authority=lambda: (self.control_generation, self.input_authorized()),
                generation_current=self.pump_current)
        else:
            self.coordinator.settle()
            self.coordinator.reanchor(pose)
        self.pursuit = None
        self.halted = False
        self.latest_requested = None

    def collect(self, context):
        idle = TargetUpdate(context.generation)
        if self.coordinator is None or self.halted:
            return idle
        settled = self.coordinator.settle()
        committed = self.coordinator._pose.model_copy(deep=True) if settled == 'committed' else None
        if (self.adapter.pump_fault is not None and context.active
                and self._fault is None):
            return TargetUpdate(context.generation, fault=str(self.adapter.pump_fault))
        if not context.authorized:
            # Let the unchanged tick handle Grip/session/stale input transitions.
            return idle
        if self.coordinator.frozen_reason and self.pursuit is None:
            if self.coordinator.frozen_reason != 'catch_up':
                return TargetUpdate(context.generation, fault='candidate_dispatch_failed')
            receipt = self.requests[-1][1]
            done = receipt.completion.result
            accepted = self.adapter._accepted_target_ref
            if (done is None or done.accepted_command != accepted or accepted is None
                    or not self.pump_current(accepted.generation)):
                return TargetUpdate(context.generation, fault='candidate_dispatch_failed')
            epoch = self.control_generation
            self.pursuit = CatchUpProbe(self.adapter, accepted,
                self.adapter._last_accepted_solution_q, clock=self.clock.now_ns,
                authorized=lambda: self.input_authorized() and self.control_generation == epoch,
                generation_current=self.pump_current)
        if self.pursuit is not None:
            state = self.pursuit.settle()
            snapshot = self.adapter._snapshot
            if (state == 'pursuing' and snapshot is not None
                    and snapshot.captured_ns > self.pursuit.last_sample_ns):
                state = self.pursuit.observe(snapshot)
            if state in {'observed_settled', 'budget_expired'}:
                self.exits.append(state)
                self.halted = True
                return TargetUpdate(context.generation, stop=True)
            if state in {'dispatch_failed', 'admission_failed', 'authority_revoked'}:
                return TargetUpdate(context.generation, fault='candidate_dispatch_failed')
        return TargetUpdate(context.generation, committed=committed)

    async def dispatch(self, plan, command_id):
        if not self.input_authorized() or self.halted:
            return
        if self.pursuit is not None:
            # Gripper/recording awaits can consume the budget after collect().
            # Let the owner complete its frame bookkeeping before stopping.
            if self.pursuit._check() != 'pursuing':
                return
            if self.pursuit.pending is not None:
                return
            receipt = await self.pursuit.submit(command_id)
            self.requests.append(('catch_up', receipt))
        else:
            if self.coordinator._pending is not None:
                return
            receipt = await self.coordinator.submit_prepared(plan.payload, command_id)
            self.requests.append(('candidate', receipt))
