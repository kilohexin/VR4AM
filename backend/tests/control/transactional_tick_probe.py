"""Test traces around the runtime strategy; no overridden control behavior."""

from app.control.robot_control import RobotControl
from app.control.transactional_strategy import TransactionalTargetStrategy
from tests.control.catch_up_probe import EXPERIMENT_PURSUIT
from tests.robots.test_vr_profile_experiments import limiter


class TransactionalStrategy(TransactionalTargetStrategy):
    def __init__(self, owner):
        self.owner = owner
        super().__init__(owner.adapter, owner.limiter, EXPERIMENT_PURSUIT,
            context=owner._target_context)

    def prepare(self, previous, requested, dt, *, limiter):
        plan = super().prepare(previous, requested, dt, limiter=limiter)
        if plan.kind == 'candidate':
            self.owner.candidate_inputs.append(requested.model_copy(deep=True))
        return plan

    async def dispatch(self, plan, command_id, *, backend):
        previous = self.last_receipt
        value = await super().dispatch(plan, command_id, backend=backend)
        if self.last_receipt is not previous:
            self.owner.requests.append((plan.kind, self.last_receipt))
        return value

    def collect(self, context):
        update = super().collect(context)
        if update.stop:
            self.owner.exits.append(self.last_exit)
        return update


class TransactionalControl(RobotControl):
    def __init__(self, *, backend, **kwargs):
        self.adapter = backend
        self.requests = []
        self.candidate_inputs = []
        self.exits = []
        super().__init__(backend=backend, limiter=limiter(), **kwargs)
        strategy = TransactionalStrategy(self)
        self.target_strategy = self.target_lifecycle = strategy

    @property
    def coordinator(self):
        return self.target_strategy.coordinator

    @property
    def pursuit(self):
        return self.target_strategy.pursuit

    @property
    def latest_requested(self):
        return self.target_strategy.latest_requested

    def collect(self, context):
        return self.target_strategy.collect(context)
