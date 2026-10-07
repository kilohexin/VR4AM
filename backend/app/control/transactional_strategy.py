"""Opt-in transaction policy using the owner's existing tick and stop flow.

One candidate/pursuit request at a time; no trace arrays or autonomous tasks.
Pursuit exit requests owner disarm and explicit reanchor, never old-velocity thaw.
This experimental runtime policy is not a commissioned physical control profile.
"""

from collections.abc import Callable
from dataclasses import dataclass

from app.control.braking_approach import InfeasibleApproach
from app.control.candidate_commit import CandidateCoordinator
from app.control.catch_up import CatchUpSupervisor
from app.control.safety import SafetyViolation
from app.control.target_strategy import TargetContext, TargetPlan, TargetUpdate
from app.control.transactional_settings import PursuitSettings
from app.robots.base import BackendCommandError
from app.robots.command_admission import CommandRef


@dataclass(frozen=True)
class PursuitPlan:
    epoch: int
    accepted: CommandRef


class TransactionalTargetStrategy:
    def __init__(self, backend, limits, settings: PursuitSettings, *,
                 context: Callable[[], TargetContext]):
        if not isinstance(settings, PursuitSettings):
            raise ValueError('invalid_pursuit:settings')
        if not all(callable(getattr(backend, name, None)) for name in (
                'submit_tcp', 'submit_catch_up', 'read_motion_view')):
            raise ValueError('transactional_backend_required')
        self.backend, self.limits, self.settings, self.context = backend, limits, settings, context
        self.coordinator = None
        self.pursuit = None
        self.latest_requested = None
        self.last_receipt = None
        self.last_exit = None
        self.halted = False

    def _authority(self):
        context = self.context()
        return context.generation, context.active and context.authorized

    def _current(self, generation):
        return self.backend.read_motion_view().is_current(generation)

    def on_anchor(self, pose, context):
        if not context.active or not context.authorized or self._authority() != (context.generation, True):
            raise RuntimeError('authority_unavailable')
        if self.coordinator is None:
            self.coordinator = CandidateCoordinator(self.backend, self.limits, pose,
                authority=self._authority, generation_current=self._current)
        else:
            self.coordinator.settle()
            self.coordinator.reanchor(pose)
        self.pursuit = None
        self.latest_requested = None
        self.last_receipt = None
        self.last_exit = None
        self.halted = False

    def on_revoke(self, generation):
        self.latest_requested = None
        if self.pursuit is not None:
            self.pursuit.state = 'authority_revoked'

    def prepare(self, previous, requested, dt, *, limiter):
        self.latest_requested = requested.model_copy(deep=True)
        if self.coordinator is None or self.halted:
            return TargetPlan('wait', None)
        if self.pursuit is not None:
            if self.pursuit.pending is not None or self.pursuit._check() != 'pursuing':
                return TargetPlan('wait', None)
            view = self.backend.read_motion_view()
            if (view.accepted is None or view.accepted.command != self.pursuit.accepted
                    or not view.is_current(view.accepted.command.generation)):
                raise SafetyViolation('candidate_dispatch_failed')
            return TargetPlan('catch_up', view.accepted.tcp,
                PursuitPlan(self.coordinator.epoch, view.accepted.command))
        if self.coordinator.pending:
            return TargetPlan('wait', None)
        try:
            draft = self.coordinator.prepare(requested, dt)
        except InfeasibleApproach as error:
            raise SafetyViolation('target_braking_infeasible') from error
        except (ValueError, RuntimeError) as error:
            raise BackendCommandError('candidate_dispatch_failed') from error
        return TargetPlan('candidate', draft.pose, draft)

    async def dispatch(self, plan, command_id, *, backend):
        if backend is not self.backend:
            raise BackendCommandError('candidate_backend_changed')
        if (self.coordinator is None or self.halted
                or self._authority() != (self.coordinator.epoch, True)):
            return None
        if plan.kind == 'wait':
            return None
        try:
            if self.pursuit is not None:
                # Recording/gripper awaits can consume the pursuit budget.
                if self.pursuit._check() != 'pursuing' or self.pursuit.pending is not None:
                    return None
                if (plan.kind != 'catch_up' or plan.payload != PursuitPlan(
                        self.coordinator.epoch, self.pursuit.accepted)):
                    raise RuntimeError('prepared_candidate_obsolete')
                self.last_receipt = await self.pursuit.submit(command_id)
            else:
                if self.coordinator.pending:
                    return None
                if plan.kind != 'candidate':
                    raise RuntimeError('prepared_candidate_obsolete')
                self.last_receipt = await self.coordinator.submit_prepared(plan.payload, command_id)
        except BackendCommandError:
            raise  # Preserve hard SDK/recording failures.
        except (ValueError, RuntimeError, AttributeError) as error:
            raise BackendCommandError('candidate_dispatch_failed') from error
        return None

    def collect(self, context):
        idle = TargetUpdate(context.generation)
        if self.coordinator is None or self.halted:
            return idle
        settled = self.coordinator.settle()
        committed = self.coordinator.pose if settled == 'committed' else None
        view = self.backend.read_motion_view()
        if view.pump_fault is not None and context.active:
            return TargetUpdate(context.generation, fault=view.pump_fault)
        if not context.active or not context.authorized:
            return idle
        if self.coordinator.frozen_reason and self.pursuit is None:
            if self.coordinator.frozen_reason != 'catch_up':
                return TargetUpdate(context.generation, fault='candidate_dispatch_failed')
            done = self.last_receipt.completion.result if self.last_receipt is not None else None
            accepted = view.accepted
            if (done is None or accepted is None or done.accepted_command != accepted.command
                    or not view.is_current(accepted.command.generation)):
                return TargetUpdate(context.generation, fault='candidate_dispatch_failed')
            epoch = context.generation
            self.pursuit = CatchUpSupervisor(self.backend, accepted.command, accepted.solution_q,
                clock=lambda: self.context().now_ns,
                authorized=lambda: self._authority() == (epoch, True),
                generation_current=self._current, settings=self.settings)
        if self.pursuit is not None:
            state = self.pursuit.settle()
            sample = view.sample
            if (state == 'pursuing' and sample is not None
                    and sample.generation == view.generation == self.pursuit.accepted.generation
                    and sample.captured_ns > self.pursuit.last_sample_ns):
                state = self.pursuit.observe(sample)
            if state in {'observed_settled', 'budget_expired'}:
                self.last_exit = state
                self.halted = True
                return TargetUpdate(context.generation, stop=True)
            if state in {'dispatch_failed', 'admission_failed', 'authority_revoked'}:
                return TargetUpdate(context.generation, fault='candidate_dispatch_failed')
        return TargetUpdate(context.generation, committed=committed)
