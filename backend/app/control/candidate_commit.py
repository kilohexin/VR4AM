"""Completion-correlated command history; reanchor only in a new owner epoch."""

import copy
from dataclasses import dataclass

from app.schemas.messages import Pose
from app.control.brake_retarget import BrakeBeforeRetarget


@dataclass(frozen=True)
class History:
    position: tuple
    orientation: tuple
    linear_velocity: tuple
    angular_velocity: tuple
    braking: bool
    version: int


@dataclass
class Draft:
    candidate: BrakeBeforeRetarget
    pose: Pose
    receipt: object
    version: int


@dataclass
class Prepared:
    owner: object
    candidate: BrakeBeforeRetarget
    pose: Pose
    version: int


class CandidateCoordinator:
    def __init__(self, backend, limits, anchor, *, authority, generation_current):
        self.backend = backend
        self.authority = authority
        self.generation_current = generation_current
        self.epoch, active = authority()
        if not active:
            raise RuntimeError('authority_unavailable')
        self.limits = copy.deepcopy(limits)
        self._candidate = BrakeBeforeRetarget(copy.deepcopy(limits))
        self._candidate.limits.set_pose_anchor(anchor)
        self._pose = anchor.model_copy(deep=True)
        self._version = 0
        self._generation = None
        self._committed_command = None
        self._pending = None
        self._submitting = False
        self.frozen_reason = None

    def snapshot(self):
        return History(tuple(self._pose.p), tuple(self._pose.q),
            tuple(self._candidate.linear_velocity), tuple(self._candidate.angular_velocity),
            self._candidate.braking, self._version)

    @property
    def pending(self):
        return self._pending is not None or self._submitting

    @property
    def pose(self):
        return self._pose.model_copy(deep=True)

    @property
    def committed_command(self):
        """Immutable command identity; not physical execution evidence."""
        return self._committed_command

    def _authorized(self):
        return self.authority() == (self.epoch, True)

    def _check_ready(self):
        if self._pending is not None or self._submitting:
            raise RuntimeError('request_pending')
        if self.frozen_reason:
            raise RuntimeError('reanchor_required')
        if not self._authorized() or (self._generation is not None
                                     and not self.generation_current(self._generation)):
            self.frozen_reason = 'authority_revoked'
            raise RuntimeError('authority_unavailable')

    def prepare(self, requested, dt):
        self._check_ready()
        trial = copy.deepcopy(self._candidate)
        try:
            pose = trial.limit_motion(self._pose, requested, dt)
        except Exception:
            self.frozen_reason = 'candidate_failed'
            raise
        return Prepared(self, trial, pose, self._version)

    async def propose(self, requested, command_id, dt):
        return await self.submit_prepared(self.prepare(requested, dt), command_id)

    async def submit_prepared(self, prepared, command_id):
        self._check_ready()
        if prepared.owner is not self or prepared.version != self._version:
            raise RuntimeError('prepared_candidate_obsolete')
        self._submitting = True
        try:
            receipt = await self.backend.submit_tcp(prepared.pose, command_id)
            if receipt.completion is None:
                raise RuntimeError('completion_missing')
            self._pending = Draft(prepared.candidate, prepared.pose, receipt, self._version)
            if self._generation is None:
                self._generation = receipt.command.generation
            if not self._authorized():
                self.frozen_reason = 'authority_revoked'
            return receipt
        except BaseException:
            self.frozen_reason = 'admission_failed'
            raise
        finally:
            self._submitting = False

    def settle(self, *, accepted=None, require_accepted=False):
        if self._pending is None:
            return 'idle'
        draft = self._pending
        receipt = draft.receipt
        valid = (self._authorized() and self.generation_current(receipt.command.generation)
                 and receipt.command.generation == self._generation
                 and not receipt.completion.invalidated)
        if not valid:
            self.frozen_reason = 'authority_revoked'
        result = receipt.completion.result
        if result is None:
            return 'waiting' if valid else 'waiting_invalidated'
        self._pending = None
        if (require_accepted and result.pvat_mode == 'advance'
                and (accepted is None or accepted.command != receipt.command
                     or accepted.tcp != draft.pose)):
            self.frozen_reason = self.frozen_reason or 'accepted_target_changed'
        if (valid and not self.frozen_reason and draft.version == self._version
                and result.command == receipt.command and not result.invalidated
                and result.outcome == 'sdk_returned' and result.sdk_outcome == 'returned'
                and result.accepted_command == receipt.command
                and result.pvat_mode == 'advance' and result.error is None):
            self._candidate, self._pose = draft.candidate, draft.pose
            self._committed_command = receipt.command
            self._version += 1
            return 'committed'
        self.frozen_reason = self.frozen_reason or (result.pvat_mode or result.outcome)
        return 'frozen'

    def reanchor(self, pose):
        if self._pending is not None or self._submitting:
            raise RuntimeError('request_pending')
        epoch, active = self.authority()
        if not active:
            raise RuntimeError('authority_unavailable')
        if epoch <= self.epoch:
            raise RuntimeError('new_epoch_required')
        candidate = BrakeBeforeRetarget(copy.deepcopy(self.limits))
        candidate.limits.set_pose_anchor(pose)
        self._candidate = candidate
        self._pose = pose.model_copy(deep=True)
        self._version += 1
        self.epoch = epoch
        self._generation = None
        self._committed_command = None
        self.frozen_reason = None
