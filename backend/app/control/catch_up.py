"""Bounded pursuit of an accepted target; observations do not certify physical stop."""

import math

from app.control.transactional_settings import PursuitSettings


class CatchUpSupervisor:
    def __init__(self, backend, accepted, solution, *, clock, authorized, generation_current, settings: PursuitSettings):
        self.settings = settings
        self.backend, self.accepted = backend, accepted
        self.solution = tuple(solution)
        self.clock, self.authorized = clock, authorized
        self.generation_current = generation_current
        self.started_ns = clock()
        self.state = 'pursuing'
        self.pending = None
        self.submitting = False
        self.good_samples = 0
        self.last_sample_ns = self.started_ns
        self.dispatched = False
        self.phase = 'dispatchable'
        self.feedback_after_ns = self.started_ns
        self.feedback_deadline_ns = None

    def _check(self):
        if not self.authorized() or not self.generation_current(self.accepted.generation):
            self.state = 'authority_revoked'
        elif self.state == 'pursuing' and self.clock() - self.started_ns >= self.settings.budget_ms * 1_000_000:
            self.state = 'budget_expired'
        elif (self.state == 'pursuing' and self.feedback_deadline_ns is not None
              and self.clock() > self.feedback_deadline_ns):
            self.state = 'feedback_expired'
        if self.state != 'pursuing':
            self.phase = 'terminal'
            if self.state != 'observed_settled':
                self.good_samples = 0
        return self.state

    @property
    def ready_to_submit(self):
        return (self._check() == 'pursuing' and self.phase == 'dispatchable'
                and self.pending is None and not self.submitting)

    async def submit(self, command_id):
        if self._check() != 'pursuing':
            raise RuntimeError(self.state)
        if self.pending is not None or self.submitting:
            raise RuntimeError('request_pending')
        if self.phase != 'dispatchable':
            raise RuntimeError('feedback_pending')
        self.submitting = True
        self.phase = 'inflight'
        self.good_samples = 0
        self.feedback_deadline_ns = None
        try:
            self.pending = await self.backend.submit_catch_up(self.accepted, command_id)
            self._check()
            return self.pending
        except BaseException:
            self.state = 'admission_failed'
            raise
        finally:
            self.submitting = False

    def settle(self):
        self._check()
        if self.pending is None:
            return self.state
        receipt = self.pending
        if receipt.completion.invalidated:
            self.state = 'authority_revoked'
        done = receipt.completion.result
        if done is None:
            return 'waiting' if self.state == 'pursuing' else self.state
        self.pending = None
        if self.state != 'pursuing':
            return self.state
        if (done.command != receipt.command or done.invalidated
                or receipt.command.generation != self.accepted.generation
                or done.outcome != 'sdk_returned' or done.sdk_outcome != 'returned'
                or done.accepted_command != self.accepted or done.pvat_mode != 'catch_up'
                or done.error is not None):
            self.state = 'dispatch_failed'
        else:
            self.dispatched = True
            # This is local result collection, not controller execution time.
            # A read begun before it cannot confirm this latest request.
            self.feedback_after_ns = self.clock()
            self.last_sample_ns = max(self.last_sample_ns, self.feedback_after_ns)
            self.feedback_deadline_ns = min(
                self.started_ns + self.settings.budget_ms * 1_000_000,
                self.feedback_after_ns + self.settings.max_observation_gap_ms * 1_000_000)
            self.phase = 'awaiting_feedback'
        return self.state

    def observe(self, snapshot):
        self._check()
        if self.state != 'pursuing':
            return self.state
        now = self.clock()
        captured = getattr(snapshot, 'captured_ns', None)
        completed = getattr(snapshot, 'read_completed_ns', None)
        generation = getattr(snapshot, 'generation', self.accepted.generation)
        source_valid = (type(captured) is int and type(completed) is int
                        and 0 <= captured <= completed <= now
                        and type(generation) is int and generation == self.accepted.generation
                        and getattr(snapshot, 'read_purpose', None) in {'state', 'motion'})
        try:
            values_valid = (len(snapshot.actual_q) == len(self.solution) == len(snapshot.actual_qd) == 6
                            and all(math.isfinite(v) for v in (*snapshot.actual_q, *snapshot.actual_qd)))
        except (TypeError, AttributeError, ValueError):
            values_valid = False
        fresh = (source_valid and self.dispatched and self.pending is None and not self.submitting
                 and values_valid
                 and captured > self.last_sample_ns and captured > self.feedback_after_ns
                 and now - captured <= self.settings.max_observation_age_ms * 1_000_000)
        close = (values_valid
                 and max(abs(a-b) for a, b in zip(snapshot.actual_q, self.solution)) <= self.settings.position_tolerance_rad
                 and max(abs(v) for v in snapshot.actual_qd) <= self.settings.velocity_tolerance_radps)
        consecutive = source_valid and captured - self.last_sample_ns <= self.settings.max_observation_gap_ms * 1_000_000
        self.good_samples = (self.good_samples if consecutive else 0) + 1 if fresh and close else 0
        if source_valid and captured <= now:
            self.last_sample_ns = max(self.last_sample_ns, captured)
        if fresh:
            if close:
                self.phase = 'awaiting_feedback'
                self.feedback_deadline_ns = min(
                    self.started_ns + self.settings.budget_ms * 1_000_000,
                    captured + self.settings.max_observation_gap_ms * 1_000_000)
            else:
                self.phase = 'dispatchable'
                self.feedback_deadline_ns = min(
                    self.started_ns + self.settings.budget_ms * 1_000_000,
                    captured + min(self.settings.max_observation_gap_ms,
                                   self.settings.max_observation_age_ms) * 1_000_000)
        if self.good_samples >= self.settings.settle_samples:
            self.state = 'observed_settled'
            self.phase = 'terminal'
        return self.state
