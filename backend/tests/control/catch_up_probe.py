"""TEST ONLY. Bounded pursuit observations, never physical stop certification.

The 500 ms budget, 40 ms observation age, 1 mrad position / 2 mrad/s speed
bands and three distinct samples are experiment inputs, not robot settings.
Every terminal state requires the owner to use its stop/reanchor flow. No
automatic thaw, velocity restoration, background retry or stop is performed.
"""

import math


class CatchUpProbe:
    def __init__(self, backend, accepted, solution, *, clock, authorized, generation_current):
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

    def _check(self):
        if not self.authorized() or not self.generation_current(self.accepted.generation):
            self.state = 'authority_revoked'
        elif self.state == 'pursuing' and self.clock() - self.started_ns >= 500_000_000:
            self.state = 'budget_expired'
        return self.state

    async def submit(self, command_id):
        if self._check() != 'pursuing':
            raise RuntimeError(self.state)
        if self.pending is not None or self.submitting:
            raise RuntimeError('request_pending')
        self.submitting = True
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
        return self.state

    def observe(self, snapshot):
        self._check()
        if self.state != 'pursuing':
            return self.state
        fresh = (self.dispatched and self.pending is None and not self.submitting
                 and snapshot.captured_ns > self.last_sample_ns
                 and 0 <= self.clock() - snapshot.captured_ns <= 40_000_000)
        values = (*snapshot.actual_q, *snapshot.actual_qd)
        close = (len(snapshot.actual_q) == len(self.solution) == len(snapshot.actual_qd) == 6
                 and all(math.isfinite(v) for v in values)
                 and max(abs(a-b) for a, b in zip(snapshot.actual_q, self.solution)) <= .001
                 and max(abs(v) for v in snapshot.actual_qd) <= .002)
        consecutive = snapshot.captured_ns - self.last_sample_ns <= 40_000_000
        self.good_samples = (self.good_samples if consecutive else 0) + 1 if fresh and close else 0
        self.last_sample_ns = max(self.last_sample_ns, snapshot.captured_ns)
        if self.good_samples >= 3:
            self.state = 'observed_settled'
        return self.state
