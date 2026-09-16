"""Offline comparison labels, never executable robot commands.

The research budget is caller-supplied and is not a production stop deadline.
No result from this model permits motion or proves a hardware safety response.
"""
import math

from tests.prototypes.stop_evidence_window import EvidenceWindow


class StopPolicyComparison:
    def __init__(self, *, reply_ms, research_budget_ms):
        if not (type(reply_ms) is int and type(research_budget_ms) is int
                and 0 < reply_ms < research_budget_ms):
            raise ValueError("invalid research budgets")
        self.reply_ms = reply_ms
        self.budget_ms = research_budget_ms
        self.window = EvidenceWindow(stable_ms=300, max_gap_ms=250,
                                     joint_drift=.001, tcp_drift=.0005, speed=.02)
        self.last_ms = -1
        self.anchor = None
        self.returned = False
        self.fault_latched = False
        self.review_latched = False
        self.budget_checked = False

    @property
    def motion_allowed(self):
        return False

    def _advance(self, ms):
        if type(ms) is not int or ms < 0 or ms < self.last_ms:
            raise ValueError("non-monotonic event")
        self.last_ms = ms
        if ms >= self.reply_ms and not self.returned:
            self.fault_latched = True
        if ms >= self.budget_ms and not self.budget_checked:
            # Evaluate expiry before accepting a late reply or a new sample.
            self.budget_checked = True
            if not self.returned or not self.window.fresh_confirmation(self.budget_ms):
                self.review_latched = True

    def reply(self, ms):
        self._advance(ms)
        self.returned = True

    def observe(self, s):
        self._advance(s.ms)
        if self.window.last_ms is not None and s.ms - self.window.last_ms > 250:
            self.review_latched = True
        valid = (len(s.q) == 6 and len(s.qd) == 6 and len(s.tcp) == 3
                 and all(math.isfinite(v) for v in (*s.q, *s.qd, *s.tcp))
                 and not s.fault and s.state in {"IDLE", "STOP", "PAUSED", "STOPPING"}
                 and max(abs(v) for v in s.qd) <= .02)
        if valid and self.anchor is not None:
            valid = (max(abs(a - b) for a, b in zip(s.q, self.anchor.q)) <= .001
                     and math.dist(s.tcp, self.anchor.tcp) <= .0005)
        if not valid:
            self.review_latched = True
        elif self.anchor is None:
            self.anchor = s
        self.window.observe(s)

    def assess(self, ms):
        self._advance(ms)
        fresh = self.window.last_ms is not None and ms - self.window.last_ms <= 250
        if not fresh or (ms >= self.budget_ms and not self.returned):
            self.review_latched = True
        if self.review_latched:
            self.fault_latched = True
            return "PROTECTION_REVIEW"
        if self.window.fresh_confirmation(ms):
            return "STATIONARY_REPLY_RETURNED" if self.returned else "STATIONARY_RPC_UNRESOLVED"
        if ms >= self.budget_ms:
            self.review_latched = True
            self.fault_latched = True
            return "PROTECTION_REVIEW"
        return "AWAIT_WITHIN_RESEARCH_BUDGET"
