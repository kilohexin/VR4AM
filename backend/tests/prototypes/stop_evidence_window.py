"""Offline-only evidence window. Never sends commands or selects stop_sys policy."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Sample:
    ms: int
    q: tuple[float, ...]
    qd: tuple[float, ...]
    tcp: tuple[float, ...]
    state: str
    fault: bool


class EvidenceWindow:
    def __init__(self, *, stable_ms, max_gap_ms, joint_drift, tcp_drift, speed):
        limits = (stable_ms, max_gap_ms, joint_drift, tcp_drift, speed)
        if any(not math.isfinite(v) or v <= 0 for v in limits):
            raise ValueError("invalid evidence limits")
        self.stable_ms = stable_ms
        self.max_gap_ms = max_gap_ms
        self.joint_drift = joint_drift
        self.tcp_drift = tcp_drift
        self.speed = speed
        self.anchor = None
        self.last_ms = None
        self.confirmed = False

    @property
    def motion_allowed(self):
        return False

    def fresh_confirmation(self, now_ms):
        if self.last_ms is None or now_ms < self.last_ms:
            return False
        if now_ms - self.last_ms > self.max_gap_ms:
            self.confirmed = False
            self.anchor = None
        return self.confirmed

    def observe(self, s):
        if type(s.ms) is not int or s.ms < 0 or (
            self.last_ms is not None and s.ms <= self.last_ms
        ):
            raise ValueError("non-monotonic sample")
        self.fresh_confirmation(s.ms)
        self.last_ms = s.ms
        valid = (len(s.q) == 6 and len(s.qd) == 6 and len(s.tcp) == 3
                 and all(math.isfinite(v) for v in (*s.q, *s.qd, *s.tcp))
                 and not s.fault and s.state in {"IDLE", "STOP", "PAUSED"}
                 and max(abs(v) for v in s.qd) <= self.speed)
        if self.anchor is not None and valid:
            valid = (max(abs(a-b) for a, b in zip(s.q, self.anchor.q)) <= self.joint_drift
                     and math.dist(s.tcp, self.anchor.tcp) <= self.tcp_drift)
        if not valid:
            self.anchor = None
            self.confirmed = False
            return False
        if self.anchor is None:
            self.anchor = s
        self.confirmed = s.ms - self.anchor.ms >= self.stable_ms
        return self.confirmed
