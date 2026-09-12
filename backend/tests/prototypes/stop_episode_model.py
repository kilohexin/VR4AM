"""Offline design model. No SDK, network, timers, or production integration."""


class StopEpisode:
    """One simulated episode; proposals are NOT executable SDK commands.

    A caller supplies monotonic time and independent stop evidence. The 200/700
    ms deadlines are illustrative test budgets, not a new production policy.
    REVIEW_ESCALATION deliberately leaves the physical response undecided.
    """

    def __init__(self):
        self.rpc_state = 'NOT_SENT'
        self.request_count = 0
        self.fault_latched = False
        self.stop_confirmed = False
        self.started_ms = None
        self.last_ms = -1
        self.reasons = []
        self.audit = []

    @property
    def motion_allowed(self):
        return False

    def _advance(self, now_ms):
        if type(now_ms) is not int or now_ms < 0 or now_ms < self.last_ms:
            raise ValueError('invalid_monotonic_time')
        self.last_ms = now_ms
        if self.started_ms is None:
            return
        age = now_ms - self.started_ms
        if self.rpc_state == 'SENT' and age >= 200:
            self.rpc_state = 'UNKNOWN'
            self.fault_latched = True
            self.audit.append((now_ms, 'RPC_RESULT_UNKNOWN'))
        if age >= 700 and not self.stop_confirmed:
            self.fault_latched = True

    def request_stop(self, reason, now_ms):
        self._advance(now_ms)
        self.reasons.append(reason)
        if self.started_ms is not None:
            self.audit.append((now_ms, 'REUSE_EPISODE', reason))
            return 'REUSE_EPISODE'
        self.started_ms = now_ms
        self.rpc_state = 'SENT'
        self.request_count = 1
        self.audit.append((now_ms, 'PROPOSE_STOP_MOVE', 1))
        return 'PROPOSE_STOP_MOVE'

    def tick(self, now_ms):
        self._advance(now_ms)
        return 'REVIEW_ESCALATION' if self.fault_latched else 'NO_ACTION'

    def reply(self, request_id, now_ms):
        self._advance(now_ms)
        if self.started_ms is None or request_id != 1:
            self.audit.append((now_ms, 'UNMATCHED_REPLY', request_id))
            return 'UNMATCHED_REPLY'
        if self.rpc_state in {'RETURNED', 'RETURNED_LATE'}:
            return 'DUPLICATE_REPLY'
        self.rpc_state = 'RETURNED_LATE' if self.rpc_state == 'UNKNOWN' else 'RETURNED'
        self.audit.append((now_ms, self.rpc_state, request_id))
        return 'REPLY_RECORDED'

    def interrupt(self, now_ms, cause):
        if cause not in {'DISCONNECTED', 'CANCELLED'}:
            raise ValueError('unknown_interruption')
        self._advance(now_ms)
        if self.rpc_state == 'SENT':
            self.rpc_state = 'UNKNOWN'
        self.stop_confirmed = False
        self.fault_latched = True
        self.audit.append((now_ms, cause))
        return 'REVIEW_ESCALATION'

    def observe(self, now_ms, state, *, stable_window_complete, position_changed=False):
        self._advance(now_ms)
        # This boolean is an injected evidence verdict, not a qd=0 shortcut.
        confirmed = (self.started_ms is not None and state in {'IDLE', 'STOP', 'PAUSED'}
                     and stable_window_complete and not position_changed)
        if position_changed or (self.stop_confirmed and not confirmed):
            self.fault_latched = True
        self.stop_confirmed = bool(confirmed)
        self.audit.append((now_ms, 'OBSERVATION', state, self.stop_confirmed))
        return 'REVIEW_ESCALATION' if self.fault_latched else 'EVIDENCE_RECORDED'
