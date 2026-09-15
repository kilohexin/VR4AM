"""Offline asynchronous request-ownership experiment; not production policy."""

import asyncio


class StopRequestOwner:
    """Own one local coroutine across callers, never grant motion permission.

    This does not implement physical stop verification, system disable,
    reconnect, process shutdown, or SDK cancellation semantics. It must not
    be substituted for the production adapter's stop implementation.
    """

    def __init__(self, send, *, wait_budget_s=None, clock=None):
        self.send = send
        self.wait_budget_s = wait_budget_s
        self.clock = clock or (lambda: asyncio.get_running_loop().time())
        self.deadline = None
        self.task = None
        self.rpc_state = 'NOT_SENT'
        self.fault_latched = False
        self.error = None
        self.reasons = []

    @property
    def motion_allowed(self):
        return False

    @property
    def stop_confirmed(self):
        return False  # RPC return is not physical stop evidence.

    async def _run(self):
        try:
            await self.send()
        except asyncio.CancelledError:
            self.rpc_state = 'UNKNOWN'
            self.fault_latched = True
            raise
        except Exception as exc:
            self.rpc_state = 'FAILED'
            self.error = f'{type(exc).__name__}: {exc}'
            self.fault_latched = True
        else:
            past_deadline = self.deadline is not None and self.clock() >= self.deadline
            if past_deadline:
                self.fault_latched = True
            self.rpc_state = ('RETURNED_LATE' if self.rpc_state == 'UNKNOWN' or past_deadline
                              else 'RETURNED')

    async def wait(self, reason, timeout):
        self.reasons.append(reason)
        if self.task is None:
            self.rpc_state = 'SENT'
            if self.wait_budget_s is not None:
                self.deadline = self.clock() + self.wait_budget_s
            self.task = asyncio.create_task(self._run())
        if self.task.done():
            # Consume cancellation/exception as well as the recorded result.
            await asyncio.shield(self.task)
            return self.rpc_state
        if self.deadline is not None:
            remaining = max(0.0, self.deadline - self.clock())
            if remaining == 0:
                self.fault_latched = True
                if self.rpc_state == 'SENT':
                    self.rpc_state = 'UNKNOWN'
                return self.rpc_state
            timeout = remaining if timeout is None else min(timeout, remaining)
        try:
            await asyncio.wait_for(asyncio.shield(self.task), timeout)
        except TimeoutError:
            self.fault_latched = True
            if self.rpc_state == 'SENT':
                self.rpc_state = 'UNKNOWN'
        except asyncio.CancelledError:
            self.fault_latched = True
            if self.rpc_state == 'SENT':
                self.rpc_state = 'UNKNOWN'
            raise
        return self.rpc_state
