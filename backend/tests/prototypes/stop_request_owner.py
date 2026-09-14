"""Offline asynchronous request-ownership experiment; not production policy."""

import asyncio


class StopRequestOwner:
    """Own one local coroutine across callers, never grant motion permission.

    This does not implement physical stop verification, system disable,
    reconnect, process shutdown, or SDK cancellation semantics. It must not
    be substituted for the production adapter's stop implementation.
    """

    def __init__(self, send):
        self.send = send
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
            self.rpc_state = ('RETURNED_LATE' if self.rpc_state == 'UNKNOWN'
                              else 'RETURNED')

    async def wait(self, reason, timeout):
        self.reasons.append(reason)
        if self.task is None:
            self.rpc_state = 'SENT'
            self.task = asyncio.create_task(self._run())
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
