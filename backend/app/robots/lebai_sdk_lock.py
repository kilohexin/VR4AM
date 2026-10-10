"""SDK mutual exclusion with priority for a pending stop RPC only.

Never interrupts a holder or an already granted waiter. Ordinary operations
remain FIFO; pending stops are FIFO among themselves and overtake only queued
ordinary operations. Used on one asyncio event loop, like asyncio.Lock.
"""

import asyncio
from collections import deque
from contextlib import asynccontextmanager
from dataclasses import dataclass


@dataclass(eq=False)
class _Waiter:
    future: asyncio.Future[None]
    stop: bool


class SdkLock:
    def __init__(self) -> None:
        self._locked = False
        self._waiters: deque[_Waiter] = deque()

    def locked(self) -> bool:
        return self._locked

    async def acquire(self, *, stop: bool = False) -> bool:
        if not self._locked and not self._waiters:
            self._locked = True
            return True
        waiter = _Waiter(asyncio.get_running_loop().create_future(), stop)
        self._waiters.append(waiter)
        try:
            await waiter.future
        except asyncio.CancelledError:
            if waiter in self._waiters:
                self._waiters.remove(waiter)
            elif not waiter.future.cancelled():
                # release() already reserved the lock for this waiter, but
                # cancellation won before acquire() returned to its caller.
                self._locked = False
            self._wake_next()
            raise
        return True

    def release(self) -> None:
        if not self._locked:
            raise RuntimeError("SDK lock is not acquired")
        self._locked = False
        self._wake_next()

    def _wake_next(self) -> None:
        if self._locked:
            return
        pending = [waiter for waiter in self._waiters if not waiter.future.done()]
        if not pending:
            return
        selected = next((waiter for waiter in pending if waiter.stop), pending[0])
        self._waiters.remove(selected)
        # Reserve before waking: a new acquire cannot steal this grant while
        # the selected task is runnable but has not resumed yet.
        self._locked = True
        selected.future.set_result(None)

    async def __aenter__(self) -> None:
        await self.acquire()

    async def __aexit__(self, *_exc) -> None:
        self.release()

    @asynccontextmanager
    async def stop_priority(self):
        await self.acquire(stop=True)
        try:
            yield
        finally:
            self.release()
