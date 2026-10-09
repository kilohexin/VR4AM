"""Optional bounded observer deferral; never gates motion or stop reads."""
import asyncio
import math


class ObserverWindow:
    def __init__(self, max_wait_s: float = 0.0):
        if (isinstance(max_wait_s, bool) or not math.isfinite(max_wait_s)
                or not 0 <= max_wait_s <= .020):
            raise ValueError('observer_wait_must_be_between_0_and_20ms')
        self.max_wait_s = max_wait_s
        self._leases: set[object] = set()
        self._idle = asyncio.Event()
        self._idle.set()

    @property
    def active(self) -> bool:
        return bool(self._leases)

    def begin(self) -> object | None:
        if self.max_wait_s == 0:
            return None
        lease = object()
        self._leases.add(lease)
        self._idle.clear()
        return lease

    def finish(self, lease: object | None) -> None:
        self._leases.discard(lease)
        if not self._leases:
            self._idle.set()

    def release_all(self) -> None:
        self._leases.clear()
        self._idle.set()
        # Keep the released event open for old waiters, even if a new motion
        # starts before their Event.wait coroutine has resumed.
        self._idle = asyncio.Event()
        self._idle.set()

    async def wait(self) -> None:
        if not self.active or self.max_wait_s == 0:
            return
        try:
            # One observer owns one wait budget, regardless of later motions.
            await asyncio.wait_for(self._idle.wait(), timeout=self.max_wait_s)
        except TimeoutError:
            pass
