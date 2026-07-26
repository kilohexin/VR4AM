from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass

from app.robots.base import BackendCommandError
from app.schemas.messages import Pose


@dataclass(frozen=True)
class PvatRequest:
    target: Pose
    command_id: int
    generation: int


class PvatPump:
    def __init__(
        self,
        handler: Callable[[PvatRequest], Awaitable[None]],
        *,
        period_s: float,
    ) -> None:
        if period_s <= 0:
            raise ValueError("period_s_must_be_positive")
        self._handler = handler
        self.period_s = period_s
        self._pending: PvatRequest | None = None
        self._generation = 0
        self._wake = asyncio.Event()
        self._task: asyncio.Task[None] | None = None
        self._running = False
        self._fault: BackendCommandError | None = None

    @property
    def fault(self) -> BackendCommandError | None:
        return self._fault

    @property
    def running(self) -> bool:
        return self._running

    @property
    def has_pending(self) -> bool:
        return self._pending is not None

    async def start(self) -> None:
        if self._running:
            return
        if self._fault is not None:
            raise self._fault
        self._running = True
        self._task = asyncio.create_task(
            self._run(),
            name="lebai-pvat-pump",
        )

    def submit(self, target: Pose, command_id: int) -> None:
        if not self._running:
            raise BackendCommandError("pvat_pump_not_running")
        if self._fault is not None:
            raise self._fault
        self._pending = PvatRequest(
            target=target.model_copy(deep=True),
            command_id=command_id,
            generation=self._generation,
        )
        self._wake.set()

    def invalidate(self) -> int:
        self._generation += 1
        self._pending = None
        self._wake.clear()
        return self._generation

    def is_current(self, generation: int) -> bool:
        return self._running and generation == self._generation

    async def stop(self) -> None:
        task, self._task = self._task, None
        self._running = False
        self.invalidate()
        self._wake.set()
        if task is None or task is asyncio.current_task():
            return
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass
        finally:
            self._wake.clear()

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        next_deadline = loop.time()
        try:
            while self._running:
                request = await self._take_latest()
                if request is None:
                    return
                if self.is_current(request.generation):
                    try:
                        await self._handler(request)
                    except asyncio.CancelledError:
                        raise
                    except Exception:
                        self._fault = BackendCommandError("pvat_pump_failed")
                        self._running = False
                        self.invalidate()
                        return
                next_deadline = max(
                    next_deadline + self.period_s,
                    loop.time(),
                )
                await asyncio.sleep(
                    max(0.0, next_deadline - loop.time())
                )
        finally:
            if self._task is asyncio.current_task():
                self._task = None

    async def _take_latest(self) -> PvatRequest | None:
        while self._running and self._pending is None:
            self._wake.clear()
            await self._wake.wait()
        if not self._running:
            return None
        request, self._pending = self._pending, None
        if self._pending is None:
            self._wake.clear()
        return request
