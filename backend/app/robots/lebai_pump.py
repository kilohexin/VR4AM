from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field

from app.robots.base import BackendCommandError
from app.robots.command_admission import CommandAdmission, CommandCompletion, CommandRef
from app.schemas.messages import Pose

Clock = Callable[[], float]
Sleep = Callable[[float], Awaitable[None]]


@dataclass(frozen=True)
class PvatRequest:
    target: Pose
    command_id: int
    generation: int
    submission_id: int = 0  # Reserved for legacy, directly constructed requests.
    completion: CommandCompletion | None = field(default=None, compare=False, repr=False)

    @property
    def reference(self) -> CommandRef:
        return CommandRef(self.command_id, self.generation, self.submission_id)


class PvatPump:
    def __init__(
        self,
        handler: Callable[[PvatRequest], Awaitable[None]],
        *,
        period_s: float,
        clock: Clock | None = None,
        sleep: Sleep = asyncio.sleep,
    ) -> None:
        if period_s <= 0:
            raise ValueError("period_s_must_be_positive")
        self._handler = handler
        self.period_s = period_s
        self._clock = clock or _event_loop_time
        self._sleep = sleep
        self._pending: PvatRequest | None = None
        self._inflight: PvatRequest | None = None
        self._generation = 0
        self._submission_id = 0
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

    def submit(self, target: Pose, command_id: int) -> CommandAdmission:
        if not self._running:
            raise BackendCommandError("pvat_pump_not_running")
        if self._fault is not None:
            raise self._fault
        replaced = self._pending.reference if self._pending is not None else None
        completion = CommandCompletion(CommandRef(command_id, self._generation, self._submission_id + 1))
        request = PvatRequest(
            target=target.model_copy(deep=True),
            command_id=command_id,
            generation=self._generation,
            submission_id=self._submission_id + 1,
            completion=completion,
        )
        if self._pending is not None and self._pending.completion is not None:
            self._pending.completion._resolve('overwritten')
        self._submission_id = request.submission_id
        self._pending = request
        self._wake.set()
        return CommandAdmission(request.reference, replaced, completion=completion)

    def invalidate(self) -> int:
        self._generation += 1
        if self._pending is not None and self._pending.completion is not None:
            self._pending.completion._invalidate()
            self._pending.completion._resolve('invalidated')
        if self._inflight is not None and self._inflight.completion is not None:
            self._inflight.completion._invalidate()
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
        next_deadline = self._clock()
        try:
            while self._running:
                request = await self._take_latest()
                if request is None:
                    return
                if self.is_current(request.generation):
                    self._inflight = request
                    try:
                        await self._handler(request)
                    except asyncio.CancelledError:
                        if request.completion is not None:
                            request.completion._resolve('cancelled', sdk_outcome='unknown')
                        raise
                    except BackendCommandError as error:
                        if request.completion is not None:
                            request.completion._resolve('failed', error=str(error), sdk_outcome='unknown')
                        self._fault = error
                        self._running = False
                        self.invalidate()
                        return
                    except Exception:
                        if request.completion is not None:
                            request.completion._resolve('failed', error='pvat_pump_failed', sdk_outcome='unknown')
                        self._fault = BackendCommandError("pvat_pump_failed")
                        self._running = False
                        self.invalidate()
                        return
                    finally:
                        if request.completion is not None:
                            request.completion._resolve(
                                'invalidated' if request.completion.invalidated else 'handled',
                                sdk_outcome='unknown',
                            )
                        self._inflight = None
                next_deadline = max(
                    next_deadline + self.period_s,
                    self._clock(),
                )
                await self._sleep(
                    max(0.0, next_deadline - self._clock())
                )
        finally:
            if self._task is asyncio.current_task():
                # Unexpected external task cancellation must not strand queued
                # result handles or keep accepting requests with no consumer.
                if self._running:
                    self._fault = self._fault or BackendCommandError("pvat_pump_failed")
                    self._running = False
                    self.invalidate()
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


def _event_loop_time() -> float:
    return asyncio.get_running_loop().time()
