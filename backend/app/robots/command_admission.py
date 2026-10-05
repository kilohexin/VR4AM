"""Immutable local queue evidence, scoped to one adapter/pump instance.

An admission is not IK acceptance, SDK completion or physical execution.
Generation is the pump cancellation generation, not RobotControl authority.
"""

import asyncio
from dataclasses import dataclass, field
from typing import Literal


@dataclass(frozen=True)
class CommandRef:
    command_id: int
    generation: int
    submission_id: int


@dataclass(frozen=True)
class CommandFeedback:
    # None explicitly means the caller supplied no originating request.
    command: CommandRef | None
    code: str
    outcome: Literal['catch_up', 'rejected', 'interpolated_advance']


@dataclass(frozen=True)
class CommandResult:
    command: CommandRef
    outcome: Literal['overwritten', 'invalidated', 'handled', 'ik_rejected',
                     'sdk_returned', 'failed', 'cancelled']
    invalidated: bool = False
    sdk_outcome: Literal['not_started', 'unknown', 'returned', 'error', 'timeout', 'cancelled'] = 'not_started'
    sdk_request_id: int | None = None
    accepted_command: CommandRef | None = None
    pvat_mode: Literal['advance', 'interpolated_advance', 'catch_up'] | None = None
    error: str | None = None


class CommandCompletion:
    """Caller-owned result handle; cancelling wait() cannot cancel dispatch.

    No global history is retained. A request in a non-returning SDK/handler can
    remain unresolved even after invalidation; never invent a remote outcome.
    """

    def __init__(self, command: CommandRef) -> None:
        self._command = command
        self._future: asyncio.Future[CommandResult] = asyncio.get_running_loop().create_future()
        self._invalidated = False

    @property
    def command(self) -> CommandRef:
        return self._command

    @property
    def invalidated(self) -> bool:
        return self._invalidated

    @property
    def result(self) -> CommandResult | None:
        return self._future.result() if self._future.done() else None

    async def wait(self) -> CommandResult:
        return await asyncio.shield(self._future)

    def _invalidate(self) -> None:
        self._invalidated = True

    def _resolve(self, outcome: str, **details) -> None:
        if not self._future.done():
            self._future.set_result(CommandResult(
                command=self.command, outcome=outcome,
                invalidated=self._invalidated, **details,
            ))


@dataclass(frozen=True)
class CommandAdmission:
    command: CommandRef
    replaced_pending: CommandRef | None
    previous_feedback: CommandFeedback | None = None
    completion: CommandCompletion | None = field(default=None, compare=False, repr=False)
