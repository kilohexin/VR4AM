"""Immutable local queue evidence, scoped to one adapter/pump instance.

An admission is not IK acceptance, SDK completion or physical execution.
Generation is the pump cancellation generation, not RobotControl authority.
"""

from dataclasses import dataclass
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
class CommandAdmission:
    command: CommandRef
    replaced_pending: CommandRef | None
    previous_feedback: CommandFeedback | None = None
