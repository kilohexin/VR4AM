"""Explicit target preparation/dispatch seam, supervised by RobotControl.

prepare() is synchronous and must not send commands or commit accepted history.
Its draft may be dropped after recording if input authority has changed. It must
not require dispatch() to run for cleanup. dispatch() returns a pose only when
the policy wants the owner to commit it; None leaves last_target unchanged.
Asynchronous receipts are not physical execution or stopping evidence.
"""

from dataclasses import dataclass, field
from typing import Literal, Protocol

from app.control.safety import SafetyLimiter
from app.robots.base import RobotBackend
from app.schemas.messages import Pose


@dataclass(frozen=True)
class TargetPlan:
    kind: Literal['candidate', 'catch_up', 'wait']
    pose: Pose | None
    payload: object = field(default=None, compare=False, repr=False)

    def __post_init__(self):
        if self.kind not in {'candidate', 'catch_up', 'wait'}:
            raise ValueError('invalid_target_plan_kind')
        if (self.kind == 'wait') != (self.pose is None):
            raise ValueError('invalid_target_plan_pose')


class TargetStrategy(Protocol):
    def prepare(self, previous: Pose, requested: Pose, dt: float, *,
                limiter: SafetyLimiter) -> TargetPlan: ...

    async def dispatch(self, plan: TargetPlan, command_id: int, *,
                       backend: RobotBackend) -> Pose | None: ...


class DefaultTargetStrategy:
    """Compatibility path: original limiter and command_tcp soft-error rules."""

    def prepare(self, previous, requested, dt, *, limiter):
        return TargetPlan('candidate', limiter.limit_motion(previous, requested, dt))

    async def dispatch(self, plan, command_id, *, backend):
        assert plan.pose is not None
        await backend.command_tcp(plan.pose, command_id)
        return plan.pose
