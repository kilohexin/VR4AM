from enum import StrEnum
from typing import Protocol

from app.schemas.messages import Pose, RobotStateMessage


class StopReason(StrEnum):
    GRIP_RELEASED = "grip_released"
    STALE = "stale"
    DISCONNECT = "disconnect"
    FAULT = "fault"
    SHUTDOWN = "shutdown"


class BackendCommandError(RuntimeError):
    pass


class RobotBackend(Protocol):
    async def connect(self) -> None: ...

    async def disconnect(self) -> None: ...

    async def command_tcp(self, target: Pose, command_id: int) -> None: ...

    async def set_gripper(self, value: float) -> None: ...

    async def stop(self, reason: StopReason) -> None: ...

    async def get_state(self) -> RobotStateMessage: ...
