from dataclasses import dataclass
from enum import StrEnum
from typing import Callable, Literal, Protocol

from app.schemas.messages import (
    BackendState,
    JointVector,
    Pose,
    RobotStateMessage,
)


class StopReason(StrEnum):
    GRIP_RELEASED = "grip_released"
    STALE = "stale"
    DISCONNECT = "disconnect"
    FAULT = "fault"
    HOME = "home"
    SHUTDOWN = "shutdown"


class BackendCommandError(RuntimeError):
    pass


HomePhase = Literal["homing", "stabilizing"]


@dataclass(frozen=True)
class HomeOptions:
    max_speed_radps: float
    timeout_s: float
    position_tolerance_rad: float
    velocity_tolerance_radps: float
    stable_seconds: float


@dataclass(frozen=True)
class BackendPreflight:
    ready: bool
    reason: str | None
    robot_state: BackendState
    actual_tcp: Pose
    actual_q: JointVector
    tcp_matches: bool
    capabilities: tuple[str, ...]


class RobotBackend(Protocol):
    async def connect(self) -> None: ...

    async def disconnect(self) -> None: ...

    async def command_tcp(self, target: Pose, command_id: int) -> None: ...

    async def set_gripper(self, value: float) -> None: ...

    async def stop(self, reason: StopReason) -> None: ...

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None: ...

    async def prepare(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None: ...

    async def get_state(self) -> RobotStateMessage: ...

    async def preflight(self) -> BackendPreflight: ...
