from typing import Callable, Never

from app.robots.base import BackendCommandError, HomeOptions, HomePhase, StopReason
from app.schemas.messages import Pose, RobotStateMessage


class RealLebaiAdapter:
    """Milestone 1 placeholder that cannot discover or contact real hardware."""

    @staticmethod
    def _disabled() -> Never:
        raise BackendCommandError("real_robot_disabled")

    async def connect(self) -> None:
        self._disabled()

    async def disconnect(self) -> None:
        self._disabled()

    async def command_tcp(self, target: Pose, command_id: int) -> None:
        self._disabled()

    async def set_gripper(self, value: float) -> None:
        self._disabled()

    async def stop(self, reason: StopReason) -> None:
        self._disabled()

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        self._disabled()

    async def get_state(self) -> RobotStateMessage:
        self._disabled()
