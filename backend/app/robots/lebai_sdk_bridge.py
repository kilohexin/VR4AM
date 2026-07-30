from __future__ import annotations

import importlib
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from app.robots.base import BackendCommandError


@runtime_checkable
class LebaiClientProtocol(Protocol):
    async def is_connected(self) -> bool: ...

    async def get_robot_state(self) -> object: ...

    async def get_estop_reason(self) -> object: ...

    async def get_kin_data(self) -> dict[str, object]: ...

    async def get_tcp(self) -> dict[str, object]: ...

    async def get_claw(self) -> dict[str, object]: ...

    async def get_running_motion(self) -> object: ...

    async def kinematics_inverse(
        self,
        pose: dict[str, float],
        joints: list[float],
    ) -> object: ...

    async def move_pvat(
        self,
        p: list[float],
        v: list[float],
        a: list[float],
        t: float,
    ) -> object: ...

    async def stop_move(self) -> None: ...

    async def stop_sys(self) -> None: ...

    async def movej(
        self,
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object: ...

    async def get_motion_state(self, motion_id: object) -> object: ...

    async def set_claw(self, force: int, amplitude: int) -> None: ...


@dataclass(frozen=True)
class SdkCapabilities:
    names: tuple[str, ...]

    @property
    def control_ready(self) -> bool:
        return self.names == _CAPABILITY_ORDER


_CAPABILITY_METHODS = {
    "state": (
        "is_connected",
        "get_robot_state",
        "get_estop_reason",
        "get_kin_data",
    ),
    "tcp": ("get_tcp",),
    "kinematics_inverse": ("kinematics_inverse",),
    "pvat": ("move_pvat",),
    "stop_move": ("stop_move",),
    "stop_sys": ("stop_sys",),
    "home": ("movej", "get_motion_state"),
    "gripper": ("get_claw", "set_claw"),
    "running_motion": ("get_running_motion",),
}
_CAPABILITY_ORDER = tuple(_CAPABILITY_METHODS)


def detect_capabilities(client: object) -> SdkCapabilities:
    names = tuple(
        name
        for name, methods in _CAPABILITY_METHODS.items()
        if all(callable(getattr(client, method, None)) for method in methods)
    )
    return SdkCapabilities(names=names)


async def connect_real_client(ip: str) -> LebaiClientProtocol:
    try:
        module = importlib.import_module("lebai_sdk")
    except ModuleNotFoundError:
        raise BackendCommandError("lebai_sdk_unavailable") from None
    try:
        module.init()
    except Exception:
        raise BackendCommandError("lebai_sdk_init_failed") from None
    try:
        client = await module.connect(ip, False)
    except Exception:
        raise BackendCommandError("lebai_sdk_connect_failed") from None
    return client
