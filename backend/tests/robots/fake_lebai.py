from __future__ import annotations

import asyncio
from collections import deque
from typing import Any


IDENTITY_TCP = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.175,
    "rz": 0.0,
    "ry": 0.0,
    "rx": 0.0,
}
ACTUAL_TCP = {
    "x": 0.3,
    "y": 0.0,
    "z": 0.4,
    "rz": 0.0,
    "ry": 0.0,
    "rx": 0.0,
}
IDLE_Q = [0.0, -1.0, 1.0, 0.0, 1.57, 0.0]


class FakeLebaiClient:
    def __init__(self) -> None:
        self.connected = True
        self.robot_state: object = "IDLE"
        self.estop_reason: object = 0
        self.tcp: dict[str, object] = dict(IDENTITY_TCP)
        self.claw: dict[str, object] = {"force": 0, "amplitude": 100}
        self.running_motion: object = None
        self.motion_state: object = "FINISHED"
        self.kin_data: dict[str, object] = {
            "actual_joint_pose": list(IDLE_Q),
            "actual_joint_speed": [0.0] * 6,
            "actual_joint_acc": [0.0] * 6,
            "actual_joint_torque": [0.0] * 6,
            "target_joint_pose": list(IDLE_Q),
            "target_joint_speed": [0.0] * 6,
            "target_joint_acc": [0.0] * 6,
            "target_joint_torque": [0.0] * 6,
            "actual_tcp_pose": dict(ACTUAL_TCP),
            "target_tcp_pose": dict(ACTUAL_TCP),
            "actual_flange_pose": dict(ACTUAL_TCP),
        }
        self.ik_results: deque[object] = deque([list(IDLE_Q)])
        self.read_calls: list[str] = []
        self.write_calls: list[tuple[Any, ...]] = []
        self.ik_calls: list[tuple[dict[str, float], list[float]]] = []
        self.block_ik = False
        self.ik_started = asyncio.Event()
        self.release_ik = asyncio.Event()
        self.discovery_calls = 0
        self.network_calls = 0
        self._write_event = asyncio.Event()

    @classmethod
    def idle(
        cls,
        *,
        tcp: dict[str, object] | None = None,
        q: list[float] | None = None,
    ) -> "FakeLebaiClient":
        client = cls()
        if tcp is not None:
            client.tcp = dict(tcp)
        if q is not None:
            client.kin_data["actual_joint_pose"] = list(q)
            client.kin_data["target_joint_pose"] = list(q)
            client.ik_results = deque([list(q)])
        return client

    async def is_connected(self) -> bool:
        self.read_calls.append("is_connected")
        return self.connected

    async def get_robot_state(self) -> object:
        self.read_calls.append("get_robot_state")
        return self.robot_state

    async def get_estop_reason(self) -> object:
        self.read_calls.append("get_estop_reason")
        return self.estop_reason

    async def get_kin_data(self) -> dict[str, object]:
        self.read_calls.append("get_kin_data")
        return dict(self.kin_data)

    async def get_tcp(self) -> dict[str, object]:
        self.read_calls.append("get_tcp")
        return dict(self.tcp)

    async def get_claw(self) -> dict[str, object]:
        self.read_calls.append("get_claw")
        return dict(self.claw)

    async def get_running_motion(self) -> object:
        self.read_calls.append("get_running_motion")
        return self.running_motion

    async def kinematics_inverse(
        self,
        pose: dict[str, float],
        joints: list[float],
    ) -> object:
        self.read_calls.append("kinematics_inverse")
        self.ik_calls.append((dict(pose), list(joints)))
        self.ik_started.set()
        if self.block_ik:
            await self.release_ik.wait()
        if not self.ik_results:
            return None
        return self.ik_results.popleft()

    async def move_pvat(
        self,
        p: list[float],
        v: list[float],
        a: list[float],
        t: float,
    ) -> object:
        call = ("move_pvat", list(p), list(v), list(a), t)
        self.write_calls.append(call)
        self._write_event.set()
        return len(self.write_calls)

    async def stop_move(self) -> None:
        self.write_calls.append(("stop_move",))
        self._write_event.set()

    async def stop_sys(self) -> None:
        self.write_calls.append(("stop_sys",))
        self._write_event.set()

    async def movej(
        self,
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object:
        call = ("movej", list(p), a, v, t, r)
        self.write_calls.append(call)
        self._write_event.set()
        return len(self.write_calls)

    async def get_motion_state(self, motion_id: object) -> object:
        self.read_calls.append("get_motion_state")
        return self.motion_state

    async def set_claw(self, force: int, amplitude: int) -> None:
        self.write_calls.append(("set_claw", force, amplitude))
        self._write_event.set()

    async def wait_for_write(
        self,
        method: str,
        timeout: float = 0.5,
    ) -> None:
        async def has_method() -> None:
            while method not in [str(call[0]) for call in self.write_calls]:
                self._write_event.clear()
                await self._write_event.wait()

        await asyncio.wait_for(has_method(), timeout)


class FakeLebaiModule:
    def __init__(self, client: FakeLebaiClient | None = None) -> None:
        self.client = client or FakeLebaiClient()
        self.init_count = 0
        self.connect_calls: list[tuple[str, bool]] = []

    def init(self) -> None:
        self.init_count += 1

    async def connect(self, ip: str, discover: bool) -> FakeLebaiClient:
        self.connect_calls.append((ip, discover))
        return self.client
