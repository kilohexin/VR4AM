from __future__ import annotations

import asyncio
import math
import time
from dataclasses import dataclass
from typing import Callable

import numpy as np

from app.config import LebaiSettings
from app.robots.lebai_codec import joint_vector, pose_from_lebai, pose_to_lebai
from app.schemas.messages import JointVector
from app.sim.cartesian_servo import cartesian_servo_step
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


@dataclass(frozen=True)
class DigitalTwinFaults:
    sdk_latency_s: float = 0.0
    disconnect: bool = False
    ik_failure: bool = False
    pvat_failure: bool = False
    stop_failure: bool = False

    def __post_init__(self) -> None:
        if not math.isfinite(self.sdk_latency_s) or self.sdk_latency_s < 0:
            raise ValueError("digital_twin_invalid_sdk_latency")


@dataclass(frozen=True)
class _Motion:
    started_ns: int
    duration_ns: int
    start_q: JointVector
    target_q: JointVector
    target_qd: JointVector
    target_qdd: JointVector


def _joint_tuple(values: object) -> JointVector:
    array = np.asarray(values, dtype=float)
    if array.shape != (6,) or not np.all(np.isfinite(array)):
        raise RuntimeError("digital_twin_invalid_joint_vector")
    return tuple(float(value) for value in array)  # type: ignore[return-value]


class DigitalTwinLebaiClient:
    def __init__(
        self,
        settings: LebaiSettings,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        faults: DigitalTwinFaults | None = None,
        initial_q: object | None = None,
    ) -> None:
        self.settings = settings
        self.model = LM3Model()
        self._clock = clock
        self._faults = faults or DigitalTwinFaults()
        self._q = _joint_tuple(
            settings.home_q if initial_q is None else initial_q
        )
        self._qd = (0.0,) * 6
        self._qdd = (0.0,) * 6
        self._motion: _Motion | None = None
        self._gripper_force = 0
        self._gripper_amplitude = settings.gripper.open_amplitude_percent
        self._write_calls: list[tuple[object, ...]] = []
        self._running_motion_id: int | None = None

    @classmethod
    def idle(
        cls,
        settings: LebaiSettings,
        *,
        clock: Callable[[], int] = time.monotonic_ns,
        faults: DigitalTwinFaults | None = None,
        initial_q: object | None = None,
    ) -> "DigitalTwinLebaiClient":
        return cls(
            settings,
            clock=clock,
            faults=faults,
            initial_q=initial_q,
        )

    def set_faults(self, faults: DigitalTwinFaults) -> None:
        self._faults = faults

    async def is_connected(self) -> bool:
        await self._delay()
        return not self._faults.disconnect

    async def get_robot_state(self) -> object:
        await self._delay()
        self._require_connected()
        self._advance()
        return "MOVING" if self._motion is not None else "IDLE"

    async def get_estop_reason(self) -> object:
        await self._delay()
        self._require_connected()
        self._advance()
        return 0

    async def get_kin_data(self) -> dict[str, object]:
        await self._delay()
        self._require_connected()
        self._advance()
        actual_tcp = pose_to_lebai(forward_pose(self._q, self.model))
        if self._motion is None:
            target_q, target_qd, target_qdd = self._q, self._qd, self._qdd
        else:
            target_q = self._motion.target_q
            target_qd = self._motion.target_qd
            target_qdd = self._motion.target_qdd
        target_tcp = pose_to_lebai(forward_pose(target_q, self.model))
        return {
            "actual_joint_pose": list(self._q),
            "actual_joint_speed": list(self._qd),
            "actual_joint_acc": list(self._qdd),
            "actual_joint_torque": [0.0] * 6,
            "target_joint_pose": list(target_q),
            "target_joint_speed": list(target_qd),
            "target_joint_acc": list(target_qdd),
            "target_joint_torque": [0.0] * 6,
            "actual_tcp_pose": actual_tcp,
            "target_tcp_pose": target_tcp,
            "actual_flange_pose": actual_tcp,
        }

    async def get_tcp(self) -> dict[str, object]:
        await self._delay()
        self._require_connected()
        self._advance()
        tcp = self.settings.expected_tcp
        return {
            "x": tcp.x,
            "y": tcp.y,
            "z": tcp.z,
            "rz": tcp.rz,
            "ry": tcp.ry,
            "rx": tcp.rx,
        }

    async def get_claw(self) -> dict[str, object]:
        await self._delay()
        self._require_connected()
        self._advance()
        return {"force": self._gripper_force, "amplitude": self._gripper_amplitude}

    async def get_running_motion(self) -> object:
        await self._delay()
        self._require_connected()
        self._advance()
        return self._running_motion_id if self._motion is not None else None

    async def kinematics_inverse(
        self,
        pose: dict[str, float],
        joints: list[float],
    ) -> object:
        await self._delay()
        self._require_connected()
        if self._faults.ik_failure:
            return None
        result = cartesian_servo_step(
            pose_from_lebai(pose),
            np.asarray(joints, dtype=float),
            self.model,
            # Keep the fake SDK's local IK step below the adapter's joint-step
            # ceiling with margin.  Using the slower PVAT cadence here can
            # manufacture a joint jump for an otherwise valid TCP command.
            dt=0.5 / self.settings.control.loop_hz,
        )
        return list(result.q)

    async def move_pvat(
        self,
        p: list[float],
        v: list[float],
        a: list[float],
        t: float,
    ) -> object:
        await self._delay()
        self._require_connected()
        if self._faults.pvat_failure:
            raise RuntimeError("digital_twin_pvat_failure")
        self._advance()
        self._motion = _Motion(
            started_ns=self._clock(),
            duration_ns=max(1, round(t * 1_000_000_000)),
            start_q=self._q,
            target_q=joint_vector(p, "pvat_position"),
            target_qd=joint_vector(v, "pvat_velocity"),
            target_qdd=joint_vector(a, "pvat_acceleration"),
        )
        self._write_calls.append(("move_pvat", list(p), list(v), list(a), t))
        self._running_motion_id = len(self._write_calls)
        return self._running_motion_id

    async def stop_move(self) -> None:
        await self._delay()
        self._require_connected()
        self._advance()
        self._motion = None
        self._running_motion_id = None
        if self._faults.stop_failure:
            self._qd = (0.03,) * 6
            self._qdd = (0.0,) * 6
        else:
            self._qd = (0.0,) * 6
            self._qdd = (0.0,) * 6
        self._write_calls.append(("stop_move",))

    async def stop_sys(self) -> None:
        await self._delay()
        self._require_connected()
        self._advance()
        self._motion = None
        self._running_motion_id = None
        if self._faults.stop_failure:
            self._qd = (0.03,) * 6
            self._qdd = (0.0,) * 6
        else:
            self._qd = (0.0,) * 6
            self._qdd = (0.0,) * 6
        self._write_calls.append(("stop_sys",))

    async def movej(
        self,
        p: list[float],
        a: float,
        v: float,
        t: float,
        r: float,
    ) -> object:
        await self._delay()
        self._require_connected()
        self._advance()
        target_q = joint_vector(p, "movej_position")
        duration_s = t if t > 0 else self.settings.control.pvat_horizon_s
        self._motion = _Motion(
            started_ns=self._clock(),
            duration_ns=max(1, round(duration_s * 1_000_000_000)),
            start_q=self._q,
            target_q=target_q,
            target_qd=_joint_tuple([v] * 6),
            target_qdd=_joint_tuple([a] * 6),
        )
        self._write_calls.append(("movej", list(p), a, v, t, r))
        self._running_motion_id = len(self._write_calls)
        return self._running_motion_id

    async def get_motion_state(self, motion_id: object) -> object:
        await self._delay()
        self._require_connected()
        self._advance()
        if self._motion is not None and motion_id == self._running_motion_id:
            return "RUNNING"
        return "FINISHED"

    async def set_claw(self, force: int, amplitude: int) -> None:
        await self._delay()
        self._require_connected()
        self._gripper_force = force
        self._gripper_amplitude = amplitude
        self._write_calls.append(("set_claw", force, amplitude))

    async def _delay(self) -> None:
        if self._faults.sdk_latency_s:
            await asyncio.sleep(self._faults.sdk_latency_s)

    def _require_connected(self) -> None:
        if self._faults.disconnect:
            raise RuntimeError("digital_twin_disconnected")

    def _advance(self) -> None:
        motion = self._motion
        if motion is None:
            return
        elapsed = max(0, self._clock() - motion.started_ns)
        alpha = min(1.0, elapsed / motion.duration_ns)
        start = np.asarray(motion.start_q)
        target = np.asarray(motion.target_q)
        self._q = _joint_tuple(start + alpha * (target - start))
        if alpha >= 1.0:
            self._qd = (0.0,) * 6
            self._qdd = (0.0,) * 6
            self._motion = None
            self._running_motion_id = None
        else:
            self._qd = motion.target_qd
            self._qdd = motion.target_qdd
