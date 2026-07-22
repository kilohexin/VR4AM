from __future__ import annotations

import asyncio
import contextlib
import time
from typing import Callable

import numpy as np

from app.robots.base import BackendCommandError, HomeOptions, HomePhase, StopReason
from app.schemas.messages import BackendState, Pose, RobotStateMessage, TeleopMode
from app.sim.ik import IKError, solve_ik
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model
from app.sim.virtual_robot import VirtualRobot


class SimRobotAdapter:
    STEP_SECONDS = 0.02

    def __init__(self) -> None:
        self.model = LM3Model()
        self.robot = VirtualRobot(self.model)
        self._ik_seed_q = np.asarray(self.model.home_q, dtype=float)
        self.gripper = 0.0
        self.command_id: int | None = None
        self.mode = TeleopMode.READY
        self.fault: str | None = None
        self._lock = asyncio.Lock()
        self._task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        await self.connect()

    async def connect(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="sim-robot-50hz")

    async def disconnect(self) -> None:
        task, self._task = self._task, None
        if task is None:
            return
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        next_deadline = loop.time()
        while True:
            next_deadline += self.STEP_SECONDS
            async with self._lock:
                self.robot.step(self.STEP_SECONDS)
            await asyncio.sleep(max(0.0, next_deadline - loop.time()))

    async def command_tcp(self, target: Pose, command_id: int) -> None:
        async with self._lock:
            try:
                result = solve_ik(target, self._ik_seed_q, self.model)
            except IKError as exc:
                raise BackendCommandError(str(exc)) from exc
            self.robot.set_target_q(result.q)
            self._ik_seed_q = np.asarray(result.q, dtype=float)
            self.command_id = command_id

    async def set_gripper(self, value: float) -> None:
        normalized = min(1.0, max(0.0, float(value)))
        async with self._lock:
            self.gripper = normalized

    async def stop(self, reason: StopReason) -> None:
        async with self._lock:
            self.robot.stop()

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + options.timeout_s
        stable_since: float | None = None
        on_phase("homing")
        async with self._lock:
            self.robot.set_target_q(
                self.model.home_q,
                max_speed_radps=options.max_speed_radps,
            )

        while True:
            now = loop.time()
            async with self._lock:
                if self.robot.target_q is None:
                    raise BackendCommandError("home_interrupted")
                position_error = float(
                    np.max(np.abs(self.robot.q - np.asarray(self.model.home_q)))
                )
                velocity = float(np.max(np.abs(self.robot.qd)))

            within_tolerance = (
                position_error <= options.position_tolerance_rad
                and velocity <= options.velocity_tolerance_radps
            )
            if within_tolerance:
                if stable_since is None:
                    stable_since = now
                    on_phase("stabilizing")
                elif now - stable_since >= options.stable_seconds:
                    self._ik_seed_q = np.asarray(self.model.home_q, dtype=float)
                    return
            elif stable_since is not None:
                stable_since = None
                on_phase("homing")

            if now >= deadline:
                async with self._lock:
                    self.robot.stop()
                raise BackendCommandError("home_timeout")
            await asyncio.sleep(self.STEP_SECONDS)

    async def get_state(self) -> RobotStateMessage:
        async with self._lock:
            backend_state = (
                BackendState.MOVING
                if np.any(np.abs(self.robot.qd) > 1e-4)
                else BackendState.IDLE
            )
            return RobotStateMessage(
                server_mono_ns=time.monotonic_ns(),
                ack_seq=self.command_id,
                mode=TeleopMode.READY,
                robot_state=backend_state,
                actual_tcp=forward_pose(self.robot.q, self.model),
                actual_q=tuple(float(value) for value in self.robot.q),
                gripper=self.gripper,
                sample_age_ms=None,
                fault=self.fault,
            )
