from __future__ import annotations

import argparse
import asyncio
import json
import math
import random
import sys
import time
from collections.abc import Awaitable, Callable, Sequence
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "backend"
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))

from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import (
    BackendCommandError,
    HomeOptions,
    HomePhase,
    StopReason,
)
from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import ControllerState, Pose, RobotStateMessage, TeleopMode, VRFrame

STEP_SECONDS = 0.02
STEP_NS = 20_000_000
STATE_PERIOD_NS = 50_000_000
CONTROLLER_PERIOD_STEPS = 5
CONTROLLER_FRAME_HZ = 1.0 / (STEP_SECONDS * CONTROLLER_PERIOD_STEPS)

EVENT_SCHEDULE = (
    (0.15, "tracking_loss"),
    (0.30, "hidden"),
    (0.45, "disconnect"),
    (0.60, "command_fault"),
    (0.68, "ik_boundary"),
    (0.75, "workspace_boundary"),
    (0.85, "visible_blurred"),
)
STOP_EVENT_REASONS = {
    "tracking_loss": StopReason.STALE,
    "hidden": StopReason.STALE,
    "visible_blurred": StopReason.STALE,
    "disconnect": StopReason.DISCONNECT,
    "command_fault": StopReason.FAULT,
}


class FakeMonotonicClock:
    def __init__(self) -> None:
        self.value_ns = 1_000_000_000

    def now_ns(self) -> int:
        return self.value_ns

    def advance_step(self) -> None:
        self.value_ns += STEP_NS


class AbsoluteDeadlinePacer:
    def __init__(
        self,
        *,
        period_seconds: float,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
    ) -> None:
        self.period_seconds = period_seconds
        self.monotonic = monotonic
        self.sleep = sleep
        self.next_deadline = monotonic()

    async def wait_next(self) -> None:
        self.next_deadline += self.period_seconds
        delay = max(0.0, self.next_deadline - self.monotonic())
        await self.sleep(delay)


def _all_finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class CountingSimAdapter(SimRobotAdapter):
    def __init__(self) -> None:
        super().__init__()
        self.commands = 0
        self.nan_count = 0
        self.fail_next_command = False
        self.fail_next_soft_constraint = False
        self.stop_counts = {reason.value: 0 for reason in StopReason}

    async def command_tcp(self, target: Pose, command_id: int) -> None:
        if not _all_finite((*target.p, *target.q)):
            self.nan_count += 1
        if self.fail_next_command:
            self.fail_next_command = False
            raise BackendCommandError("backend_command_failed")
        if self.fail_next_soft_constraint:
            self.fail_next_soft_constraint = False
            raise BackendCommandError("ik_unreachable")
        await super().command_tcp(target, command_id)
        self.commands += 1
        if self.robot.target_q is not None and not np.all(np.isfinite(self.robot.target_q)):
            self.nan_count += 1

    async def set_gripper(self, value: float) -> None:
        if not math.isfinite(float(value)):
            self.nan_count += 1
        await super().set_gripper(value)

    async def stop(self, reason: StopReason) -> None:
        self.stop_counts[reason.value] += 1
        await super().stop(reason)

    async def home(
        self,
        options: HomeOptions,
        on_phase: Callable[[HomePhase], None],
    ) -> None:
        on_phase("homing")
        self.robot.set_target_q(
            self.model.home_q,
            max_speed_radps=options.max_speed_radps,
        )
        stable_steps = max(1, math.ceil(options.stable_seconds / STEP_SECONDS))
        stable_count = 0
        for _ in range(math.ceil(options.timeout_s / STEP_SECONDS)):
            self.robot.step(STEP_SECONDS)
            position_error = float(
                np.max(
                    np.abs(
                        self.robot.q - np.asarray(self.model.home_q)
                    )
                )
            )
            velocity = float(np.max(np.abs(self.robot.qd)))
            if (
                position_error <= options.position_tolerance_rad
                and velocity <= options.velocity_tolerance_radps
            ):
                if stable_count == 0:
                    on_phase("stabilizing")
                stable_count += 1
                if stable_count >= stable_steps:
                    return
            else:
                stable_count = 0
            await asyncio.sleep(0)
        self.robot.stop()
        raise BackendCommandError("home_timeout")

    async def get_state(self) -> RobotStateMessage:
        state = await super().get_state()
        if not _all_finite(
            (*state.actual_tcp.p, *state.actual_tcp.q, *state.actual_q, state.gripper)
        ):
            self.nan_count += 1
        return state


class SoakScenario:
    def __init__(self, *, minutes: float, seed: int, realtime: bool) -> None:
        self.minutes = float(minutes)
        self.seed = int(seed)
        self.realtime = realtime
        self.total_steps = int(round(self.minutes * 60.0 / STEP_SECONDS))
        self.rng = random.Random(self.seed)
        self.clock = FakeMonotonicClock()
        self.latest = LatestVRFrame()
        self.adapter = CountingSimAdapter()
        self.control = RobotControl(
            backend=self.adapter,
            latest=self.latest,
            clock=self.clock,
            recorder=NoopRecorder(),
        )
        self.session_index = 0
        self.session_id = "soak-0"
        self.seq = 0
        self.frames = 0
        self.control_steps = 0
        self.virtual_steps = 0
        self.max_queue_depth = 0
        self.state_messages = 0
        self.state_accumulator_ns = 0
        self.mode_counts = {mode.value: 0 for mode in TeleopMode}
        self.path_checksum = 0
        self.grip = True
        self.next_grip_toggle = self._grip_interval_steps()
        self.recovery_event: str | None = None
        self.event_schedule = [
            (max(1, int(self.total_steps * fraction)), name)
            for fraction, name in EVENT_SCHEDULE
        ]
        self.event_index = 0
        self.injected_by_kind = {name: 0 for _fraction, name in EVENT_SCHEDULE}
        self.injected_events = 0
        self.verified_injected_events = 0
        self.stop_expected_events = 0
        self.verified_injected_stops = 0
        self.rearm_count = 0
        self.error_count = 0
        self.invariant_failures: list[str] = []
        self.realtime_pacer: AbsoluteDeadlinePacer | None = None
        self.phases = tuple(self.rng.uniform(-math.pi, math.pi) for _ in range(5))
        self.frequencies = tuple(self.rng.uniform(0.035, 0.11) for _ in range(4))

    def _grip_interval_steps(self) -> int:
        return self.rng.randint(100, 250)

    def _controller_sample(
        self, step: int
    ) -> tuple[tuple[float, float, float], tuple[float, float, float, float], float]:
        t = step * STEP_SECONDS
        x = 0.010 * math.sin(self.frequencies[0] * t + self.phases[0])
        y = 1.2 + 0.008 * math.sin(self.frequencies[1] * t + self.phases[1])
        z = -0.3 + 0.010 * math.sin(self.frequencies[2] * t + self.phases[2])
        yaw = 0.08 * math.sin(self.frequencies[3] * t + self.phases[3])
        quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
        trigger = 0.5 + 0.45 * math.sin(0.07 * t + self.phases[4])
        return (x, y, z), quaternion, trigger

    def _make_frame(
        self,
        step: int,
        *,
        grip: bool,
        tracking_valid: bool = True,
        visibility: str = "visible",
        workspace_boundary: bool = False,
    ) -> VRFrame:
        position, quaternion, trigger = self._controller_sample(step)
        if workspace_boundary:
            position = (position[0] + 0.5, position[1], position[2])
        if not _all_finite((*position, *quaternion, trigger)):
            self.adapter.nan_count += 1
        self.seq += 1
        quantized = sum(
            int(round(value * 1_000_000_000))
            for value in (*position, *quaternion, trigger)
        )
        self.path_checksum = (
            self.path_checksum + self.seq * quantized
        ) % 9_223_372_036_854_775_783
        return VRFrame(
            v=1,
            type="vr_frame",
            session_id=self.session_id,
            seq=self.seq,
            client_mono_ms=self.clock.now_ns() / 1_000_000.0,
            tracking_valid=tracking_valid,
            visibility=visibility,
            right=ControllerState(
                p=position,
                q=quaternion,
                grip=grip,
                trigger=trigger,
            ),
        )

    def _publish(self, frame: VRFrame) -> None:
        self.latest.publish(frame, self.clock.now_ns())
        self.frames += 1
        self._sample_storage_depth()

    def _sample_storage_depth(self) -> None:
        self.max_queue_depth = max(self.max_queue_depth, self.latest.depth)

    def _publish_normal(self, step: int, grip: bool) -> None:
        burst = step > 0 and step % 137 == 0
        burst_count = 3 if burst else 1
        last_seq = self.seq
        for offset in range(burst_count - 1, -1, -1):
            frame = self._make_frame(max(0, step - offset), grip=grip)
            self._publish(frame)
            last_seq = frame.seq
        snapshot = self.latest.snapshot()
        if snapshot is None or snapshot.frame.seq != last_seq:
            self.invariant_failures.append("latest_frame_not_newest")

    def _record_state(self, state: RobotStateMessage) -> None:
        self.state_messages += 1
        self.mode_counts[state.mode.value] += 1
        if not _all_finite(
            (*state.actual_tcp.p, *state.actual_tcp.q, *state.actual_q, state.gripper)
        ):
            self.adapter.nan_count += 1

    async def _publish_state(self) -> RobotStateMessage:
        state = await self.control.state_message()
        self._record_state(state)
        return state

    async def _initialize(self) -> None:
        await self.control.connect()
        self._publish(self._make_frame(0, grip=False))
        await self.control.tick()
        if self.control.mode is not TeleopMode.READY:
            self.invariant_failures.append("initial_release_not_ready")
        await self.control.arm()
        self.rearm_count += 1

    async def _normal_step(self, step: int) -> None:
        grip_changed = False
        if step >= self.next_grip_toggle:
            next_grip = not self.grip
            if next_grip and self.control.mode is TeleopMode.READY:
                await self.control.arm()
                self.rearm_count += 1
            self.grip = next_grip
            grip_changed = True
            self.next_grip_toggle = step + self._grip_interval_steps()

        if grip_changed or step % CONTROLLER_PERIOD_STEPS == 0:
            self._publish_normal(step, self.grip)
        await self._tick_control()
        if not self.grip and self.control.mode is TeleopMode.HOLD:
            await self.control.disarm()

    async def _tick_control(self) -> None:
        await self.control.tick()
        self.control_steps += 1

    async def _recover(self, step: int) -> None:
        if self.control.mode in {TeleopMode.STALE, TeleopMode.FAULT}:
            await self._tick_control()
            return
        if self.recovery_event == "command_fault":
            reset = await self.control.reset_fault()
            if not reset.accepted:
                self.invariant_failures.append(
                    f"{self.recovery_event}_explicit_reset_rejected"
                )
                return
        if self.control.mode is TeleopMode.DISCONNECTED:
            await self.control.connect()
        if self.control.mode is TeleopMode.DISARMED:
            self._publish(self._make_frame(step, grip=False))
        elif self.control.mode is TeleopMode.READY:
            self._publish(self._make_frame(step, grip=False))
        await self._tick_control()
        if self.control.mode is not TeleopMode.READY:
            self.invariant_failures.append(
                f"{self.recovery_event}_release_did_not_reach_ready"
            )
            return
        await self.control.arm()
        self.rearm_count += 1
        self.grip = True
        self.next_grip_toggle = step + self._grip_interval_steps()
        self.recovery_event = None

    def _replace_session(self) -> None:
        self.session_index += 1
        self.session_id = f"soak-{self.session_index}"
        self.seq = 0
        self.latest = LatestVRFrame()
        self.control.latest = self.latest
        self._sample_storage_depth()

    async def _inject_event(self, step: int, name: str) -> bool:
        if (
            name in {"command_fault", "ik_boundary", "workspace_boundary"}
            and self.control.mode is not TeleopMode.ACTIVE
        ):
            return False
        expected_reason = STOP_EVENT_REASONS.get(name)
        before = (
            self.adapter.stop_counts[expected_reason.value]
            if expected_reason is not None
            else self.adapter.stop_counts[StopReason.FAULT.value]
        )
        event_verified = True

        if name == "disconnect":
            await self.control.on_disconnect()
            self._replace_session()
            await self._tick_control()
        else:
            if name == "command_fault":
                self.adapter.fail_next_command = True
            elif name == "ik_boundary":
                self.adapter.fail_next_soft_constraint = True
            frame = self._make_frame(
                step,
                grip=True,
                tracking_valid=name != "tracking_loss",
                visibility=(
                    "hidden"
                    if name == "hidden"
                    else "visible-blurred"
                    if name == "visible_blurred"
                    else "visible"
                ),
                workspace_boundary=name == "workspace_boundary",
            )
            self._publish(frame)
            await self._tick_control()
            if name in {"ik_boundary", "workspace_boundary"}:
                expected_mode = TeleopMode.ACTIVE
            else:
                expected_mode = (
                    TeleopMode.STALE
                    if expected_reason is StopReason.STALE
                    else TeleopMode.FAULT
                )
            if self.control.mode is not expected_mode:
                self.invariant_failures.append(f"{name}_gate_not_closed")
                event_verified = False
            published = await self._publish_state()
            if published.mode is not expected_mode:
                self.invariant_failures.append(f"{name}_mode_not_observable")
                event_verified = False
            expected_constraint = {
                "ik_boundary": "ik_boundary",
                "workspace_boundary": "workspace_boundary",
            }.get(name)
            if (
                expected_constraint is not None
                and published.constraint != expected_constraint
            ):
                self.invariant_failures.append(
                    f"{name}_constraint_not_observable"
                )
                event_verified = False

        self.injected_by_kind[name] += 1
        self.injected_events += 1
        if expected_reason is not None:
            self.stop_expected_events += 1
            after = self.adapter.stop_counts[expected_reason.value]
            if after == before + 1:
                self.verified_injected_stops += 1
            else:
                self.invariant_failures.append(f"{name}_stop_count")
                event_verified = False
        else:
            after = self.adapter.stop_counts[StopReason.FAULT.value]
            if after != before:
                self.invariant_failures.append(
                    f"{name}_unexpected_fault_stop"
                )
                event_verified = False
        if event_verified:
            self.verified_injected_events += 1
        self.recovery_event = name if expected_reason is not None else None
        return True

    async def _advance(self) -> None:
        self.adapter.robot.step(STEP_SECONDS)
        self.virtual_steps += 1
        if not np.all(np.isfinite(self.adapter.robot.q)) or not np.all(
            np.isfinite(self.adapter.robot.qd)
        ):
            self.adapter.nan_count += 1
        self.clock.advance_step()
        self.state_accumulator_ns += STEP_NS
        if self.state_accumulator_ns >= STATE_PERIOD_NS:
            self.state_accumulator_ns -= STATE_PERIOD_NS
            await self._publish_state()
        if self.realtime_pacer is not None:
            await self.realtime_pacer.wait_next()

    async def run(self) -> dict[str, Any]:
        await self._initialize()
        if self.realtime:
            self.realtime_pacer = AbsoluteDeadlinePacer(period_seconds=STEP_SECONDS)
        for step in range(self.total_steps):
            if self.recovery_event is not None:
                await self._recover(step)
            else:
                event_due = (
                    self.event_index < len(self.event_schedule)
                    and step >= self.event_schedule[self.event_index][0]
                )
                if event_due:
                    name = self.event_schedule[self.event_index][1]
                    if await self._inject_event(step, name):
                        self.event_index += 1
                    else:
                        await self._normal_step(step)
                else:
                    await self._normal_step(step)
            await self._advance()

        if self.recovery_event is not None:
            self.invariant_failures.append("recovery_incomplete")
        if self.event_index != len(self.event_schedule):
            self.invariant_failures.append("scheduled_events_missing")
        if self.control.mode not in {TeleopMode.STALE, TeleopMode.FAULT}:
            await self.control.disarm()
        if self.control.mode is not TeleopMode.DISARMED:
            self.invariant_failures.append("final_mode_not_disarmed")
        if self.max_queue_depth != 1:
            self.invariant_failures.append("capacity_one_violated")
        if self.control_steps != self.total_steps:
            self.invariant_failures.append("control_step_count_mismatch")
        if self.virtual_steps != self.total_steps:
            self.invariant_failures.append("virtual_step_count_mismatch")
        if self.injected_events != self.verified_injected_events:
            self.invariant_failures.append("injected_event_unverified")
        if self.stop_expected_events != self.verified_injected_stops:
            self.invariant_failures.append("injected_stop_unverified")
        if self.adapter.nan_count != 0:
            self.invariant_failures.append("non_finite_value")

        self.error_count = len(self.invariant_failures)
        return {
            "simulated_minutes": self.minutes,
            "seed": self.seed,
            "frames": self.frames,
            "controller_frame_hz": CONTROLLER_FRAME_HZ,
            "control_steps": self.control_steps,
            "virtual_steps": self.virtual_steps,
            "commands": self.adapter.commands,
            "stops": sum(self.adapter.stop_counts.values()),
            "max_queue_depth": self.max_queue_depth,
            "nan_count": self.adapter.nan_count,
            "error_count": self.error_count,
            "final_mode": self.control.mode.value,
            "injected_events": self.injected_events,
            "verified_injected_events": self.verified_injected_events,
            "stop_expected_events": self.stop_expected_events,
            "verified_injected_stops": self.verified_injected_stops,
            "injected_by_kind": dict(self.injected_by_kind),
            "stop_counts": dict(self.adapter.stop_counts),
            "state_messages": self.state_messages,
            "mode_counts": dict(self.mode_counts),
            "rearm_count": self.rearm_count,
            "path_checksum": self.path_checksum,
        }


def run_soak(minutes: float, seed: int, realtime: bool = False) -> dict[str, Any]:
    if not math.isfinite(float(minutes)) or minutes <= 0:
        raise ValueError("minutes_must_be_positive_finite")
    return asyncio.run(
        SoakScenario(minutes=float(minutes), seed=int(seed), realtime=realtime).run()
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the deterministic LM3 simulator soak")
    parser.add_argument("--minutes", type=float, default=10.0)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--realtime", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        summary = run_soak(args.minutes, args.seed, args.realtime)
    except Exception as error:
        print(f"soak failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps(summary, sort_keys=True, separators=(",", ":")))
    return 1 if summary["error_count"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
