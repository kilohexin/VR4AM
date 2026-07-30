from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper
from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError, StopReason
from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import (
    ControllerState,
    Pose,
    TeleopMode,
    VRFrame,
)
from app.sim.cartesian_servo import cartesian_servo_step
from app.sim.ik import IKError
from app.sim.kinematics import forward_pose


@dataclass(frozen=True)
class ScenarioResult:
    name: str
    passed: bool
    metrics: dict[str, object]
    failures: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "passed": self.passed,
            "metrics": {
                key: _json_value(value)
                for key, value in sorted(self.metrics.items())
            },
            "failures": list(self.failures),
        }


def _json_value(value: object) -> object:
    if isinstance(value, tuple):
        return [_json_value(item) for item in value]
    if isinstance(value, list):
        return [_json_value(item) for item in value]
    if isinstance(value, dict):
        return {
            str(key): _json_value(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    return value


def run_mapping_scenario() -> ScenarioResult:
    mapper = CoordinateMapper(
        translation_scale=1.0,
        rotation_scale=1.0,
        rotation_dead_zone_deg=0.0,
    )
    hand_anchor = Pose(
        p=(0.0, 1.2, -0.3),
        q=(0.0, 0.0, 0.0, 1.0),
    )
    tcp_rotation = Rotation.from_euler(
        "xyz",
        (-30.0, 5.0, 40.0),
        degrees=True,
    )
    tcp_anchor = Pose(
        p=(0.3, 0.4, -0.2),
        q=tuple(float(value) for value in tcp_rotation.as_quat()),
    )
    mapper.capture(hand_anchor, tcp_anchor)

    translation_errors: list[float] = []
    for axis in range(3):
        delta = np.zeros(3)
        delta[axis] = 0.02
        target = mapper.target(
            Pose(
                p=tuple(np.asarray(hand_anchor.p) + delta),
                q=hand_anchor.q,
            )
        )
        expected = np.asarray(tcp_anchor.p) + delta
        translation_errors.append(
            float(np.linalg.norm(np.asarray(target.p) - expected))
        )

    rotation_errors: list[float] = []
    for axis in "xyz":
        delta_rotation = Rotation.from_euler(
            axis,
            10.0,
            degrees=True,
        )
        target = mapper.target(
            Pose(
                p=hand_anchor.p,
                q=tuple(float(value) for value in delta_rotation.as_quat()),
            )
        )
        expected = delta_rotation * tcp_rotation
        actual = Rotation.from_quat(target.q)
        rotation_errors.append(
            float((actual * expected.inv()).magnitude())
        )

    max_translation_error = max(translation_errors, default=0.0)
    max_rotation_error = max(rotation_errors, default=0.0)
    failures: list[str] = []
    if max_translation_error > 1e-9:
        failures.append("translation_axis_mapping")
    if max_rotation_error > 1e-9:
        failures.append("rotation_axis_mapping")
    return ScenarioResult(
        name="six_axis_mapping",
        passed=not failures,
        metrics={
            "translation_axes_checked": 3,
            "rotation_axes_checked": 3,
            "max_translation_error_m": max_translation_error,
            "max_rotation_error_rad": max_rotation_error,
        },
        failures=tuple(failures),
    )


async def run_servo_tracking_scenario() -> ScenarioResult:
    adapter = SimRobotAdapter()
    model = adapter.model
    target_q = np.asarray(model.home_q) + np.array(
        [0.03, -0.02, 0.025, 0.015, -0.01, 0.02]
    )
    target = forward_pose(target_q, model)
    initial_tcp = forward_pose(adapter.robot.q, model)
    initial_error = float(
        np.linalg.norm(
            np.asarray(initial_tcp.p) - np.asarray(target.p)
        )
    )
    max_joint_speed = 0.0
    joint_window_violations = 0
    nan_count = 0
    failures: list[str] = []

    for cycle in range(150):
        try:
            await adapter.command_tcp(target, command_id=cycle)
        except BackendCommandError as error:
            failures.append(f"command:{error}")
            break
        adapter.robot.step(adapter.STEP_SECONDS)
        max_joint_speed = max(
            max_joint_speed,
            float(np.max(np.abs(adapter.robot.qd))),
        )
        if np.any(
            np.abs(adapter.robot.q - np.asarray(model.home_q))
            > model.joint_window_rad
        ):
            joint_window_violations += 1
        if (
            not np.all(np.isfinite(adapter.robot.q))
            or not np.all(np.isfinite(adapter.robot.qd))
        ):
            nan_count += 1

    final_tcp = forward_pose(adapter.robot.q, model)
    final_error = float(
        np.linalg.norm(
            np.asarray(final_tcp.p) - np.asarray(target.p)
        )
    )
    if final_error >= initial_error:
        failures.append("position_error_not_decreasing")
    if final_error > 0.005:
        failures.append("position_error_above_tolerance")
    if max_joint_speed > model.max_joint_speed_radps + 1e-9:
        failures.append("joint_speed_limit")
    if joint_window_violations:
        failures.append("joint_window")
    if nan_count:
        failures.append("non_finite_state")
    return ScenarioResult(
        name="servo_tracking",
        passed=not failures,
        metrics={
            "cycles": 150,
            "initial_position_error_m": initial_error,
            "final_position_error_m": final_error,
            "max_joint_speed_radps": max_joint_speed,
            "joint_window_violations": joint_window_violations,
            "nan_count": nan_count,
        },
        failures=tuple(failures),
    )


class _AcceptanceClock:
    def __init__(self) -> None:
        self.value_ns = 1_000_000_000

    def now_ns(self) -> int:
        return self.value_ns

    def advance_ms(self, milliseconds: float) -> None:
        self.value_ns += int(milliseconds * 1_000_000)


class _StopCountingSimAdapter(SimRobotAdapter):
    def __init__(self, *, servo: Callable[..., object]) -> None:
        super().__init__(servo=servo)
        self.stop_counts = {reason: 0 for reason in StopReason}

    async def stop(self, reason: StopReason) -> None:
        self.stop_counts[reason] += 1
        await super().stop(reason)


def _frame(
    seq: int,
    *,
    grip: bool,
    position: tuple[float, float, float] = (0.0, 1.2, -0.3),
) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id="acceptance-soft-constraint",
        seq=seq,
        client_mono_ms=float(seq * 20),
        tracking_valid=True,
        visibility="visible",
        right=ControllerState(
            p=position,
            q=(0.0, 0.0, 0.0, 1.0),
            grip=grip,
            trigger=0.0,
        ),
    )


async def run_soft_constraint_scenario() -> ScenarioResult:
    calls = 0

    def fail_once_then_delegate(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 1:
            raise IKError("ik_unreachable")
        return cartesian_servo_step(*args, **kwargs)

    adapter = _StopCountingSimAdapter(servo=fail_once_then_delegate)
    latest = LatestVRFrame()
    clock = _AcceptanceClock()
    control = RobotControl(
        backend=adapter,
        latest=latest,
        clock=clock,
        recorder=NoopRecorder(),
        constraint_clear_ms=100,
    )

    await control.connect()
    latest.publish(_frame(1, grip=False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(_frame(2, grip=True), clock.now_ns())
    await control.tick()
    clock.advance_ms(20)
    latest.publish(
        _frame(3, grip=True, position=(0.01, 1.2, -0.3)),
        clock.now_ns(),
    )
    await control.tick()
    constrained = await control.state_message()
    constraint_observed = (
        control.mode is TeleopMode.ACTIVE
        and constrained.constraint == "ik_boundary"
        and constrained.fault is None
    )

    for seq in range(4, 12):
        clock.advance_ms(20)
        latest.publish(
            _frame(seq, grip=True, position=(0.005, 1.2, -0.3)),
            clock.now_ns(),
        )
        await control.tick()
        adapter.robot.step(adapter.STEP_SECONDS)

    final = await control.state_message()
    constraint_cleared = final.constraint is None
    fault_stop_count = adapter.stop_counts[StopReason.FAULT]
    failures: list[str] = []
    if not constraint_observed:
        failures.append("constraint_not_observed")
    if not constraint_cleared:
        failures.append("constraint_not_cleared")
    if fault_stop_count:
        failures.append("unexpected_fault_stop")
    if control.mode is not TeleopMode.ACTIVE:
        failures.append("control_not_active")
    return ScenarioResult(
        name="soft_constraint_retreat",
        passed=not failures,
        metrics={
            "constraint_observed": constraint_observed,
            "constraint_cleared": constraint_cleared,
            "fault_stop_count": fault_stop_count,
            "final_mode": control.mode.value,
        },
        failures=tuple(failures),
    )


async def run_virtual_scenarios() -> tuple[ScenarioResult, ...]:
    return (
        run_mapping_scenario(),
        await run_servo_tracking_scenario(),
        await run_soft_constraint_scenario(),
    )
