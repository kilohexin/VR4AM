from __future__ import annotations

import asyncio
import math
from contextlib import asynccontextmanager, suppress
from dataclasses import dataclass
from typing import AsyncIterator, Callable

import numpy as np
from scipy.spatial.transform import Rotation

from app.commissioning.actions import (
    GripperAction,
    HomeAction,
    RotationAction,
    StopAction,
    TranslationAction,
)
from app.config import Settings
from app.control.robot_control import LatestVRFrame, RobotControl
from app.digital_twin.lebai_client import (
    DigitalTwinFaults,
    DigitalTwinLebaiClient,
)
from app.digital_twin.runtime import load_digital_twin_settings
from app.main import _build_limiter, _build_mapper
from app.recording.noop import NoopRecorder
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.lebai_sdk_bridge import LebaiClientProtocol
from app.schemas.messages import (
    ControllerState,
    Pose,
    RobotStateMessage,
    TeleopMode,
    VRFrame,
)


@dataclass(frozen=True)
class FakeLebaiScenarioResult:
    name: str
    passed: bool
    metrics: dict[str, object]
    failures: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "passed": self.passed,
            "metrics": self.metrics,
            "failures": list(self.failures),
        }


class FakeClock:
    def __init__(self) -> None:
        self.value_ns = 1_000_000_000

    def now_ns(self) -> int:
        return self.value_ns

    def advance(self, seconds: float) -> None:
        self.value_ns += round(seconds * 1_000_000_000)

    async def sleep(self, seconds: float) -> None:
        self.advance(seconds)
        await asyncio.sleep(0)


@dataclass
class Harness:
    settings: Settings
    clock: FakeClock
    client: DigitalTwinLebaiClient
    backend: RealLebaiAdapter
    control: RobotControl
    latest: LatestVRFrame
    published_states: list[RobotStateMessage]
    preserve_faults_on_cleanup: bool = False


@asynccontextmanager
async def fake_real_harness() -> AsyncIterator[Harness]:
    settings = load_digital_twin_settings()
    assert settings.lebai is not None
    clock = FakeClock()
    client = DigitalTwinLebaiClient.idle(
        settings.lebai,
        clock=clock.now_ns,
    )

    async def factory(_ip: str) -> LebaiClientProtocol:
        return client

    recorder = NoopRecorder()
    backend = RealLebaiAdapter(
        settings.lebai,
        client_factory=factory,
        clock=clock.now_ns,
        sleep=clock.sleep,
        event_callback=recorder.write_event,
        backend_label="LEBAI_FAKE",
    )
    latest = LatestVRFrame()
    control = RobotControl(
        backend=backend,
        latest=latest,
        clock=clock,
        recorder=recorder,
        mapper=_build_mapper(settings),
        limiter=_build_limiter(settings),
        constraint_clear_ms=settings.constraint_clear_ms,
    )
    harness = Harness(
        settings=settings,
        clock=clock,
        client=client,
        backend=backend,
        control=control,
        latest=latest,
        published_states=[],
    )
    await backend.connect()
    await control.connect()
    try:
        yield harness
    finally:
        if not harness.preserve_faults_on_cleanup:
            client.set_faults(DigitalTwinFaults())
        with suppress(BaseException):
            await control.stop()
        with suppress(BaseException):
            await backend.disconnect()


def _frame(
    seq: int,
    *,
    grip: bool,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    trigger: float = 0.0,
) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id="fake-real-acceptance",
        seq=seq,
        client_mono_ms=float(seq * 20),
        tracking_valid=True,
        visibility="visible",
        right=ControllerState(
            p=position,
            q=rotation,
            grip=grip,
            trigger=trigger,
        ),
    )


def _writes(harness: Harness) -> list[tuple[object, ...]]:
    return harness.client._write_calls


def _methods(harness: Harness) -> list[str]:
    return [str(call[0]) for call in _writes(harness)]


async def _yield_until(
    predicate: Callable[[], bool],
    *,
    cycles: int = 2_000,
) -> None:
    for _ in range(cycles):
        if predicate():
            return
        await asyncio.sleep(0)
    raise AssertionError("deterministic_condition_not_reached")


def _assert_finite_state(state: RobotStateMessage) -> None:
    numeric = [
        float(state.server_mono_ns),
        *state.actual_tcp.p,
        *state.actual_tcp.q,
        *state.actual_q,
        state.gripper,
    ]
    if state.sample_age_ms is not None:
        numeric.append(state.sample_age_ms)
    assert all(math.isfinite(value) for value in numeric)


async def _publish_state(harness: Harness) -> RobotStateMessage:
    state = await harness.control.state_message()
    _assert_finite_state(state)
    harness.published_states.append(state)
    return state


async def _prepare_active(
    harness: Harness,
    *,
    trigger: float | None = None,
) -> RobotStateMessage:
    observed = await harness.control.initialize_observed_gripper()
    command = observed if trigger is None else trigger
    harness.latest.publish(
        _frame(1, grip=False, trigger=command),
        harness.clock.now_ns(),
    )
    await harness.control.tick()
    await harness.control.arm()
    harness.latest.publish(
        _frame(2, grip=True, trigger=command),
        harness.clock.now_ns(),
    )
    await harness.control.tick()
    assert harness.control.mode is TeleopMode.ACTIVE
    return await _publish_state(harness)


def _motion_frame(
    action: TranslationAction | RotationAction,
    settings: Settings,
    seq: int,
) -> VRFrame:
    if isinstance(action, TranslationAction):
        assert settings.lebai is not None
        offset = [0.0, 0.0, 0.0]
        index = {"x": 0, "y": 1, "z": 2}[action.axis]
        offset[index] = (
            action.distance_m / settings.lebai.control.translation_scale
        )
        return _frame(seq, grip=True, position=tuple(offset))
    axis = {"roll": "x", "pitch": "y", "yaw": "z"}[action.axis]
    rotation = tuple(
        float(value)
        for value in Rotation.from_euler(
            axis,
            action.angle_deg,
            degrees=True,
        ).as_quat()
    )
    return _frame(seq, grip=True, rotation=rotation)


async def _release_and_disarm(harness: Harness, seq: int) -> None:
    stops_before = _methods(harness).count("stop_move")
    harness.latest.publish(
        _frame(seq, grip=False),
        harness.clock.now_ns(),
    )
    await harness.control.tick()
    assert _methods(harness).count("stop_move") > stops_before
    await harness.control.disarm()
    assert harness.control.mode is TeleopMode.DISARMED
    final = await _publish_state(harness)
    assert final.mode is TeleopMode.DISARMED


def _assert_q_safe(harness: Harness, state: RobotStateMessage) -> None:
    assert harness.settings.lebai is not None
    q = np.asarray(state.actual_q, dtype=float)
    assert np.all(np.isfinite(q))
    assert np.all(q >= np.asarray(harness.settings.lebai.soft_joint_min_rad))
    assert np.all(q <= np.asarray(harness.settings.lebai.soft_joint_max_rad))


async def _run_signed_motion(
    action: TranslationAction | RotationAction,
) -> tuple[float, float, int]:
    async with fake_real_harness() as harness:
        before = await _prepare_active(harness)
        initial_q = np.asarray(before.actual_q, dtype=float)
        pvat_before = _methods(harness).count("move_pvat")
        harness.latest.publish(
            _motion_frame(action, harness.settings, 3),
            harness.clock.now_ns(),
        )
        await harness.control.tick()
        await _yield_until(
            lambda: _methods(harness).count("move_pvat") > pvat_before
        )
        pvat_call = next(
            call
            for call in reversed(_writes(harness))
            if call[0] == "move_pvat"
        )
        target_q = np.asarray(pvat_call[1], dtype=float)
        immediate = await harness.client.get_kin_data()
        immediate_q = np.asarray(immediate["actual_joint_pose"], dtype=float)
        assert np.allclose(immediate_q, initial_q, atol=1e-12, rtol=0.0)
        assert not np.allclose(target_q, initial_q, atol=1e-12, rtol=0.0)

        assert harness.settings.lebai is not None
        harness.clock.advance(harness.settings.lebai.control.pvat_horizon_s)
        after = await _publish_state(harness)
        _assert_q_safe(harness, after)

        position_delta = np.asarray(after.actual_tcp.p) - np.asarray(
            before.actual_tcp.p
        )
        rotation_delta = (
            Rotation.from_quat(after.actual_tcp.q)
            * Rotation.from_quat(before.actual_tcp.q).inv()
        ).as_rotvec()
        if isinstance(action, TranslationAction):
            index = {"x": 0, "y": 1, "z": 2}[action.axis]
            commanded = float(position_delta[index])
            uncommanded = float(
                np.max(np.abs(np.delete(position_delta, index)))
            )
            assert math.copysign(1.0, commanded) == math.copysign(
                1.0, action.distance_m
            )
            assert abs(commanded) > 1e-9
            assert (
                uncommanded
                <= harness.settings.lebai.control.max_tcp_step_m + 1e-9
            )
        else:
            index = {"roll": 0, "pitch": 1, "yaw": 2}[action.axis]
            commanded = float(rotation_delta[index])
            uncommanded = float(
                np.max(np.abs(np.delete(rotation_delta, index)))
            )
            assert math.copysign(1.0, commanded) == math.copysign(
                1.0, action.angle_deg
            )
            assert abs(commanded) > 1e-9
            assert uncommanded <= math.radians(
                harness.settings.lebai.control.max_tcp_rotation_step_deg
            ) + 1e-9

        await _release_and_disarm(harness, 4)
        assert all(
            state.mode is not TeleopMode.FAULT
            for state in harness.published_states
        )
        return commanded, uncommanded, len(harness.published_states)


async def run_translation_scenario() -> FakeLebaiScenarioResult:
    axes: list[str] = []
    deltas: dict[str, float] = {}
    max_uncommanded = 0.0
    published_states = 0
    for axis in ("x", "y", "z"):
        for sign, label in ((1.0, "+"), (-1.0, "-")):
            action = TranslationAction(axis, sign * 0.005)
            assert abs(action.distance_m) == 0.005
            commanded, uncommanded, published = await _run_signed_motion(action)
            name = f"{label}{axis}"
            axes.append(name)
            deltas[name] = commanded
            max_uncommanded = max(max_uncommanded, uncommanded)
            published_states += published
    return FakeLebaiScenarioResult(
        name="translation",
        passed=True,
        metrics={
            "axes": axes,
            "requested_distance_m": 0.005,
            "actual_component_delta_m": deltas,
            "max_uncommanded_delta_m": max_uncommanded,
            "published_states": published_states,
        },
    )


async def run_rotation_scenario() -> FakeLebaiScenarioResult:
    axes: list[str] = []
    deltas: dict[str, float] = {}
    max_uncommanded = 0.0
    published_states = 0
    for axis in ("roll", "pitch", "yaw"):
        for sign, label in ((1.0, "+"), (-1.0, "-")):
            action = RotationAction(axis, sign * 2.0)
            assert abs(action.angle_deg) == 2.0
            commanded, uncommanded, published = await _run_signed_motion(action)
            name = f"{label}{axis}"
            axes.append(name)
            deltas[name] = math.degrees(commanded)
            max_uncommanded = max(max_uncommanded, uncommanded)
            published_states += published
    return FakeLebaiScenarioResult(
        name="rotation",
        passed=True,
        metrics={
            "axes": axes,
            "requested_angle_deg": 2.0,
            "actual_component_delta_deg": deltas,
            "max_uncommanded_delta_deg": math.degrees(max_uncommanded),
            "published_states": published_states,
        },
    )


async def _run_gripper_action(action: GripperAction) -> tuple[int, int]:
    async with fake_real_harness() as harness:
        assert harness.settings.lebai is not None
        gripper = harness.settings.lebai.gripper
        if action.target == "open":
            await harness.backend.set_gripper(1.0)
        await _prepare_active(harness)
        harness.clock.advance(1 / gripper.command_hz)
        await _publish_state(harness)
        writes_before = len(_writes(harness))
        trigger = 0.0 if action.target == "open" else 1.0
        harness.latest.publish(
            _frame(3, grip=True, trigger=trigger),
            harness.clock.now_ns(),
        )
        await harness.control.tick()
        claw_calls = [
            call
            for call in _writes(harness)[writes_before:]
            if call[0] == "set_claw"
        ]
        expected = (
            gripper.open_amplitude_percent
            if action.target == "open"
            else gripper.closed_amplitude_percent
        )
        assert claw_calls == [("set_claw", gripper.max_force_percent, expected)]
        assert int(claw_calls[0][1]) <= 30
        await _release_and_disarm(harness, 4)
        return int(claw_calls[0][1]), int(claw_calls[0][2])


async def _run_home_action() -> float:
    action = HomeAction()
    assert isinstance(action, HomeAction)
    async with fake_real_harness() as harness:
        harness.latest.publish(
            _frame(1, grip=False),
            harness.clock.now_ns(),
        )
        await harness.control.tick()
        result = await harness.control.home()
        assert result.accepted is True
        assert harness.control.mode is TeleopMode.DISARMED
        state = await _publish_state(harness)
        assert harness.settings.lebai is not None
        movej_calls = [
            call for call in _writes(harness) if call[0] == "movej"
        ]
        assert len(movej_calls) == 1
        assert movej_calls[0][1] == list(harness.settings.lebai.home_q)
        error = max(
            abs(actual - target)
            for actual, target in zip(
                state.actual_q,
                harness.settings.lebai.home_q,
                strict=True,
            )
        )
        assert error <= harness.settings.home_position_tolerance_rad
        return error


async def _run_repeated_stop_action() -> tuple[int, bool]:
    action = StopAction()
    assert isinstance(action, StopAction)
    async with fake_real_harness() as harness:
        await harness.control.disarm()
        await harness.control.disarm()
        before_final = len(_writes(harness))
        await harness.control.stop()
        await asyncio.sleep(0)
        final_writes = _writes(harness)[before_final:]
        assert final_writes and final_writes[0][0] == "stop_move"
        assert not any(call[0] in {"move_pvat", "movej"} for call in final_writes)
        assert harness.control.mode is TeleopMode.DISARMED
        return _methods(harness).count("stop_move"), True


async def run_gripper_home_stop_scenario() -> FakeLebaiScenarioResult:
    open_force, open_amplitude = await _run_gripper_action(
        GripperAction("open")
    )
    close_force, close_amplitude = await _run_gripper_action(
        GripperAction("close")
    )
    home_error = await _run_home_action()
    stop_count, no_motion_after_final_stop = await _run_repeated_stop_action()
    assert close_force <= 30
    return FakeLebaiScenarioResult(
        name="gripper_home_stop",
        passed=True,
        metrics={
            "actions": ["open", "close", "home", "stop"],
            "open_force_percent": open_force,
            "open_amplitude_percent": open_amplitude,
            "close_force_percent": close_force,
            "close_amplitude_percent": close_amplitude,
            "home_max_error_rad": home_error,
            "repeated_stop_count": stop_count,
            "no_motion_after_final_stop": no_motion_after_final_stop,
        },
    )


async def _publish_motion(
    harness: Harness,
    seq: int,
    distance_m: float,
) -> None:
    action = TranslationAction("x", distance_m)
    harness.latest.publish(
        _motion_frame(action, harness.settings, seq),
        harness.clock.now_ns(),
    )
    await harness.control.tick()


async def _run_ik_failure_case() -> tuple[int, int]:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        harness.client.set_faults(DigitalTwinFaults(ik_failure=True))
        for seq in range(3, 7):
            await _publish_motion(harness, seq, 0.001 * (seq - 2))
            expected = seq - 2
            await _yield_until(
                lambda: harness.backend._consecutive_ik_failures >= expected
            )
            await harness.backend._pump.stop()
            if seq == 4:
                soft = await _publish_state(harness)
                assert soft.constraint == "ik_boundary"
                assert soft.fault is None
            await harness.backend._pump.start()

        await _publish_motion(harness, 7, 0.005)
        await _yield_until(lambda: harness.backend.pump_fault is not None)
        assert str(harness.backend.pump_fault) == "ik_failure_persistent"
        await _publish_motion(harness, 8, 0.004)
        assert harness.control.mode is TeleopMode.FAULT
        assert harness.control._fault == "ik_failure_persistent"
        assert "stop_move" in _methods(harness)
        return 5, len(harness.published_states)


async def _run_pvat_failure_case() -> int:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        harness.client.set_faults(DigitalTwinFaults(pvat_failure=True))
        await _publish_motion(harness, 3, 0.005)
        await _yield_until(lambda: harness.backend.pump_fault is not None)
        assert str(harness.backend.pump_fault) == "sdk_call_failed:move_pvat"
        await _publish_motion(harness, 4, 0.004)
        assert harness.control.mode is TeleopMode.FAULT
        assert harness.control._fault == "sdk_call_failed:move_pvat"
        assert "stop_move" in _methods(harness)
        return len(harness.published_states)


async def _run_disconnect_case() -> int:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        harness.client.set_faults(DigitalTwinFaults(disconnect=True))
        harness.preserve_faults_on_cleanup = True
        writes_at_disconnect = len(_writes(harness))
        await _publish_motion(harness, 3, 0.005)
        await _yield_until(lambda: harness.backend.pump_fault is not None)
        assert str(harness.backend.pump_fault) == "robot_disconnected"
        with suppress(BackendCommandError):
            await _publish_motion(harness, 4, 0.004)
        assert harness.control.mode is TeleopMode.FAULT
        assert harness.control._fault == "robot_disconnected"
        assert len(_writes(harness)) == writes_at_disconnect
        return len(harness.published_states)


async def _run_stale_snapshot_case() -> int:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        harness.clock.advance(0.081)
        await _publish_motion(harness, 3, 0.005)
        assert harness.control._fault == "robot_state_stale"
        assert "stop_move" in _methods(harness)
        fault_state = await _publish_state(harness)
        assert fault_state.fault == "robot_state_stale"
        return len(harness.published_states)


async def _run_stop_failure_case() -> tuple[int, int]:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        pvat_before = _methods(harness).count("move_pvat")
        await _publish_motion(harness, 3, 0.005)
        await _yield_until(
            lambda: _methods(harness).count("move_pvat") > pvat_before
        )
        harness.client.set_faults(DigitalTwinFaults(stop_failure=True))
        with suppress(BackendCommandError):
            harness.latest.publish(
                _frame(4, grip=False),
                harness.clock.now_ns(),
            )
            await harness.control.tick()
        methods = _methods(harness)
        assert "stop_move" in methods
        assert "stop_sys" in methods
        assert harness.backend._latched_fault == "stop_incomplete"
        assert harness.control.mode is TeleopMode.FAULT
        kin_data = await harness.client.get_kin_data()
        actual_qd = np.asarray(kin_data["actual_joint_speed"], dtype=float)
        assert np.all(np.isfinite(actual_qd))
        assert np.allclose(actual_qd, 0.03, atol=0.0, rtol=0.0)
        assert np.min(actual_qd) > 0.02
        state = await _publish_state(harness)
        assert state.fault == "stop_incomplete"
        _assert_finite_state(state)
        return methods.count("stop_sys"), len(harness.published_states)


async def run_fault_scenario() -> FakeLebaiScenarioResult:
    ik_failures, ik_states = await _run_ik_failure_case()
    pvat_states = await _run_pvat_failure_case()
    disconnect_states = await _run_disconnect_case()
    stale_states = await _run_stale_snapshot_case()
    stop_sys_calls, stop_states = await _run_stop_failure_case()
    cases = [
        "ik_failure",
        "pvat_failure",
        "disconnect",
        "stale_snapshot",
        "stop_failure",
    ]
    assert len(cases) == 5
    return FakeLebaiScenarioResult(
        name="faults",
        passed=True,
        metrics={
            "cases": cases,
            "injected": 5,
            "verified": 5,
            "ik_failures_before_hard_fault": ik_failures,
            "stop_sys_calls": stop_sys_calls,
            "published_states": (
                ik_states
                + pvat_states
                + disconnect_states
                + stale_states
                + stop_states
            ),
        },
    )


async def run_fake_lebai_scenarios() -> tuple[FakeLebaiScenarioResult, ...]:
    return (
        await run_translation_scenario(),
        await run_rotation_scenario(),
        await run_gripper_home_stop_scenario(),
        await run_fault_scenario(),
    )
