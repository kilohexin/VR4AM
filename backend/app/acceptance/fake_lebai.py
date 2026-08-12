from __future__ import annotations

import asyncio
import math
from contextlib import asynccontextmanager, suppress
from dataclasses import dataclass
from typing import AsyncIterator, Awaitable, Callable, TypeVar

import numpy as np
from scipy.spatial.transform import Rotation

from app.commissioning.actions import (
    GripperAction,
    HomeAction,
    RotationAction,
    StopAction,
    TranslationAction,
)
from app.commissioning.smoke import (
    COMMISSIONING_MOTION_TIMEOUT_S,
    ROTATION_TOLERANCE_DEG,
    TRANSLATION_TOLERANCE_M,
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

TRANSLATION_DOMINANCE_RATIO = 20.0
ROTATION_DOMINANCE_RATIO = 50.0
# Exercise all signed Cartesian axes away from the LM3 model's nearly singular
# nominal Home pose while staying inside the configured soft joint envelope.
ACCEPTANCE_START_Q = (0.1, -0.3, 2.0, -0.3, 1.5, -1.2)
T = TypeVar("T")


@dataclass(frozen=True)
class FakeLebaiScenarioResult:
    name: str
    metrics: dict[str, object]
    failures: tuple[str, ...] = ()

    @property
    def passed(self) -> bool:
        return not self.failures

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

    def now_seconds(self) -> float:
        return self.value_ns / 1_000_000_000

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
        initial_q=ACCEPTANCE_START_Q,
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
        pump_clock=clock.now_seconds,
        pump_sleep=clock.sleep,
    )
    latest = LatestVRFrame()
    control = RobotControl(
        backend=backend,
        latest=latest,
        clock=clock,
        recorder=recorder,
        mapper=_build_mapper(settings),
            limiter=_build_limiter(settings, "LEBAI_FAKE"),
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
    try:
        await backend.connect()
        await control.connect()
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


def _write_log_is_quiescent(
    final_log: tuple[tuple[object, ...], ...],
    observed_log: tuple[tuple[object, ...], ...],
) -> bool:
    return observed_log == final_log


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


async def _verify_case(
    label: str,
    operation: Callable[[], Awaitable[T]],
    failures: list[str],
) -> T | None:
    try:
        return await operation()
    except AssertionError as error:
        detail = str(error) or "verification_failed"
        failures.append(f"{label}:{detail}")
        return None


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


def _motion_quality_failures(
    action: TranslationAction | RotationAction,
    commanded: float,
    uncommanded: float,
) -> list[str]:
    magnitude = abs(commanded)
    if isinstance(action, TranslationAction):
        requested = abs(action.distance_m)
        tolerance = TRANSLATION_TOLERANCE_M
        dominance = TRANSLATION_DOMINANCE_RATIO
    else:
        magnitude = math.degrees(magnitude)
        uncommanded = math.degrees(uncommanded)
        requested = abs(action.angle_deg)
        tolerance = ROTATION_TOLERANCE_DEG
        dominance = ROTATION_DOMINANCE_RATIO
    failures: list[str] = []
    if commanded * (
        action.distance_m
        if isinstance(action, TranslationAction)
        else math.radians(action.angle_deg)
    ) <= 0:
        failures.append("wrong_direction")
    elif not requested - tolerance <= magnitude <= requested + tolerance:
        failures.append("magnitude_out_of_band")
    if magnitude < dominance * uncommanded:
        failures.append("commanded_axis_not_dominant")
    return failures


async def _run_signed_motion(
    action: TranslationAction | RotationAction,
) -> tuple[float, float, int]:
    async with fake_real_harness() as harness:
        before = await _prepare_active(harness)
        initial_q = np.asarray(before.actual_q, dtype=float)
        pvat_before = _methods(harness).count("move_pvat")
        assert harness.settings.lebai is not None
        period_s = 1 / harness.settings.lebai.control.loop_hz
        maximum_frames = max(
            1,
            math.ceil(COMMISSIONING_MOTION_TIMEOUT_S / period_s),
        )
        after = before
        commanded = 0.0
        uncommanded = 0.0
        reached = False
        checked_first_pvat = False
        for sequence in range(3, 3 + maximum_frames):
            harness.latest.publish(
                _motion_frame(action, harness.settings, sequence),
                harness.clock.now_ns(),
            )
            await harness.control.tick()
            await harness.clock.sleep(period_s)
            if _methods(harness).count("move_pvat") == pvat_before:
                await asyncio.sleep(0)
                continue
            if (
                not checked_first_pvat
                and _methods(harness).count("move_pvat") == pvat_before + 1
            ):
                pvat_call = next(
                    call
                    for call in reversed(_writes(harness))
                    if call[0] == "move_pvat"
                )
                target_q = np.asarray(pvat_call[1], dtype=float)
                immediate = await harness.client.get_kin_data()
                immediate_q = np.asarray(
                    immediate["actual_joint_pose"],
                    dtype=float,
                )
                assert not np.allclose(
                    target_q,
                    initial_q,
                    atol=1e-12,
                    rtol=0.0,
                )
                assert not np.allclose(
                    immediate_q,
                    target_q,
                    atol=1e-12,
                    rtol=0.0,
                )
                checked_first_pvat = True

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
                requested = action.distance_m
                tolerance = TRANSLATION_TOLERANCE_M
            else:
                index = {"roll": 0, "pitch": 1, "yaw": 2}[action.axis]
                commanded = float(rotation_delta[index])
                uncommanded = float(
                    np.max(np.abs(np.delete(rotation_delta, index)))
                )
                requested = math.radians(action.angle_deg)
                tolerance = math.radians(ROTATION_TOLERANCE_DEG)
            signed_progress = math.copysign(1.0, requested) * commanded
            assert signed_progress >= -tolerance, "wrong_direction"
            assert signed_progress <= abs(requested) + tolerance, "overshoot"
            if abs(commanded - requested) <= tolerance:
                reached = True
                break
        assert reached, "motion_timeout"

        await _release_and_disarm(harness, sequence + 1)
        assert all(
            state.mode is not TeleopMode.FAULT
            for state in harness.published_states
        )
        return commanded, uncommanded, len(harness.published_states)


async def run_translation_scenario() -> FakeLebaiScenarioResult:
    failures: list[str] = []
    axes: list[str] = []
    deltas: dict[str, float] = {}
    max_uncommanded = 0.0
    published_states = 0
    for axis in ("x", "y", "z"):
        for sign, label in ((1.0, "+"), (-1.0, "-")):
            action = TranslationAction(axis, sign * 0.005)
            assert abs(action.distance_m) == 0.005
            name = f"{label}{axis}"
            axes.append(name)
            result = await _verify_case(
                name,
                lambda action=action: _run_signed_motion(action),
                failures,
            )
            if result is None:
                continue
            commanded, uncommanded, published = result
            failures.extend(
                f"{name}:{failure}"
                for failure in _motion_quality_failures(
                    action,
                    commanded,
                    uncommanded,
                )
            )
            deltas[name] = commanded
            max_uncommanded = max(max_uncommanded, uncommanded)
            published_states += published
    return FakeLebaiScenarioResult(
        name="translation",
        metrics={
            "axes": axes,
            "requested_distance_m": 0.005,
            "actual_component_delta_m": deltas,
            "max_uncommanded_delta_m": max_uncommanded,
            "published_states": published_states,
        },
        failures=tuple(failures),
    )


async def run_rotation_scenario() -> FakeLebaiScenarioResult:
    failures: list[str] = []
    axes: list[str] = []
    deltas: dict[str, float] = {}
    max_uncommanded = 0.0
    published_states = 0
    for axis in ("roll", "pitch", "yaw"):
        for sign, label in ((1.0, "+"), (-1.0, "-")):
            action = RotationAction(axis, sign * 2.0)
            assert abs(action.angle_deg) == 2.0
            name = f"{label}{axis}"
            axes.append(name)
            result = await _verify_case(
                name,
                lambda action=action: _run_signed_motion(action),
                failures,
            )
            if result is None:
                continue
            commanded, uncommanded, published = result
            failures.extend(
                f"{name}:{failure}"
                for failure in _motion_quality_failures(
                    action,
                    commanded,
                    uncommanded,
                )
            )
            deltas[name] = math.degrees(commanded)
            max_uncommanded = max(max_uncommanded, uncommanded)
            published_states += published
    return FakeLebaiScenarioResult(
        name="rotation",
        metrics={
            "axes": axes,
            "requested_angle_deg": 2.0,
            "actual_component_delta_deg": deltas,
            "max_uncommanded_delta_deg": math.degrees(max_uncommanded),
            "published_states": published_states,
        },
        failures=tuple(failures),
    )


async def _run_gripper_action(action: GripperAction) -> tuple[int, int]:
    async with fake_real_harness() as harness:
        assert harness.settings.lebai is not None
        gripper = harness.settings.lebai.gripper
        await _prepare_active(harness)
        harness.clock.advance(1 / gripper.command_hz)
        await _publish_state(harness)
        writes_before = len(_writes(harness))
        trigger = 0.0 if action.target == "open" else 1.0
        await harness.control.command_gripper(trigger)
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
        final_writes = _writes(harness)[before_final:]
        assert final_writes and final_writes[0][0] == "stop_move"
        assert not any(call[0] in {"move_pvat", "movej"} for call in final_writes)
        assert harness.control.mode is TeleopMode.DISARMED
        final_log = tuple(_writes(harness))
        assert harness.settings.lebai is not None
        await harness.clock.sleep(
            2 / harness.settings.lebai.control.pvat_send_hz
        )
        no_writes_after_final_stop = _write_log_is_quiescent(
            final_log,
            tuple(_writes(harness)),
        )
        assert no_writes_after_final_stop
        return (
            _methods(harness).count("stop_move"),
            no_writes_after_final_stop,
        )


async def run_gripper_home_stop_scenario() -> FakeLebaiScenarioResult:
    failures: list[str] = []
    open_result = await _verify_case(
        "open",
        lambda: _run_gripper_action(GripperAction("open")),
        failures,
    )
    close_result = await _verify_case(
        "close",
        lambda: _run_gripper_action(GripperAction("close")),
        failures,
    )
    home_error = await _verify_case(
        "home",
        _run_home_action,
        failures,
    )
    stop_result = await _verify_case(
        "stop",
        _run_repeated_stop_action,
        failures,
    )
    open_force, open_amplitude = open_result or (None, None)
    close_force, close_amplitude = close_result or (None, None)
    stop_count, no_writes_after_final_stop = stop_result or (0, False)
    return FakeLebaiScenarioResult(
        name="gripper_home_stop",
        metrics={
            "actions": ["open", "close", "home", "stop"],
            "open_force_percent": open_force,
            "open_amplitude_percent": open_amplitude,
            "close_force_percent": close_force,
            "close_amplitude_percent": close_amplitude,
            "home_max_error_rad": home_error,
            "repeated_stop_count": stop_count,
            "no_writes_after_final_stop": no_writes_after_final_stop,
        },
        failures=tuple(failures),
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
        assert harness.settings.lebai is not None
        harness.client.set_faults(DigitalTwinFaults(ik_failure=True))
        period_ns = round(
            1_000_000_000 / harness.settings.lebai.control.pvat_send_hz
        )
        verified_failures = 0
        for failure_index, seq in enumerate(range(3, 9), start=1):
            started_ns = harness.clock.now_ns()
            await _publish_motion(harness, seq, 0.001 * (seq - 2))
            await _yield_until(
                lambda: (
                    harness.backend.constraint == "ik_boundary"
                    and harness.clock.now_ns() >= started_ns + period_ns
                )
            )
            verified_failures += 1
            if failure_index == 2:
                soft = await _publish_state(harness)
                assert soft.constraint == "ik_boundary"
                assert soft.fault is None
        assert harness.backend.pump_fault is None
        assert harness.backend._pump.running is True
        assert harness.control.mode is TeleopMode.ACTIVE

        harness.client.set_faults(DigitalTwinFaults())
        pvat_before = _methods(harness).count("move_pvat")
        await _publish_motion(harness, 9, 0.004)
        await _yield_until(
            lambda: _methods(harness).count("move_pvat") > pvat_before
        )
        recovered = await _publish_state(harness)
        assert recovered.fault is None
        assert harness.backend.pump_fault is None
        return verified_failures, len(harness.published_states)


async def _run_pvat_failure_case() -> int:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        harness.client.set_faults(DigitalTwinFaults(pvat_failure=True))
        await _publish_motion(harness, 3, 0.005)
        await _yield_until(lambda: harness.backend.pump_fault is not None)
        assert str(harness.backend.pump_fault) == "sdk_call_failed:move_pvat"
        await _publish_motion(harness, 4, 0.004)
        assert harness.control.mode is TeleopMode.FAULT
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
        assert len(_writes(harness)) == writes_at_disconnect
        return len(harness.published_states)


async def _run_stale_snapshot_case() -> int:
    async with fake_real_harness() as harness:
        await _prepare_active(harness)
        harness.clock.advance(0.081)
        await _publish_motion(harness, 3, 0.005)
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
    failures: list[str] = []
    case_operations: list[
        tuple[str, Callable[[], Awaitable[object]]]
    ] = [
        ("ik_failure", _run_ik_failure_case),
        ("pvat_failure", _run_pvat_failure_case),
        ("disconnect", _run_disconnect_case),
        ("stale_snapshot", _run_stale_snapshot_case),
        ("stop_failure", _run_stop_failure_case),
    ]
    outcomes: dict[str, object] = {}
    for name, operation in case_operations:
        outcome = await _verify_case(name, operation, failures)
        if outcome is not None:
            outcomes[name] = outcome
    ik_failures, ik_states = outcomes.get("ik_failure", (0, 0))
    pvat_states = outcomes.get("pvat_failure", 0)
    disconnect_states = outcomes.get("disconnect", 0)
    stale_states = outcomes.get("stale_snapshot", 0)
    stop_sys_calls, stop_states = outcomes.get("stop_failure", (0, 0))
    cases = [name for name, _operation in case_operations]
    return FakeLebaiScenarioResult(
        name="faults",
        metrics={
            "cases": cases,
            "injected": len(case_operations),
            "verified": len(outcomes),
            "recoverable_ik_failures": ik_failures,
            "stop_sys_calls": stop_sys_calls,
            "published_states": (
                ik_states
                + pvat_states
                + disconnect_states
                + stale_states
                + stop_states
            ),
        },
        failures=tuple(failures),
    )


async def run_fake_lebai_scenarios() -> tuple[FakeLebaiScenarioResult, ...]:
    return (
        await run_translation_scenario(),
        await run_rotation_scenario(),
        await run_gripper_home_stop_scenario(),
        await run_fault_scenario(),
    )
