from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import yaml
from scipy.spatial.transform import Rotation

from app.commissioning.actions import (
    GripperAction,
    HomeAction,
    PrepareAction,
    RotationAction,
    SmokeAction,
    SmokeOptions,
    SmokeResult,
    StopAction,
    TranslationAction,
)
from app.config import REAL_ROBOT_CONFIRMATION, Settings
from app.control.robot_control import LatestVRFrame, RobotControl
from app.main import _build_limiter, _build_mapper, build_backend
from app.recording.commissioning import CommissioningRecorder
from app.robots.base import HomeOptions, RobotBackend
from app.robots.lebai_adapter import ClientFactory
from app.robots.lebai_sdk_bridge import connect_real_client
from app.schemas.messages import (
    BackendState,
    ControllerState,
    RobotStateMessage,
    VRFrame,
)
from app.timebase import MonotonicClock

COMMISSIONING_LOG_ROOT = (
    Path(__file__).resolve().parents[3] / "logs" / "commissioning"
)
_IDENTITY_Q = (0.0, 0.0, 0.0, 1.0)
_ORIGIN = (0.0, 0.0, 0.0)
COMMISSIONING_MOTION_TIMEOUT_S = 4.0
TRANSLATION_TOLERANCE_M = 0.0005
ROTATION_TOLERANCE_DEG = 0.2


@dataclass(frozen=True)
class _MotionActionResult:
    reached_state: RobotStateMessage | None
    measurement_initial: RobotStateMessage | None
    max_cross_axis_drift_m: float | None


def parse_smoke_args(argv: Sequence[str] | None = None) -> SmokeOptions:
    parser = argparse.ArgumentParser(
        description="Run one guarded LM3 commissioning action.",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    _add_safety_inputs(commands.add_parser("translate"))
    translate = commands.choices["translate"]
    translate.add_argument("--axis", choices=("x", "y", "z"), required=True)
    translate.add_argument("--distance-m", type=float, required=True)

    _add_safety_inputs(commands.add_parser("rotate"))
    rotate = commands.choices["rotate"]
    rotate.add_argument(
        "--axis", choices=("roll", "pitch", "yaw"), required=True
    )
    rotate.add_argument("--angle-deg", type=float, required=True)

    _add_safety_inputs(commands.add_parser("gripper"))
    gripper = commands.choices["gripper"]
    gripper.add_argument("--target", choices=("open", "close"), required=True)

    _add_safety_inputs(commands.add_parser("home"))
    _add_safety_inputs(commands.add_parser("prepare"))
    _add_safety_inputs(commands.add_parser("stop"))
    args = parser.parse_args(argv)
    confirmation = str(args.confirm)
    if confirmation != REAL_ROBOT_CONFIRMATION:
        raise ValueError("smoke_confirmation_required")

    action: SmokeAction
    if args.command == "translate":
        distance = float(args.distance_m)
        if (
            not math.isfinite(distance)
            or distance == 0
            or abs(distance) > 0.005
        ):
            raise ValueError("smoke_translation_out_of_bounds")
        action = TranslationAction(args.axis, distance)
    elif args.command == "rotate":
        angle = float(args.angle_deg)
        if (
            not math.isfinite(angle)
            or angle == 0
            or abs(angle) > 2.0
        ):
            raise ValueError("smoke_rotation_out_of_bounds")
        action = RotationAction(args.axis, angle)
    elif args.command == "gripper":
        action = GripperAction(args.target)
    elif args.command == "home":
        action = HomeAction()
    elif args.command == "prepare":
        action = PrepareAction()
    else:
        action = StopAction()
    return SmokeOptions(Path(args.config), action, confirmation)


def _add_safety_inputs(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--confirm", required=True)


def _configured_mode(config_path: Path) -> object:
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("invalid_config")
    real = payload.get("real_robot")
    if not isinstance(real, dict):
        raise RuntimeError("invalid_config:real_robot")
    return real.get("mode")


def _action_name(action: SmokeAction) -> str:
    if isinstance(action, TranslationAction):
        return "translate"
    if isinstance(action, RotationAction):
        return "rotate"
    if isinstance(action, GripperAction):
        return "gripper"
    if isinstance(action, HomeAction):
        return "home"
    if isinstance(action, PrepareAction):
        return "prepare"
    return "stop"


def _validate_smoke_action(action: SmokeAction) -> None:
    if isinstance(action, TranslationAction):
        if (
            action.axis not in {"x", "y", "z"}
            or not math.isfinite(action.distance_m)
            or action.distance_m == 0
            or abs(action.distance_m) > 0.005
        ):
            raise ValueError("smoke_translation_out_of_bounds")
        return
    if isinstance(action, RotationAction):
        if (
            action.axis not in {"roll", "pitch", "yaw"}
            or not math.isfinite(action.angle_deg)
            or action.angle_deg == 0
            or abs(action.angle_deg) > 2.0
        ):
            raise ValueError("smoke_rotation_out_of_bounds")
        return
    if isinstance(action, GripperAction):
        if action.target not in {"open", "close"}:
            raise ValueError("smoke_gripper_target_invalid")
        return
    if isinstance(action, (PrepareAction, HomeAction, StopAction)):
        return
    raise ValueError("smoke_action_invalid")


def _frame(
    seq: int,
    *,
    grip: bool,
    p: tuple[float, float, float] = _ORIGIN,
    q: tuple[float, float, float, float] = _IDENTITY_Q,
    trigger: float = 0.0,
) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id="real-smoke",
        seq=seq,
        client_mono_ms=float(seq),
        tracking_valid=True,
        visibility="visible",
        right=ControllerState(p=p, q=q, grip=grip, trigger=trigger),
    )


async def _wait_for_stable_state(
    backend: RobotBackend,
    timeout_s: float,
) -> RobotStateMessage:
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout_s
    stable_since: float | None = None
    while True:
        state = await backend.get_state()
        now = loop.time()
        # RobotStateMessage intentionally does not expose joint velocity.  The
        # RealLebaiAdapter stop path has already verified qd <= 0.02 rad/s for
        # 300 ms; this post-action poll keeps the reported state idle and fresh.
        if state.robot_state is BackendState.IDLE:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= 0.3:
                return state
        else:
            stable_since = None
        if now >= deadline:
            raise RuntimeError("smoke_stability_timeout")
        await asyncio.sleep(min(0.02, deadline - now))


async def _run_motion_action(
    control: RobotControl,
    latest: LatestVRFrame,
    clock: MonotonicClock,
    settings: Settings,
    action: TranslationAction | RotationAction | GripperAction,
) -> _MotionActionResult:
    # The released sample is deliberately ticked before arming so the
    # RobotControl session guards remain the sole authority for motion.
    observed_trigger = await control.initialize_observed_gripper()
    latest.publish(
        _frame(1, grip=False, trigger=observed_trigger),
        clock.now_ns(),
    )
    await control.tick()
    await control.arm()

    latest.publish(
        _frame(2, grip=True, trigger=observed_trigger),
        clock.now_ns(),
    )
    await control.tick()

    if settings.lebai is None:
        raise RuntimeError("missing_lebai_settings")
    sequence = 3
    reached_state: RobotStateMessage | None = None
    measurement_initial: RobotStateMessage | None = None
    max_cross_axis_drift_m: float | None = None
    try:
        if isinstance(action, (TranslationAction, RotationAction)):
            initial = await control.backend.get_state()
            measurement_initial = initial
            target_frame = _commissioning_target_frame(
                action,
                settings,
                trigger=observed_trigger,
            )
            period_s = 1 / settings.lebai.control.loop_hz
            maximum_frames = max(
                1,
                math.ceil(COMMISSIONING_MOTION_TIMEOUT_S / period_s),
            )
            reached = False
            target_axis_reached = False
            cross_axes_converged_after_target = False
            for _ in range(maximum_frames):
                latest.publish(
                    target_frame.model_copy(
                        update={
                            "seq": sequence,
                            "client_mono_ms": float(sequence),
                        }
                    ),
                    clock.now_ns(),
                )
                await control.tick()
                sequence += 1
                await asyncio.sleep(period_s)
                current = await control.backend.get_state()
                displacement = _authoritative_action_displacement(
                    initial,
                    current,
                    action,
                )
                requested, tolerance = _requested_displacement(action)
                signed_progress = math.copysign(1.0, requested) * displacement
                if signed_progress < -tolerance:
                    raise RuntimeError("smoke_motion_wrong_direction")
                if signed_progress > abs(requested) + tolerance:
                    raise RuntimeError("smoke_motion_overshoot")
                target_axis_reached_now = (
                    abs(displacement - requested) <= tolerance
                )
                target_axis_reached = (
                    target_axis_reached or target_axis_reached_now
                )
                cross_axis_converged = True
                if isinstance(action, TranslationAction):
                    cross_axis_drift = _translation_cross_axis_drift(
                        initial,
                        current,
                        action,
                    )
                    max_cross_axis_drift_m = max(
                        max_cross_axis_drift_m or 0.0,
                        cross_axis_drift,
                    )
                    cross_axis_converged = (
                        cross_axis_drift <= TRANSLATION_TOLERANCE_M
                    )
                    if target_axis_reached and cross_axis_converged:
                        cross_axes_converged_after_target = True
                if target_axis_reached_now and cross_axis_converged:
                    reached = True
                    reached_state = current
                    break
            if not reached:
                if (
                    isinstance(action, TranslationAction)
                    and target_axis_reached
                    and not cross_axes_converged_after_target
                ):
                    raise RuntimeError("smoke_cross_axis_not_settled")
                raise RuntimeError("smoke_motion_timeout")
        else:
            await asyncio.sleep(1 / settings.lebai.gripper.command_hz)
            trigger = 0.0 if action.target == "open" else 1.0
            await control.command_gripper(trigger)
            sequence += 1
    finally:
        latest.publish(
            _frame(sequence, grip=False, trigger=observed_trigger),
            clock.now_ns(),
        )
        await control.tick()
        await control.disarm()
    return _MotionActionResult(
        reached_state=reached_state,
        measurement_initial=measurement_initial,
        max_cross_axis_drift_m=max_cross_axis_drift_m,
    )


def _commissioning_target_frame(
    action: TranslationAction | RotationAction,
    settings: Settings,
    *,
    trigger: float,
) -> VRFrame:
    if settings.lebai is None:
        raise RuntimeError("missing_lebai_settings")
    if isinstance(action, TranslationAction):
        hand_distance = action.distance_m / settings.lebai.control.translation_scale
        offset = [0.0, 0.0, 0.0]
        offset[{"x": 0, "y": 1, "z": 2}[action.axis]] = hand_distance
        return _frame(0, grip=True, p=tuple(offset), trigger=trigger)
    controller_axis = {"roll": "x", "pitch": "y", "yaw": "z"}[action.axis]
    rotation = tuple(
        float(value)
        for value in Rotation.from_euler(
            controller_axis,
            action.angle_deg,
            degrees=True,
        ).as_quat()
    )
    return _frame(0, grip=True, q=rotation, trigger=trigger)


def _authoritative_action_displacement(
    initial: RobotStateMessage,
    current: RobotStateMessage,
    action: TranslationAction | RotationAction,
) -> float:
    if isinstance(action, TranslationAction):
        index = {"x": 0, "y": 1, "z": 2}[action.axis]
        return current.actual_tcp.p[index] - initial.actual_tcp.p[index]
    relative = (
        Rotation.from_quat(current.actual_tcp.q)
        * Rotation.from_quat(initial.actual_tcp.q).inv()
    ).as_rotvec()
    index = {"roll": 0, "pitch": 1, "yaw": 2}[action.axis]
    return math.degrees(float(relative[index]))


def _translation_cross_axis_drift(
    initial: RobotStateMessage,
    current: RobotStateMessage,
    action: TranslationAction,
) -> float:
    commanded_index = {"x": 0, "y": 1, "z": 2}[action.axis]
    return max(
        abs(current.actual_tcp.p[index] - initial.actual_tcp.p[index])
        for index in range(3)
        if index != commanded_index
    )


def _requested_displacement(
    action: TranslationAction | RotationAction,
) -> tuple[float, float]:
    if isinstance(action, TranslationAction):
        return action.distance_m, TRANSLATION_TOLERANCE_M
    return action.angle_deg, ROTATION_TOLERANCE_DEG


async def run_smoke(
    options: SmokeOptions,
    client_factory: ClientFactory = connect_real_client,
) -> SmokeResult:
    if options.confirmation != REAL_ROBOT_CONFIRMATION:
        raise ValueError("smoke_confirmation_required")
    _validate_smoke_action(options.action)
    if _configured_mode(options.config_path) != "control":
        raise RuntimeError("smoke_requires_control_mode")

    previous_config = os.environ.get("VR4ARM_CONFIG")
    previous_confirmation = os.environ.get("VR4ARM_REAL_ROBOT_CONFIRM")
    os.environ["VR4ARM_CONFIG"] = str(options.config_path)
    os.environ["VR4ARM_REAL_ROBOT_CONFIRM"] = options.confirmation
    try:
        settings = Settings.load()
    finally:
        if previous_config is None:
            os.environ.pop("VR4ARM_CONFIG", None)
        else:
            os.environ["VR4ARM_CONFIG"] = previous_config
        if previous_confirmation is None:
            os.environ.pop("VR4ARM_REAL_ROBOT_CONFIRM", None)
        else:
            os.environ["VR4ARM_REAL_ROBOT_CONFIRM"] = previous_confirmation

    if settings.lebai is None or settings.lebai.mode != "control":
        raise RuntimeError("smoke_requires_control_mode")
    action_name = _action_name(options.action)
    recorder = CommissioningRecorder(
        COMMISSIONING_LOG_ROOT,
        metadata={
            "backend": "LEBAI",
            "real_robot_mode": "control",
            "workflow": "single_guarded_commissioning_action",
            "action": action_name,
            "python_version": platform.python_version(),
        },
    )
    backend = build_backend(settings, recorder, client_factory)
    latest = LatestVRFrame()
    clock = MonotonicClock()
    control: RobotControl | None = None
    recorder_started = False
    backend_connected = False
    primary_error: BaseException | None = None
    cleanup_error: BaseException | None = None
    try:
        await recorder.start()
        recorder_started = True
        await backend.connect()
        backend_connected = True
        preflight = await backend.preflight()
        await recorder.write_critical_event(
            "preflight_result",
            {
                "ready": preflight.ready,
                "reason": preflight.reason,
                "robot_state": preflight.robot_state.value,
                "tcp_matches": preflight.tcp_matches,
                "capabilities": list(preflight.capabilities),
            },
            clock.now_ns(),
        )
        if not preflight.ready:
            if not (
                isinstance(options.action, (PrepareAction, HomeAction))
                and preflight.reason == "singular_configuration"
            ):
                raise RuntimeError(
                    f"smoke_preflight_failed:{preflight.reason}"
                )

        home_options = HomeOptions(
            max_speed_radps=settings.home_joint_speed_radps,
            timeout_s=settings.home_timeout_s,
            position_tolerance_rad=settings.home_position_tolerance_rad,
            velocity_tolerance_radps=settings.home_velocity_tolerance_radps,
            stable_seconds=settings.home_stable_ms / 1000,
        )
        before = await backend.get_state()
        if isinstance(options.action, PrepareAction):
            await backend.prepare(home_options, lambda _phase: None)
            prepared = await backend.preflight()
            if not prepared.ready:
                raise RuntimeError(
                    f"smoke_prepare_failed:{prepared.reason}"
                )
            after = await _wait_for_stable_state(
                backend,
                settings.home_timeout_s,
            )
            result = SmokeResult(action_name, before, after, stable=True)
            await recorder.write_critical_event(
                "smoke_result",
                result.to_dict(),
                clock.now_ns(),
            )
            print(json.dumps(result.to_dict(), separators=(",", ":")))
            return result

        control = RobotControl(
            backend=backend,
            latest=latest,
            clock=clock,
            recorder=recorder,
            mapper=_build_mapper(settings),
            limiter=_build_limiter(settings, "LEBAI"),
            constraint_clear_ms=settings.constraint_clear_ms,
            home_options=home_options,
        )
        await control.connect()
        motion_result = _MotionActionResult(None, None, None)
        if isinstance(options.action, (TranslationAction, RotationAction, GripperAction)):
            motion_result = await _run_motion_action(
                control,
                latest,
                clock,
                settings,
                options.action,
            )
        elif isinstance(options.action, HomeAction):
            latest.publish(_frame(1, grip=False), clock.now_ns())
            home = await control.home()
            if not home.accepted:
                raise RuntimeError(f"smoke_home_failed:{home.reason}")
        else:
            await control.disarm()
        after = await _wait_for_stable_state(backend, settings.home_timeout_s)
        requested_displacement: float | None = None
        reached_displacement: float | None = None
        settled_displacement: float | None = None
        displacement_unit: str | None = None
        reached_cross_axis_drift_m: float | None = None
        settled_cross_axis_drift_m: float | None = None
        if isinstance(options.action, (TranslationAction, RotationAction)):
            measurement_initial = motion_result.measurement_initial or before
            requested_displacement, _ = _requested_displacement(options.action)
            if motion_result.reached_state is not None:
                reached_displacement = _authoritative_action_displacement(
                    measurement_initial,
                    motion_result.reached_state,
                    options.action,
                )
            settled_displacement = _authoritative_action_displacement(
                measurement_initial,
                after,
                options.action,
            )
            displacement_unit = (
                "m" if isinstance(options.action, TranslationAction) else "deg"
            )
            if isinstance(options.action, TranslationAction):
                if motion_result.reached_state is not None:
                    reached_cross_axis_drift_m = (
                        _translation_cross_axis_drift(
                            measurement_initial,
                            motion_result.reached_state,
                            options.action,
                        )
                    )
                settled_cross_axis_drift_m = _translation_cross_axis_drift(
                    measurement_initial,
                    after,
                    options.action,
                )
        result = SmokeResult(
            action_name,
            before,
            after,
            stable=True,
            requested_displacement=requested_displacement,
            reached_displacement=reached_displacement,
            settled_displacement=settled_displacement,
            displacement_unit=displacement_unit,
            reached_cross_axis_drift_m=reached_cross_axis_drift_m,
            settled_cross_axis_drift_m=settled_cross_axis_drift_m,
            max_cross_axis_drift_m=motion_result.max_cross_axis_drift_m,
        )
        await recorder.write_critical_event(
            "smoke_result",
            result.to_dict(),
            clock.now_ns(),
        )
        print(json.dumps(result.to_dict(), separators=(",", ":")))
        return result
    except BaseException as error:
        primary_error = error
        raise
    finally:
        if control is not None:
            try:
                await control.stop()
            except BaseException as error:
                cleanup_error = error
        if backend_connected:
            try:
                await backend.disconnect()
            except BaseException as error:
                if cleanup_error is None:
                    cleanup_error = error
        if recorder_started:
            try:
                await recorder.close()
            except BaseException as error:
                if cleanup_error is None:
                    cleanup_error = error
        if primary_error is None and cleanup_error is not None:
            raise cleanup_error


def main(argv: Sequence[str] | None = None) -> int:
    asyncio.run(run_smoke(parse_smoke_args(argv)))
    return 0
