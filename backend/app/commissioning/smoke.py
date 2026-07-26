from __future__ import annotations

import argparse
import asyncio
import math
import os
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Sequence

import yaml

from app.config import REAL_ROBOT_CONFIRMATION, Settings
from app.control.robot_control import LatestVRFrame, RobotControl
from app.main import _build_limiter, _build_mapper, build_backend
from app.recording.commissioning import CommissioningRecorder
from app.robots.base import HomeOptions
from app.robots.lebai_adapter import ClientFactory
from app.robots.lebai_sdk_bridge import connect_real_client
from app.schemas.messages import ControllerState, VRFrame
from app.timebase import MonotonicClock

Axis = Literal["x", "y", "z"]
COMMISSIONING_LOG_ROOT = (
    Path(__file__).resolve().parents[3] / "logs" / "commissioning"
)


@dataclass(frozen=True)
class SmokeOptions:
    config_path: Path
    axis: Axis
    distance_m: float
    confirmation: str


def parse_smoke_args(argv: Sequence[str] | None = None) -> SmokeOptions:
    parser = argparse.ArgumentParser(
        description="Run one guarded LM3 Cartesian smoke move.",
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--axis", choices=("x", "y", "z"), required=True)
    parser.add_argument("--distance-m", type=float, default=0.005)
    parser.add_argument("--confirm", required=True)
    args = parser.parse_args(argv)
    distance = float(args.distance_m)
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError("smoke_distance_must_be_positive_finite")
    if distance > 0.005:
        raise ValueError("smoke_distance_exceeds_0.005_m")
    if args.confirm != REAL_ROBOT_CONFIRMATION:
        raise ValueError("smoke_confirmation_required")
    return SmokeOptions(
        config_path=args.config,
        axis=args.axis,
        distance_m=distance,
        confirmation=args.confirm,
    )


def _configured_mode(config_path: Path) -> object:
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("invalid_config")
    real = payload.get("real_robot")
    if not isinstance(real, dict):
        raise RuntimeError("invalid_config:real_robot")
    return real.get("mode")


def _frame(
    seq: int,
    *,
    grip: bool,
    p: tuple[float, float, float],
) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id="real-smoke",
        seq=seq,
        client_mono_ms=float(seq),
        tracking_valid=True,
        visibility="visible",
        right=ControllerState(
            p=p,
            q=(0.0, 0.0, 0.0, 1.0),
            grip=grip,
            trigger=0.0,
        ),
    )


async def run_smoke(
    options: SmokeOptions,
    client_factory: ClientFactory = connect_real_client,
) -> int:
    if options.confirmation != REAL_ROBOT_CONFIRMATION:
        raise ValueError("smoke_confirmation_required")
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
    recorder = CommissioningRecorder(
        COMMISSIONING_LOG_ROOT,
        metadata={
            "backend": "LEBAI",
            "real_robot_mode": "control",
            "workflow": "single_axis_5mm_smoke",
            "axis": options.axis,
            "distance_m": options.distance_m,
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
            raise RuntimeError(
                f"smoke_preflight_failed:{preflight.reason}"
            )

        control = RobotControl(
            backend=backend,
            latest=latest,
            clock=clock,
            recorder=recorder,
            mapper=_build_mapper(settings),
            limiter=_build_limiter(settings),
            constraint_clear_ms=settings.constraint_clear_ms,
            home_options=HomeOptions(
                max_speed_radps=settings.home_joint_speed_radps,
                timeout_s=settings.home_timeout_s,
                position_tolerance_rad=settings.home_position_tolerance_rad,
                velocity_tolerance_radps=settings.home_velocity_tolerance_radps,
                stable_seconds=settings.home_stable_ms / 1000,
            ),
        )
        await control.connect()
        origin = (0.0, 0.0, 0.0)
        latest.publish(_frame(1, grip=False, p=origin), clock.now_ns())
        await control.tick()
        await control.arm()
        latest.publish(_frame(2, grip=True, p=origin), clock.now_ns())
        await control.tick()

        hand_distance = (
            options.distance_m / settings.lebai.control.translation_scale
        )
        offset = [0.0, 0.0, 0.0]
        offset[{"x": 0, "y": 1, "z": 2}[options.axis]] = hand_distance
        latest.publish(
            _frame(3, grip=True, p=tuple(offset)),
            clock.now_ns(),
        )
        await control.tick()
        await asyncio.sleep(
            settings.lebai.control.pvat_horizon_s
            + 1 / settings.lebai.control.pvat_send_hz
        )
        latest.publish(_frame(4, grip=False, p=tuple(offset)), clock.now_ns())
        await control.tick()
        print(
            "Smoke move complete. Verify the actual direction before testing "
            "another axis."
        )
        return 0
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
    return asyncio.run(run_smoke(parse_smoke_args(argv)))
