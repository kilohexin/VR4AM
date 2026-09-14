from __future__ import annotations

import argparse
import json
import os
import platform
from datetime import UTC, datetime
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path
from typing import Sequence

import yaml

from app.config import Settings
from app.main import build_backend
from app.recording.noop import NoopRecorder
from app.robots.lebai_adapter import ClientFactory
from app.robots.lebai_sdk_bridge import connect_real_client


class _CaptureRecorder(NoopRecorder):
    def __init__(self) -> None:
        self.kinematics: dict[str, object] | None = None

    async def write_event(self, event: object, server_mono_ns: int) -> None:
        if (
            isinstance(event, dict)
            and event.get("kind") == "robot_kinematics"
        ):
            self.kinematics = dict(event)
            self.kinematics["captured_ns"] = server_mono_ns


def _package_version(distribution: str) -> str | None:
    try:
        return version(distribution)
    except PackageNotFoundError:
        return None


def _configured_mode(config_path: Path) -> object:
    payload = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise RuntimeError("invalid_config")
    real = payload.get("real_robot")
    if not isinstance(real, dict):
        raise RuntimeError("invalid_config:real_robot")
    return real.get("mode")


async def run_preflight(
    config_path: Path,
    output_path: Path,
    client_factory: ClientFactory = connect_real_client,
) -> int:
    config_path = Path(config_path)
    output_path = Path(output_path)
    if _configured_mode(config_path) != "readonly":
        raise RuntimeError("preflight_requires_readonly_mode")

    previous_config = os.environ.get("VR4ARM_CONFIG")
    os.environ["VR4ARM_CONFIG"] = str(config_path)
    try:
        settings = Settings.load()
    finally:
        if previous_config is None:
            os.environ.pop("VR4ARM_CONFIG", None)
        else:
            os.environ["VR4ARM_CONFIG"] = previous_config

    recorder = _CaptureRecorder()
    backend = build_backend(settings, recorder, client_factory)
    connected = False
    started_utc = datetime.now(UTC).isoformat()
    try:
        await backend.connect()
        connected = True
        preflight = await backend.preflight()
        preflight_observation = dict(recorder.kinematics or {})
        state = await backend.get_state()
        kinematics = recorder.kinematics or {}
        report = {
            "started_utc": started_utc,
            "sampled_utc": datetime.now(UTC).isoformat(),
            # Preserve the two reads separately: state may change after preflight.
            # estop is the adapter's decoded SDK value, not a visual inspection.
            "preflight_observation": preflight_observation,
            "state_observation": {**kinematics, "fault": state.fault},
            "complete": bool(
                preflight.reason == "real_robot_readonly"
                and kinematics.get("kind") == "robot_kinematics"
            ),
            "python_version": platform.python_version(),
            "platform": platform.platform(),
            "backend_version": _package_version("vr4arm-backend") or "source",
            "lebai_sdk_version": _package_version("lebai-sdk-asyncio"),
            "backend": "LEBAI",
            "real_robot_mode": "readonly",
            "preflight_ready": preflight.ready,
            "preflight_reason": preflight.reason,
            "robot_state": preflight.robot_state.value,
            "tcp_matches": preflight.tcp_matches,
            "capabilities": list(preflight.capabilities),
            "actual_q": list(preflight.actual_q),
            "actual_qd": kinematics.get("actual_qd"),
            "actual_qdd": kinematics.get("actual_qdd"),
            "actual_tcp": preflight.actual_tcp.model_dump(mode="json"),
            "gripper": state.gripper,
            "sdk_latencies_ms": kinematics.get("sdk_latencies_ms"),
        }
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(
                report,
                ensure_ascii=False,
                allow_nan=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        return 0 if report["complete"] else 2
    finally:
        if connected:
            await backend.disconnect()


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read LM3 state and generate a zero-write preflight report.",
    )
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    import asyncio

    return asyncio.run(run_preflight(args.config, args.output))
