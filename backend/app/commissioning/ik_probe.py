"""Bounded, zero-motion vendor IK diagnostics. Solutions are never executed."""
from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import math
import os
import time
from pathlib import Path

from scipy.spatial.transform import Rotation

from app.commissioning.preflight import _CaptureRecorder, _configured_mode, _package_version
from app.config import Settings
from app.robots.lebai_adapter import RealLebaiAdapter, ClientFactory, TELEOP_SINGULARITY_GUARD_RAD
from app.robots.lebai_codec import joint_vector, pose_to_lebai
from app.robots.lebai_sdk_bridge import connect_real_client
from app.schemas.messages import BackendState, Pose


def probe_targets(origin: Pose) -> list[tuple[str, Pose]]:
    targets = [("baseline", origin)]
    for index, axis in enumerate(("x", "y", "z")):
        for sign, label in ((1, "+"), (-1, "-")):
            position = list(origin.p)
            position[index] += sign * .002
            targets.append((label + axis, Pose(p=tuple(position), q=origin.q)))
    for index, axis in enumerate(("roll", "pitch", "yaw")):
        for sign, label in ((1, "+"), (-1, "-")):
            vector = [0., 0., 0.]
            vector[index] = math.radians(sign * .5)
            rotation = Rotation.from_rotvec(vector) * Rotation.from_quat(origin.q)
            targets.append((label + axis, Pose(p=origin.p, q=tuple(rotation.as_quat()))))
    return targets


async def run_ik_probe(
    config_path: Path,
    output_path: Path,
    client_factory: ClientFactory = connect_real_client,
) -> int:
    config_path, output_path = Path(config_path), Path(output_path)
    if _configured_mode(config_path) != "readonly":
        raise RuntimeError("ik_probe_requires_readonly")
    previous = os.environ.get("VR4ARM_CONFIG")
    os.environ["VR4ARM_CONFIG"] = str(config_path)
    try:
        settings = Settings.load()
    finally:
        if previous is None:
            os.environ.pop("VR4ARM_CONFIG", None)
        else:
            os.environ["VR4ARM_CONFIG"] = previous
    if settings.lebai is None or settings.lebai.mode != "readonly":
        raise RuntimeError("ik_probe_requires_readonly")
    robot = settings.lebai
    recorder = _CaptureRecorder()
    client = None

    async def capture_client(ip):
        nonlocal client
        client = await client_factory(ip)
        return client

    # Explicit readonly adapter: no RobotControl, pump submissions or motion API.
    backend = RealLebaiAdapter(robot, client_factory=capture_client, event_callback=recorder.write_event)
    report = {
        "schema_version": 1, "complete": False, "motion_authorized": False,
        "mode": "readonly", "error": None, "results": [],
        "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "sdk_version": _package_version("lebai-sdk-asyncio"),
        "frame": "robot_base", "translation_m": .002, "rotation_deg": .5,
        "fk_verified": False,
        "note": "IK solutions are diagnostic only; no collision/path validation or motion permission.",
    }
    connected = False
    baseline_q = None

    async def observe():
        preflight = await asyncio.wait_for(backend.preflight(), 2.)
        state = await asyncio.wait_for(backend.get_state(), 2.)
        if preflight.reason != "real_robot_readonly":
            raise RuntimeError("ik_probe_preflight:" + str(preflight.reason))
        if state.robot_state is not BackendState.IDLE or state.fault:
            raise RuntimeError("ik_probe_robot_not_idle")
        kin = recorder.kinematics or {}
        velocity = joint_vector(kin.get("actual_qd"), "probe_velocity")
        if max(abs(v) for v in velocity) > .02:
            raise RuntimeError("ik_probe_robot_not_stationary")
        if baseline_q is not None and max(abs(a-b) for a, b in zip(state.actual_q, baseline_q)) > .001:
            raise RuntimeError("ik_probe_pose_changed")
        return state

    try:
        await backend.connect()
        connected = True
        initial = await observe()
        baseline_q = initial.actual_q
        report["baseline"] = initial.model_dump(mode="json")
        report["baseline_kinematics"] = recorder.kinematics
        report["configured_ready_q"] = list(robot.teleop_ready_q)
        if client is None or not callable(getattr(client, "kinematics_inverse", None)):
            raise RuntimeError("ik_probe_capability_missing")
        for name, target in probe_targets(initial.actual_tcp):
            for seed_name, seed in (("actual", baseline_q), ("configured_ready", robot.teleop_ready_q)):
                await observe()
                row = {
                    "target_name": name, "target": target.model_dump(mode="json"),
                    "vendor_target": pose_to_lebai(target), "seed_name": seed_name,
                    "seed_q": list(seed), "status": "pending",
                }
                report["results"].append(row)
                started = time.monotonic_ns()
                try:
                    raw = await asyncio.wait_for(
                        client.kinematics_inverse(row["vendor_target"], list(seed)), .2,
                    )
                    if raw is None:
                        row["status"] = "no_solution"
                    else:
                        solution = joint_vector(list(raw), "probe_ik_solution")
                        delta = [q-a for q, a in zip(solution, baseline_q)]
                        margins = [min(q-lo, hi-q) for q, lo, hi in zip(
                            solution, robot.soft_joint_min_rad, robot.soft_joint_max_rad)]
                        row.update({
                            "status": "solution", "solution_q": list(solution),
                            "delta_from_actual_rad": delta, "max_abs_delta_rad": max(map(abs, delta)),
                            "joint_margin_rad": margins,
                            "inside_soft_limits_with_margin": min(margins) >= robot.joint_limit_margin_rad,
                            "j3_abs_deg": math.degrees(abs(solution[2])),
                            "j5_abs_deg": math.degrees(abs(solution[4])),
                            "passes_existing_singularity_guard": min(abs(solution[2]), abs(solution[4])) > TELEOP_SINGULARITY_GUARD_RAD,
                            "exceeds_single_step_limit": max(map(abs, delta)) > robot.control.max_joint_step_rad,
                        })
                except Exception as error:
                    row["status"] = "error"
                    row["error"] = f"{type(error).__name__}:{error}"
                    raise
                finally:
                    row["ik_latency_ms"] = (time.monotonic_ns() - started) / 1_000_000
                final = await observe()
                report["final"] = final.model_dump(mode="json")
        report["complete"] = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}:{error}"
    finally:
        if connected:
            await backend.disconnect()  # readonly: no stop or other robot writes
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    return 0 if report["complete"] else 2


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Read-only IK neighborhood probe; never sends robot motion.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    return asyncio.run(run_ik_probe(args.config, args.output))
