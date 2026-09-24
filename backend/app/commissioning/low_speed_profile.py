from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path

import yaml

from app.config import REAL_ROBOT_CONFIRMATION, Settings


# Candidate limits for the first low-speed VR exercise. These are upper bounds,
# never replacements for tighter values already approved in an onsite config.
LOW_SPEED_CAPS: dict[str, float] = {
    "max_tcp_speed_mps": 0.005,
    "max_tcp_rotation_radps": 0.05,
    "max_tcp_acceleration_mps2": 0.02,
    "max_tcp_angular_acceleration_radps2": 0.1,
    "max_joint_speed_radps": 0.05,
    "max_joint_acceleration_radps2": 0.2,
    "max_tcp_step_m": 0.0005,
    "max_tcp_rotation_step_deg": 0.2,
    "translation_scale": 0.2,
}


def create_low_speed_profile(source: Path, output: Path) -> None:
    source = source.resolve()
    output = output.resolve()
    if source == output:
        raise ValueError("output_must_differ_from_source")
    original_bytes = source.read_bytes()
    payload = yaml.safe_load(original_bytes)
    if not isinstance(payload, dict) or payload.get("backend") != "lebai":
        raise ValueError("source_requires_lebai")
    real = payload.get("real_robot")
    if not isinstance(real, dict) or real.get("mode") != "readonly":
        raise ValueError("source_requires_readonly")
    Settings.load(source)
    control = real["control"]
    for key, cap in LOW_SPEED_CAPS.items():
        control[key] = min(control[key], cap)
    if source.read_bytes() != original_bytes:
        raise RuntimeError("source_changed_during_generation")
    with output.open("x", encoding="utf-8", newline="\n") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)


def verify_low_speed_profile(
    source: Path,
    candidate: Path,
    *,
    expected_mode: str = "readonly",
) -> dict[str, object]:
    if expected_mode not in {"readonly", "control"}:
        raise ValueError("invalid_expected_mode")
    if source.resolve() == candidate.resolve():
        raise ValueError("candidate_must_differ_from_source")
    source_bytes = source.read_bytes()
    candidate_bytes = candidate.read_bytes()
    original = yaml.safe_load(source_bytes)
    selected = yaml.safe_load(candidate_bytes)
    if not isinstance(original, dict) or original.get("backend") != "lebai":
        raise ValueError("source_requires_lebai")
    real = original.get("real_robot")
    if not isinstance(real, dict) or real.get("mode") != "readonly":
        raise ValueError("source_requires_readonly")
    if not isinstance(selected, dict):
        raise ValueError("invalid_candidate")
    selected_real = selected.get("real_robot")
    if not isinstance(selected_real, dict):
        raise ValueError("invalid_candidate")
    if selected_real.get("mode") != expected_mode:
        raise ValueError("candidate_mode_mismatch")
    Settings.load(source)
    selected_control = selected_real.get("control")
    if not isinstance(selected_control, dict):
        raise ValueError("invalid_candidate")
    expected = deepcopy(original)
    expected_real = expected["real_robot"]
    expected_real["mode"] = expected_mode
    expected_control = expected_real["control"]
    for key, cap in LOW_SPEED_CAPS.items():
        expected_limit = min(expected_control[key], cap)
        if selected_control.get(key) != expected_limit:
            raise ValueError(f"candidate_limit_mismatch:{key}")
        expected_control[key] = expected_limit
    if selected != expected:
        raise ValueError("candidate_unexpected_change")
    Settings.load(candidate)
    if source.read_bytes() != source_bytes or candidate.read_bytes() != candidate_bytes:
        raise RuntimeError("config_changed_during_verification")
    return {
        "mode": expected_mode,
        "source_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "candidate_sha256": hashlib.sha256(candidate_bytes).hexdigest(),
        "limits": {key: selected_control[key] for key in LOW_SPEED_CAPS},
    }


def create_low_speed_control_copy(
    source: Path,
    readonly_candidate: Path,
    output: Path,
) -> None:
    if (
        os.environ.get("VR4ARM_REAL_ROBOT_CONFIRM")
        != REAL_ROBOT_CONFIRMATION
    ):
        raise RuntimeError("real_robot_confirmation_required")
    if output.resolve() in {source.resolve(), readonly_candidate.resolve()}:
        raise ValueError("output_must_differ_from_inputs")
    verify_low_speed_profile(source, readonly_candidate)
    readonly_bytes = readonly_candidate.read_bytes()
    payload = yaml.safe_load(readonly_bytes)
    payload["real_robot"]["mode"] = "control"
    if readonly_candidate.read_bytes() != readonly_bytes:
        raise RuntimeError("candidate_changed_during_generation")
    with output.open("x", encoding="utf-8", newline="\n") as handle:
        yaml.safe_dump(payload, handle, sort_keys=False, allow_unicode=True)
    verify_low_speed_profile(source, output, expected_mode="control")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate a separate readonly low-speed VR candidate profile."
    )
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    create_low_speed_profile(args.source, args.output)
    output = args.output.resolve()
    print(json.dumps({
        "mode": "readonly",
        "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
        "output_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "output": str(output),
    }))
    return 0


def verification_main() -> int:
    parser = argparse.ArgumentParser(
        description="Verify a low-speed VR candidate without connecting to a robot."
    )
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument(
        "--expect-mode", choices=("readonly", "control"), default="readonly"
    )
    args = parser.parse_args()
    report = verify_low_speed_profile(
        args.source, args.candidate, expected_mode=args.expect_mode
    )
    print(json.dumps(report, sort_keys=True))
    return 0


def control_copy_main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a separate control-mode copy of a verified low-speed profile."
    )
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--readonly", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    create_low_speed_control_copy(args.source, args.readonly, args.output)
    print(json.dumps(verify_low_speed_profile(
        args.source, args.output, expected_mode="control"
    ), sort_keys=True))
    return 0
