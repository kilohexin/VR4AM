from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import yaml

from app.config import Settings


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
    "max_relative_translation_m": 0.02,
    "max_relative_rotation_deg": 5.0,
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
