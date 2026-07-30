from __future__ import annotations

import argparse
import asyncio
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "backend"
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.acceptance.fake_lebai import (  # noqa: E402
    FakeLebaiScenarioResult,
    run_fake_lebai_scenarios,
)


HARDWARE_PENDING = (
    "sdk_connection",
    "tcp_home_joint_limits",
    "translation_direction",
    "rotation_direction",
    "gripper_direction_force",
    "pvat_tracking_latency",
    "stop_distance_estop",
    "lightweight_grasp_release",
)


@dataclass(frozen=True)
class CommandResult:
    name: str
    argv: tuple[str, ...]
    cwd: str
    returncode: int
    duration_s: float
    passed_count: int | None
    failed_count: int | None
    output_tail: str


@dataclass(frozen=True)
class FakeLebaiAcceptanceReport:
    generated_at: str
    git: dict[str, object]
    model: dict[str, object]
    commands: tuple[CommandResult, ...]
    scenarios: tuple[FakeLebaiScenarioResult, ...]
    hardware_pending: tuple[str, ...]
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "generated_at": self.generated_at,
            "runtime": "LEBAI_FAKE",
            "digital_twin": True,
            "git": self.git,
            "model": self.model,
            "commands": [
                {
                    "name": command.name,
                    "argv": list(command.argv),
                    "cwd": command.cwd,
                    "returncode": command.returncode,
                    "duration_s": command.duration_s,
                    "passed_count": command.passed_count,
                    "failed_count": command.failed_count,
                    "output_tail": command.output_tail,
                }
                for command in self.commands
            ],
            "scenarios": [scenario.to_dict() for scenario in self.scenarios],
            "hardware_verified": False,
            "hardware_pending": list(self.hardware_pending),
            "passed": self.passed,
        }


CommandRunner = Callable[[str, Sequence[str], Path], CommandResult]


def parse_test_counts(output: str) -> tuple[int | None, int | None]:
    passed = [int(value) for value in re.findall(r"(\d+)\s+passed", output)]
    failed = [int(value) for value in re.findall(r"(\d+)\s+failed", output)]
    if not passed and not failed:
        return None, None
    return passed[-1] if passed else 0, failed[-1] if failed else 0


def run_command(
    name: str,
    argv: Sequence[str],
    cwd: Path,
) -> CommandResult:
    started = time.perf_counter()
    completed = subprocess.run(
        list(argv),
        cwd=cwd,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    duration_s = time.perf_counter() - started
    output = "\n".join(
        part for part in (completed.stdout, completed.stderr) if part
    ).strip()
    passed_count, failed_count = parse_test_counts(output)
    return CommandResult(
        name=name,
        argv=tuple(str(value) for value in argv),
        cwd=str(cwd),
        returncode=completed.returncode,
        duration_s=duration_s,
        passed_count=passed_count,
        failed_count=failed_count,
        output_tail=output[-4000:],
    )


def _git_output(repo_root: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", *args],
        cwd=repo_root,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        return ""
    return completed.stdout.rstrip("\r\n")


def _parse_porcelain_z(status: str) -> list[str]:
    entries = status.split("\0")
    paths: list[str] = []
    index = 0
    while index < len(entries):
        entry = entries[index]
        index += 1
        if not entry:
            continue
        if len(entry) < 4:
            continue
        state = entry[:2]
        paths.append(entry[3:])
        if "R" in state or "C" in state:
            if index < len(entries) and entries[index]:
                paths.append(entries[index])
            index += 1
    return sorted(set(paths))


def _git_provenance(repo_root: Path) -> dict[str, object]:
    dirty_paths = _parse_porcelain_z(
        _git_output(repo_root, "status", "--porcelain=v1", "-z")
    )
    return {
        "commit": _git_output(repo_root, "rev-parse", "HEAD"),
        "dirty": bool(dirty_paths),
        "dirty_paths": dirty_paths,
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _model_provenance(repo_root: Path) -> dict[str, object]:
    kinematics = repo_root / "config" / "lm3_visual_kinematics_v1.json"
    glb = repo_root / "web" / "public" / "models" / "Lebai_LM3.glb"
    fake_config = repo_root / "config" / "fake-lebai.yaml"
    return {
        "kinematics_path": str(kinematics.relative_to(repo_root)),
        "kinematics_sha256": _sha256(kinematics),
        "glb_path": str(glb.relative_to(repo_root)),
        "glb_sha256": _sha256(glb),
        "fake_config_path": str(fake_config.relative_to(repo_root)),
        "fake_config_sha256": _sha256(fake_config),
    }


def run_gate(
    repo_root: Path,
    command_runner: CommandRunner = run_command,
) -> FakeLebaiAcceptanceReport:
    npm = "npm.cmd" if os.name == "nt" else "npm"
    command_specs = (
        (
            "backend_tests",
            (sys.executable, "-m", "pytest", "-q"),
            repo_root / "backend",
        ),
        (
            "frontend_tests",
            (npm, "test", "--", "--run"),
            repo_root / "web",
        ),
        (
            "frontend_build",
            (npm, "run", "build"),
            repo_root / "web",
        ),
    )
    commands = tuple(
        command_runner(name, argv, cwd) for name, argv, cwd in command_specs
    )
    scenarios = tuple(asyncio.run(run_fake_lebai_scenarios()))
    passed = all(command.returncode == 0 for command in commands) and all(
        scenario.passed for scenario in scenarios
    )
    return FakeLebaiAcceptanceReport(
        generated_at=datetime.now(timezone.utc).isoformat(),
        git=_git_provenance(repo_root),
        model=_model_provenance(repo_root),
        commands=commands,
        scenarios=scenarios,
        hardware_pending=HARDWARE_PENDING,
        passed=passed,
    )


def write_report(report: FakeLebaiAcceptanceReport, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f"{output.name}.tmp")
    temporary.write_text(
        json.dumps(report.to_dict(), ensure_ascii=False, indent=2, sort_keys=True)
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)


def report_exit_code(report: FakeLebaiAcceptanceReport) -> int:
    return 0 if report.passed else 1


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the offline Fake Lebai acceptance gate"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/acceptance/fake-lebai-latest.json"),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output = args.output if args.output.is_absolute() else ROOT / args.output
    try:
        report = run_gate(ROOT)
        write_report(report, output)
    except Exception as error:
        print(f"Fake Lebai acceptance failed: {error}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "passed": report.passed,
                "runtime": "LEBAI_FAKE",
                "digital_twin": True,
                "hardware_verified": False,
                "output": str(output),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return report_exit_code(report)


if __name__ == "__main__":
    raise SystemExit(main())
