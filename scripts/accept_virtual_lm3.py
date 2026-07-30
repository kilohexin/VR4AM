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
from collections.abc import Awaitable, Callable, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BACKEND = ROOT / "backend"
if str(BACKEND) not in sys.path:
    sys.path.insert(0, str(BACKEND))
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.acceptance.profile import (  # noqa: E402
    compare_profiles,
    onsite_profile,
    simulation_profile,
)
from app.acceptance.scenarios import (  # noqa: E402
    ScenarioResult,
    run_virtual_scenarios,
)
from app.config import Settings  # noqa: E402
from scripts.soak_simulator import run_soak  # noqa: E402


HARDWARE_PENDING = (
    "sdk_connection",
    "tcp_home_joint_limits",
    "gripper_direction_force",
    "pvat_tracking_latency",
    "stop_distance_estop",
    "physical_collision_load",
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
class AcceptanceReport:
    generated_at: str
    git: dict[str, object]
    model: dict[str, object]
    profile: dict[str, object]
    commands: tuple[CommandResult, ...]
    scenarios: tuple[ScenarioResult, ...]
    soak: dict[str, object]
    hardware_pending: tuple[str, ...]
    passed: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "generated_at": self.generated_at,
            "git": self.git,
            "model": self.model,
            "profile": self.profile,
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
            "scenarios": [
                scenario.to_dict() for scenario in self.scenarios
            ],
            "soak": self.soak,
            "hardware_verified": False,
            "hardware_pending": list(self.hardware_pending),
            "passed": self.passed,
        }


CommandRunner = Callable[[str, Sequence[str], Path], CommandResult]
ScenarioRunner = Callable[[], Awaitable[tuple[ScenarioResult, ...]]]
SoakRunner = Callable[..., dict[str, object]]


def parse_test_counts(output: str) -> tuple[int | None, int | None]:
    passed = [int(value) for value in re.findall(r"(\d+)\s+passed", output)]
    failed = [int(value) for value in re.findall(r"(\d+)\s+failed", output)]
    if not passed and not failed:
        return None, None
    return (
        passed[-1] if passed else 0,
        failed[-1] if failed else 0,
    )


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


def _git_provenance(repo_root: Path) -> dict[str, object]:
    status = _git_output(repo_root, "status", "--porcelain")
    dirty_paths = sorted(
        line[3:].strip()
        for line in status.splitlines()
        if len(line) >= 4
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
    return {
        "kinematics_path": str(kinematics.relative_to(repo_root)),
        "kinematics_sha256": _sha256(kinematics),
        "glb_path": str(glb.relative_to(repo_root)),
        "glb_sha256": _sha256(glb),
    }


def _profile_report(real_config: Path | None) -> dict[str, object]:
    simulation = simulation_profile()
    onsite = None
    if real_config is not None:
        settings = Settings.load(real_config)
        if settings.backend != "lebai" or settings.lebai is None:
            raise RuntimeError("real_config_requires_lebai")
        onsite = onsite_profile(settings.lebai)
    comparison = compare_profiles(simulation, onsite)
    return {
        "simulation": simulation.to_dict(),
        "onsite": None if onsite is None else onsite.to_dict(),
        "comparison": comparison.to_dict(),
    }


def run_gate(
    repo_root: Path,
    real_config: Path | None = None,
    *,
    command_runner: CommandRunner = run_command,
    scenario_runner: ScenarioRunner = run_virtual_scenarios,
    soak_runner: SoakRunner = run_soak,
) -> AcceptanceReport:
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
        command_runner(name, argv, cwd)
        for name, argv, cwd in command_specs
    )
    scenarios = asyncio.run(scenario_runner())
    soak = soak_runner(minutes=10.0, seed=42)
    passed = (
        all(command.returncode == 0 for command in commands)
        and all(scenario.passed for scenario in scenarios)
        and soak.get("error_count") == 0
    )
    return AcceptanceReport(
        generated_at=datetime.now(timezone.utc).isoformat(),
        git=_git_provenance(repo_root),
        model=_model_provenance(repo_root),
        profile=_profile_report(real_config),
        commands=commands,
        scenarios=scenarios,
        soak=soak,
        hardware_pending=HARDWARE_PENDING,
        passed=passed,
    )


def write_report(report: AcceptanceReport, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(
            report.to_dict(),
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)


def report_exit_code(report: AcceptanceReport) -> int:
    return 0 if report.passed else 1


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the non-hardware virtual LM3 acceptance gate"
    )
    parser.add_argument("--real-config", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/acceptance/virtual-lm3-latest.json"),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    real_config = (
        None
        if args.real_config is None
        else (
            args.real_config
            if args.real_config.is_absolute()
            else ROOT / args.real_config
        )
    )
    output = args.output if args.output.is_absolute() else ROOT / args.output
    try:
        report = run_gate(ROOT, real_config)
        write_report(report, output)
    except Exception as error:
        print(f"virtual LM3 acceptance failed: {error}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "passed": report.passed,
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
