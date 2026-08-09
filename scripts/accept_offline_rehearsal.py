from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
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
REHEARSAL_PHASES = (
    "identity_preflight",
    "home",
    "arm_and_anchor",
    "translate",
    "rotate",
    "gripper",
    "pick_place",
    "soft_boundary",
    "tracking_loss",
    "recovery_and_home",
    "final_stop",
    "finalize",
)
_HASH_NAMES = (
    "kinematics_sha256",
    "fake_config_sha256",
    "glb_sha256",
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

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "argv": list(self.argv),
            "cwd": self.cwd,
            "returncode": self.returncode,
            "duration_s": self.duration_s,
            "passed_count": self.passed_count,
            "failed_count": self.failed_count,
            "output_tail": self.output_tail,
        }


@dataclass(frozen=True)
class GateResult:
    name: str
    report_path: str | None
    errors: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "report_path": self.report_path,
            "passed": self.passed,
            "errors": list(self.errors),
        }


@dataclass(frozen=True)
class BrowserSmoke:
    status: str
    report_path: str | None
    reason: str | None

    def to_dict(self) -> dict[str, object]:
        return {
            "status": self.status,
            "report_path": self.report_path,
            "reason": self.reason,
        }


@dataclass(frozen=True)
class OfflineRehearsalAcceptanceReport:
    generated_at: str
    provenance: Mapping[str, object]
    commands: tuple[CommandResult, ...]
    gates: tuple[GateResult, ...]
    browser_smoke: BrowserSmoke

    @property
    def passed(self) -> bool:
        return all(gate.passed for gate in self.gates)

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": 1,
            "generated_at": self.generated_at,
            "runtime": "LEBAI_FAKE",
            "digital_twin": True,
            "hardware_verified": False,
            "hardware_pending": list(HARDWARE_PENDING),
            "provenance": dict(self.provenance),
            "commands": [command.to_dict() for command in self.commands],
            "gates": [gate.to_dict() for gate in self.gates],
            "browser_smoke": self.browser_smoke.to_dict(),
            "passed": self.passed,
        }


CommandRunner = Callable[[str, Sequence[str], Path], CommandResult]
ProvenanceProvider = Callable[[Path], Mapping[str, object]]
BrowserReportProvider = Callable[[Path], Path | None]


def parse_test_counts(output: str) -> tuple[int | None, int | None]:
    passed = [int(value) for value in re.findall(r"(\d+)\s+passed", output)]
    failed = [int(value) for value in re.findall(r"(\d+)\s+failed", output)]
    if not passed and not failed:
        return None, None
    return passed[-1] if passed else 0, failed[-1] if failed else 0


def run_command(name: str, argv: Sequence[str], cwd: Path) -> CommandResult:
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


def _safe_command(
    runner: CommandRunner,
    name: str,
    argv: Sequence[str],
    cwd: Path,
) -> CommandResult:
    try:
        return runner(name, argv, cwd)
    except Exception as error:
        return CommandResult(
            name=name,
            argv=tuple(str(value) for value in argv),
            cwd=str(cwd),
            returncode=-1,
            duration_s=0.0,
            passed_count=0,
            failed_count=1,
            output_tail=f"command_runner_error:{error}",
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
        detail = completed.stderr.strip() or completed.stdout.strip()
        raise RuntimeError(
            f"git_command_failed:{' '.join(args)}:{completed.returncode}:{detail}"
        )
    return completed.stdout.rstrip("\r\n")


def _git_provenance(repo_root: Path) -> dict[str, object]:
    status = _git_output(repo_root, "status", "--porcelain=v1", "-z")
    entries = [entry for entry in status.split("\0") if entry]
    dirty_paths = sorted(
        {entry[3:] for entry in entries if len(entry) >= 4}
    )
    return {
        "commit": _git_output(repo_root, "rev-parse", "HEAD"),
        "dirty": bool(dirty_paths),
        "dirty_paths": dirty_paths,
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _provenance(repo_root: Path) -> Mapping[str, object]:
    return {
        "git": _git_provenance(repo_root),
        "model": {
            "kinematics_sha256": _sha256(
                repo_root / "config" / "lm3_visual_kinematics_v1.json"
            ),
            "fake_config_sha256": _sha256(
                repo_root / "config" / "fake-lebai.yaml"
            ),
            "glb_sha256": _sha256(
                repo_root / "web" / "public" / "models" / "Lebai_LM3.glb"
            ),
        },
    }


def _relative_path(path: Path, repo_root: Path) -> str:
    try:
        return path.relative_to(repo_root).as_posix()
    except ValueError:
        return str(path)


def _load_report(path: Path) -> tuple[Mapping[str, object] | None, list[str]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None, ["report_missing"]
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        return None, [f"report_unreadable:{type(error).__name__}"]
    if not isinstance(payload, dict):
        return None, ["report_not_object"]
    return payload, []


def _valid_hash(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and re.fullmatch(r"[0-9a-f]{64}", value) is not None
    )


def _hash_errors(
    payload: Mapping[str, object],
    required: Sequence[str],
    current_model: Mapping[str, object],
) -> list[str]:
    model = payload.get("model")
    if not isinstance(model, dict):
        return ["model"]
    errors: list[str] = []
    for name in required:
        value = model.get(name)
        if not _valid_hash(value):
            errors.append(f"model.{name}")
        elif current_model.get(name) != value:
            errors.append(f"model.{name}:stale")
    return errors


def _scenario_errors(payload: Mapping[str, object]) -> list[str]:
    scenarios = payload.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        return ["scenarios"]
    if any(
        not isinstance(item, dict) or item.get("passed") is not True
        for item in scenarios
    ):
        return ["scenarios.passed"]
    return []


def _validate_virtual(
    payload: Mapping[str, object] | None,
    load_errors: Sequence[str],
    current_model: Mapping[str, object],
) -> list[str]:
    errors = list(load_errors)
    if payload is None:
        return errors
    if payload.get("passed") is not True:
        errors.append("passed")
    if payload.get("hardware_verified") is not False:
        errors.append("hardware_verified")
    errors.extend(
        _hash_errors(
            payload,
            ("kinematics_sha256", "glb_sha256"),
            current_model,
        )
    )
    errors.extend(_scenario_errors(payload))
    soak = payload.get("soak")
    expected_soak = {
        "control_steps": 30_000,
        "virtual_steps": 30_000,
        "injected_events": 7,
        "error_count": 0,
    }
    if not isinstance(soak, dict):
        errors.append("soak")
    else:
        for name, expected in expected_soak.items():
            if soak.get(name) != expected:
                errors.append(f"soak.{name}")
    performance = payload.get("soak_performance")
    if not isinstance(performance, dict) or performance.get("passed") is not True:
        errors.append("soak_performance.passed")
    return errors


def _validate_fake(
    payload: Mapping[str, object] | None,
    load_errors: Sequence[str],
    current_model: Mapping[str, object],
) -> list[str]:
    errors = list(load_errors)
    if payload is None:
        return errors
    for key, expected in (
        ("passed", True),
        ("runtime", "LEBAI_FAKE"),
        ("digital_twin", True),
        ("hardware_verified", False),
    ):
        if payload.get(key) != expected or type(payload.get(key)) is not type(expected):
            errors.append(key)
    if payload.get("hardware_pending") != list(HARDWARE_PENDING):
        errors.append("hardware_pending")
    errors.extend(_hash_errors(payload, _HASH_NAMES, current_model))
    errors.extend(_scenario_errors(payload))
    return errors


def _latest_browser_report(repo_root: Path) -> Path | None:
    root = repo_root / "artifacts" / "acceptance" / "offline-rehearsal"
    candidates = list(root.glob("*.json"))
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime_ns)


def _validate_browser_report(
    path: Path,
    repo_root: Path,
    current_model: Mapping[str, object],
) -> tuple[BrowserSmoke, list[str]]:
    payload, errors = _load_report(path)
    relative = _relative_path(path, repo_root)
    if payload is not None:
        for key, expected in (
            ("runtime", "LEBAI_FAKE"),
            ("hardware_verified", False),
            ("outcome", "passed"),
        ):
            if payload.get(key) != expected or type(payload.get(key)) is not type(expected):
                errors.append(key)
        if payload.get("hardware_pending") != list(HARDWARE_PENDING):
            errors.append("hardware_pending")
        errors.extend(_hash_errors(payload, _HASH_NAMES, current_model))
        phases = payload.get("phases")
        if not isinstance(phases, list):
            errors.append("phases")
        else:
            phase_names = [
                item.get("phase") if isinstance(item, dict) else None
                for item in phases
            ]
            if phase_names != list(REHEARSAL_PHASES):
                errors.append("phases.order")
            if any(
                not isinstance(item, dict) or item.get("status") != "passed"
                for item in phases
            ):
                errors.append("phases.status")
    if errors:
        return (
            BrowserSmoke(
                status="pending",
                report_path=relative,
                reason="invalid_browser_report:" + ",".join(errors),
            ),
            [f"browser_smoke:{error}" for error in errors],
        )
    return BrowserSmoke("done", relative, None), []


def run_gate(
    repo_root: Path,
    command_runner: CommandRunner = run_command,
    *,
    provenance_provider: ProvenanceProvider = _provenance,
    browser_report_provider: BrowserReportProvider = _latest_browser_report,
) -> OfflineRehearsalAcceptanceReport:
    npm = "npm.cmd" if os.name == "nt" else "npm"
    command_specs = (
        (
            "backend_rehearsal",
            (
                sys.executable,
                "-m",
                "pytest",
                "tests/rehearsal",
                "tests/api/test_teleop_ws.py",
                "-q",
            ),
            repo_root / "backend",
        ),
        (
            "frontend_rehearsal",
            (
                npm,
                "test",
                "--",
                "--run",
                "tests/offlineRehearsalController.test.ts",
                "tests/offlineRehearsalPanel.test.ts",
            ),
            repo_root / "web",
        ),
        (
            "frontend_build",
            (npm, "run", "build"),
            repo_root / "web",
        ),
    )
    commands = tuple(
        _safe_command(command_runner, name, argv, cwd)
        for name, argv, cwd in command_specs
    )

    provenance_errors: list[str] = []
    try:
        provenance = provenance_provider(repo_root)
        model = provenance.get("model")
        if not isinstance(model, dict):
            model = {}
            provenance_errors.append("provenance.model")
        else:
            for name in _HASH_NAMES:
                if not _valid_hash(model.get(name)):
                    provenance_errors.append(f"provenance.model.{name}")
    except Exception as error:
        provenance = {
            "git": {"commit": None, "dirty": None, "dirty_paths": []},
            "model": {},
            "failure": str(error),
        }
        model = {}
        provenance_errors.append("provenance")

    acceptance_root = repo_root / "artifacts" / "acceptance"
    virtual_path = acceptance_root / "virtual-lm3-latest.json"
    fake_path = acceptance_root / "fake-lebai-latest.json"
    virtual, virtual_load_errors = _load_report(virtual_path)
    fake, fake_load_errors = _load_report(fake_path)
    virtual_errors = _validate_virtual(virtual, virtual_load_errors, model)
    fake_errors = _validate_fake(fake, fake_load_errors, model)

    browser_errors: list[str] = []
    try:
        browser_path = browser_report_provider(repo_root)
    except Exception as error:
        browser_path = None
        browser_errors.append(f"browser_smoke:provider:{error}")
    if browser_path is None:
        browser_smoke = BrowserSmoke(
            "pending",
            None,
            "browser_smoke_not_run",
        )
    else:
        browser_smoke, validation_errors = _validate_browser_report(
            browser_path,
            repo_root,
            model,
        )
        browser_errors.extend(validation_errors)

    command_errors = [
        command.name for command in commands if command.returncode != 0
    ]
    gates = (
        GateResult(
            "Virtual",
            _relative_path(virtual_path, repo_root),
            tuple(virtual_errors),
        ),
        GateResult(
            "Fake Lebai",
            _relative_path(fake_path, repo_root),
            tuple(fake_errors),
        ),
        GateResult(
            "Offline Rehearsal",
            None,
            tuple(command_errors + provenance_errors + browser_errors),
        ),
    )
    return OfflineRehearsalAcceptanceReport(
        generated_at=datetime.now(timezone.utc).isoformat(),
        provenance=provenance,
        commands=commands,
        gates=gates,
        browser_smoke=browser_smoke,
    )


def write_report(
    report: OfflineRehearsalAcceptanceReport | Mapping[str, object],
    output: Path,
) -> None:
    payload = report.to_dict() if hasattr(report, "to_dict") else dict(report)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f"{output.name}.tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output)


def report_exit_code(report: OfflineRehearsalAcceptanceReport) -> int:
    return 0 if report.passed else 1


def _failure_payload(error: Exception) -> dict[str, object]:
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "runtime": "LEBAI_FAKE",
        "digital_twin": True,
        "hardware_verified": False,
        "hardware_pending": list(HARDWARE_PENDING),
        "commands": [],
        "gates": [],
        "browser_smoke": {
            "status": "pending",
            "report_path": None,
            "reason": "gate_construction_failed",
        },
        "passed": False,
        "failure": {"stage": "gate", "message": str(error)},
    }


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Aggregate the non-hardware offline rehearsal gates"
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("artifacts/acceptance/offline-rehearsal-latest.json"),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output = args.output if args.output.is_absolute() else ROOT / args.output
    try:
        report = run_gate(ROOT)
        write_report(report, output)
    except Exception as error:
        try:
            write_report(_failure_payload(error), output)
        except Exception as report_error:
            print(
                f"Offline rehearsal failure report could not be written: {report_error}",
                file=sys.stderr,
            )
        print(f"Offline rehearsal acceptance failed: {error}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "passed": report.passed,
                "runtime": "LEBAI_FAKE",
                "hardware_verified": False,
                "browser_smoke": report.browser_smoke.status,
                "output": str(output),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return report_exit_code(report)


if __name__ == "__main__":
    raise SystemExit(main())
