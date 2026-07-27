from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

from app.acceptance.scenarios import ScenarioResult


ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = ROOT / "scripts" / "accept_virtual_lm3.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "accept_virtual_lm3_test_module",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise AssertionError("could not load acceptance runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


async def _passing_scenarios() -> tuple[ScenarioResult, ...]:
    return (
        ScenarioResult(
            name="fake_scenario",
            passed=True,
            metrics={"cycles": 1},
        ),
    )


def _passing_soak(*, minutes: float, seed: int) -> dict[str, object]:
    return {
        "simulated_minutes": minutes,
        "seed": seed,
        "control_steps": 30_000,
        "virtual_steps": 30_000,
        "error_count": 0,
        "nan_count": 0,
        "final_mode": "DISARMED",
    }


def test_run_gate_emits_complete_hardware_separated_report(
    tmp_path: Path,
) -> None:
    module = _load_module()

    def command_runner(name, argv, cwd):
        return module.CommandResult(
            name=name,
            argv=tuple(argv),
            cwd=str(cwd),
            returncode=0,
            duration_s=0.1,
            passed_count=1 if name != "frontend_build" else None,
            failed_count=0 if name != "frontend_build" else None,
            output_tail="1 passed",
        )

    report = module.run_gate(
        ROOT,
        command_runner=command_runner,
        scenario_runner=_passing_scenarios,
        soak_runner=_passing_soak,
    )
    payload = report.to_dict()
    output = tmp_path / "acceptance" / "report.json"
    module.write_report(report, output)

    assert set(payload) == {
        "schema_version",
        "generated_at",
        "git",
        "model",
        "profile",
        "commands",
        "scenarios",
        "soak",
        "hardware_verified",
        "hardware_pending",
        "passed",
    }
    assert payload["schema_version"] == 1
    assert payload["hardware_verified"] is False
    assert payload["passed"] is True
    assert len(payload["hardware_pending"]) == 6
    assert payload["profile"]["comparison"] == {
        "onsite_configured": False,
        "differences": [],
    }
    assert len(payload["model"]["kinematics_sha256"]) == 64
    assert len(payload["model"]["glb_sha256"]) == 64
    assert json.loads(output.read_text(encoding="utf-8")) == payload
    assert not output.with_suffix(".json.tmp").exists()


def test_command_failure_fails_gate_and_exit_code() -> None:
    module = _load_module()

    def command_runner(name, argv, cwd):
        failed = name == "backend_tests"
        return module.CommandResult(
            name=name,
            argv=tuple(argv),
            cwd=str(cwd),
            returncode=1 if failed else 0,
            duration_s=0.1,
            passed_count=323 if failed else 1,
            failed_count=1 if failed else 0,
            output_tail="1 failed, 323 passed" if failed else "1 passed",
        )

    report = module.run_gate(
        ROOT,
        command_runner=command_runner,
        scenario_runner=_passing_scenarios,
        soak_runner=_passing_soak,
    )

    assert report.passed is False
    assert module.report_exit_code(report) == 1


def test_scenario_or_soak_failure_fails_gate() -> None:
    module = _load_module()

    def command_runner(name, argv, cwd):
        return module.CommandResult(
            name=name,
            argv=tuple(argv),
            cwd=str(cwd),
            returncode=0,
            duration_s=0.1,
            passed_count=1,
            failed_count=0,
            output_tail="1 passed",
        )

    async def failing_scenarios():
        return (
            ScenarioResult(
                name="failed_scenario",
                passed=False,
                metrics={},
                failures=("failure",),
            ),
        )

    def failing_soak(*, minutes: float, seed: int):
        return {
            "simulated_minutes": minutes,
            "seed": seed,
            "error_count": 1,
        }

    scenario_report = module.run_gate(
        ROOT,
        command_runner=command_runner,
        scenario_runner=failing_scenarios,
        soak_runner=_passing_soak,
    )
    soak_report = module.run_gate(
        ROOT,
        command_runner=command_runner,
        scenario_runner=_passing_scenarios,
        soak_runner=failing_soak,
    )

    assert scenario_report.passed is False
    assert soak_report.passed is False


def test_parse_test_counts_supports_pytest_and_vitest() -> None:
    module = _load_module()

    assert module.parse_test_counts("324 passed in 41.23s") == (324, 0)
    assert module.parse_test_counts(
        "Tests  5 failed | 253 passed (258)"
    ) == (253, 5)
    assert module.parse_test_counts("built in 1.07s") == (None, None)


def test_git_provenance_preserves_first_dirty_path_character(
    monkeypatch,
) -> None:
    module = _load_module()

    def fake_run(argv, **kwargs):
        del kwargs
        if argv[-2:] == ["status", "--porcelain"]:
            return SimpleNamespace(
                returncode=0,
                stdout=" M README.md\n?? docs/new.md\n",
            )
        if argv[-2:] == ["rev-parse", "HEAD"]:
            return SimpleNamespace(returncode=0, stdout="a" * 40 + "\n")
        raise AssertionError(f"unexpected git command: {argv}")

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    provenance = module._git_provenance(ROOT)

    assert provenance["commit"] == "a" * 40
    assert provenance["dirty_paths"] == ["README.md", "docs/new.md"]
