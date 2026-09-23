from __future__ import annotations

import asyncio
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = ROOT / "scripts" / "accept_fake_lebai.py"

HARDWARE_PENDING = [
    "sdk_connection",
    "tcp_home_joint_limits",
    "translation_direction",
    "rotation_direction",
    "gripper_direction_force",
    "pvat_tracking_latency",
    "stop_distance_estop",
    "lightweight_grasp_release",
]


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "accept_fake_lebai_test_module",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise AssertionError("could not load Fake Lebai acceptance runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


async def _passing_scenarios(module):
    return (
        module.FakeLebaiScenarioResult(
            name="fake_real_path",
            metrics={"actions": ["translate", "stop"]},
        ),
    )


def _command_runner(module, *, failing: str | None = None):
    def run(name, argv, cwd):
        failed = name == failing
        return module.CommandResult(
            name=name,
            argv=tuple(argv),
            cwd=str(cwd),
            returncode=1 if failed else 0,
            duration_s=0.1,
            passed_count=0 if failed else 1,
            failed_count=1 if failed else 0,
            output_tail="1 failed" if failed else "1 passed",
        )

    return run


def test_run_gate_reports_the_fake_runtime_and_software_provenance(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """A wrong runtime boundary or missing artifact hash must fail this test."""
    module = _load_module()
    monkeypatch.setattr(
        module,
        "run_fake_lebai_scenarios",
        lambda: _passing_scenarios(module),
    )

    report = module.run_gate(ROOT, command_runner=_command_runner(module))
    payload = report.to_dict()
    output = tmp_path / "acceptance" / "report.json"
    module.write_report(report, output)

    assert payload["schema_version"] == 1
    assert payload["runtime"] == "LEBAI_FAKE"
    assert payload["digital_twin"] is True
    assert payload["hardware_verified"] is False
    assert payload["passed"] is True
    assert payload["hardware_pending"] == HARDWARE_PENDING
    assert [command["name"] for command in payload["commands"]] == [
        "backend_tests",
        "frontend_tests",
        "frontend_build",
    ]
    assert all(
        len(payload["model"][name]) == 64
        and payload["model"][name].islower()
        for name in (
            "kinematics_sha256",
            "glb_sha256",
            "fake_config_sha256",
        )
    )
    assert json.loads(output.read_text(encoding="utf-8")) == payload
    assert not output.with_suffix(".json.tmp").exists()


def test_backend_suite_collects_root_script_tests(monkeypatch) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "run_fake_lebai_scenarios",
        lambda: _passing_scenarios(module),
    )

    def command_runner(name, argv, cwd):
        if name == "backend_tests":
            assert cwd == ROOT
            assert tuple(argv[1:]) == (
                "-m", "pytest", "-c", "backend/pyproject.toml", "backend/tests", "-q",
            )
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

    module.run_gate(ROOT, command_runner=command_runner)


def test_command_failure_marks_report_failed_and_cli_returns_one(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """A nonzero child command must never produce a passing gate result."""
    module = _load_module()
    monkeypatch.setattr(
        module,
        "run_fake_lebai_scenarios",
        lambda: _passing_scenarios(module),
    )
    report = module.run_gate(
        ROOT,
        command_runner=_command_runner(module, failing="frontend_tests"),
    )
    monkeypatch.setattr(module, "run_gate", lambda _root: report)

    assert report.passed is False
    assert module.report_exit_code(report) == 1
    assert module.main(["--output", str(tmp_path / "failed.json")]) == 1


def test_git_porcelain_z_records_paths_with_spaces_and_rename_pairs(
    monkeypatch,
) -> None:
    """A status parser must retain both sides of a rename without mangling spaces."""
    module = _load_module()
    monkeypatch.setattr(
        module,
        "_git_output",
        lambda _root, *args: (
            "a" * 40
            if args == ("rev-parse", "HEAD")
            else " M docs/space name.md\0R  docs/new name.md\0docs/old name.md\0"
        ),
    )

    provenance = module._git_provenance(ROOT)

    assert provenance == {
        "commit": "a" * 40,
        "dirty": True,
        "dirty_paths": [
            "docs/new name.md",
            "docs/old name.md",
            "docs/space name.md",
        ],
    }


@pytest.mark.parametrize(
    ("git_args", "failure"),
    [
        (("status", "--porcelain=v1", "-z"), "returncode"),
        (("rev-parse", "HEAD"), "returncode"),
        (("status", "--porcelain=v1", "-z"), "exception"),
        (("rev-parse", "HEAD"), "exception"),
    ],
)
def test_git_failures_fail_the_gate_and_cli(
    monkeypatch,
    tmp_path: Path,
    git_args: tuple[str, ...],
    failure: str,
) -> None:
    """A failed Git subprocess must not be represented as clean provenance."""
    module = _load_module()
    monkeypatch.setattr(
        module,
        "run_fake_lebai_scenarios",
        lambda: _passing_scenarios(module),
    )

    def fake_subprocess(argv, **kwargs):
        del kwargs
        args = tuple(argv[1:])
        if args == git_args:
            if failure == "exception":
                raise OSError("git unavailable")
            return SimpleNamespace(
                returncode=1,
                stdout="",
                stderr="git failed",
            )
        if args == ("status", "--porcelain=v1", "-z"):
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args == ("rev-parse", "HEAD"):
            return SimpleNamespace(returncode=0, stdout="a" * 40, stderr="")
        raise AssertionError(f"unexpected git command: {argv}")

    monkeypatch.setattr(module.subprocess, "run", fake_subprocess)
    report = module.run_gate(ROOT, command_runner=_command_runner(module))
    original_run_gate = module.run_gate
    monkeypatch.setattr(
        module,
        "run_gate",
        lambda repo_root: original_run_gate(
            repo_root,
            command_runner=_command_runner(module),
        ),
    )

    assert report.passed is False
    assert report.to_dict()["git"]["dirty"] is None
    assert module.main(["--output", str(tmp_path / "git-failed.json")]) == 1


@pytest.mark.parametrize("failure_stage", ["scenario", "provenance"])
def test_cli_replaces_stale_success_report_when_gate_construction_fails(
    monkeypatch,
    tmp_path: Path,
    failure_stage: str,
) -> None:
    """A construction exception must atomically replace stale success JSON."""
    module = _load_module()
    output = tmp_path / "fake-lebai-latest.json"
    output.write_text(
        json.dumps({"schema_version": 1, "passed": True}),
        encoding="utf-8",
    )
    original_run_gate = module.run_gate

    if failure_stage == "scenario":
        async def failing_scenarios():
            raise RuntimeError("scenario exploded")

        monkeypatch.setattr(module, "run_fake_lebai_scenarios", failing_scenarios)
    else:
        monkeypatch.setattr(
            module,
            "_model_provenance",
            lambda _root: (_ for _ in ()).throw(
                RuntimeError("provenance exploded")
            ),
        )

    monkeypatch.setattr(
        module,
        "run_gate",
        lambda repo_root: original_run_gate(
            repo_root,
            command_runner=_command_runner(module),
        ),
    )

    assert module.main(["--output", str(output)]) == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == 1
    assert payload["passed"] is False
    assert payload["hardware_verified"] is False
    assert payload["failure"]["stage"] == failure_stage
    assert failure_stage in payload["failure"]["message"]
    assert not output.with_suffix(".json.tmp").exists()
