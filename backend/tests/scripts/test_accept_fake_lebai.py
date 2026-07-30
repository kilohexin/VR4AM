from __future__ import annotations

import asyncio
import importlib.util
import json
import sys
from pathlib import Path


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
