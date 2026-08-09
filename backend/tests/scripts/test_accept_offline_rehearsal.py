from __future__ import annotations

import importlib.util
import json
import os
import signal
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = ROOT / "scripts" / "accept_offline_rehearsal.py"

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
REHEARSAL_PHASES = [
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
]


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "accept_offline_rehearsal_test_module",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise AssertionError("could not load offline rehearsal acceptance runner")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_input_reports(root: Path) -> tuple[Path, Path]:
    acceptance = root / "artifacts" / "acceptance"
    acceptance.mkdir(parents=True)
    virtual_path = acceptance / "virtual-lm3-latest.json"
    fake_path = acceptance / "fake-lebai-latest.json"
    virtual_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "passed": True,
                "hardware_verified": False,
                "model": {
                    "kinematics_sha256": "a" * 64,
                    "glb_sha256": "b" * 64,
                },
                "scenarios": [{"name": "all", "passed": True}],
                "soak": {
                    "control_steps": 30_000,
                    "virtual_steps": 30_000,
                    "injected_events": 7,
                    "error_count": 0,
                },
                "soak_performance": {"passed": True, "warning": False},
            }
        ),
        encoding="utf-8",
    )
    fake_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "passed": True,
                "runtime": "LEBAI_FAKE",
                "digital_twin": True,
                "hardware_verified": False,
                "hardware_pending": HARDWARE_PENDING,
                "model": {
                    "kinematics_sha256": "a" * 64,
                    "fake_config_sha256": "c" * 64,
                    "glb_sha256": "b" * 64,
                },
                "scenarios": [{"name": "all", "passed": True}],
            }
        ),
        encoding="utf-8",
    )
    return virtual_path, fake_path


def _passing_runner(module, calls: list[tuple[str, tuple[str, ...], Path]]):
    def run(name, argv, cwd):
        calls.append((name, tuple(argv), cwd))
        return module.CommandResult(
            name=name,
            argv=tuple(argv),
            cwd=str(cwd),
            returncode=0,
            duration_s=0.1,
            passed_count=3 if name != "frontend_build" else None,
            failed_count=0 if name != "frontend_build" else None,
            output_tail="3 passed",
        )

    return run


def test_run_command_times_out_as_a_truthful_failed_result(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """A hung aggregate subprocess must fail quickly and remain reportable."""
    module = _load_module()
    monkeypatch.setattr(module, "COMMAND_TIMEOUT_S", 0.05, raising=False)
    started = time.perf_counter()

    result = module.run_command(
        "hung_command",
        (sys.executable, "-c", "import time; time.sleep(0.4)"),
        tmp_path,
    )

    assert time.perf_counter() - started < 0.3
    assert result.returncode == 124
    assert result.passed_count == 0
    assert result.failed_count == 1
    assert "command_timeout:0.05s" in result.output_tail


def _pid_is_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def test_npm_timeout_kills_descendants_and_atomically_writes_failed_report(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """A stubborn npm/Node tree must not retain capture or stale success."""
    module = _load_module()
    _write_input_reports(tmp_path)
    npm_root = tmp_path / "npm-hang"
    npm_root.mkdir()
    pid_path = npm_root / "pids.json"
    package = {
        "name": "task8-timeout-probe",
        "private": True,
        "scripts": {"hang": "node hang.cjs"},
    }
    npm_root.joinpath("package.json").write_text(
        json.dumps(package),
        encoding="utf-8",
    )
    stubborn_child = (
        "process.on('SIGTERM',()=>{});"
        "process.on('SIGBREAK',()=>{});"
        "setTimeout(()=>process.exit(0),4000)"
    )
    npm_root.joinpath("hang.cjs").write_text(
        "const fs=require('fs');"
        "const {spawn}=require('child_process');"
        f"const child=spawn(process.execPath,['-e',{stubborn_child!r}],"
        "{stdio:'inherit'});"
        "fs.writeFileSync(process.env.TASK8_PID_FILE,"
        "JSON.stringify([process.ppid,process.pid,child.pid]));"
        "console.log('stubborn-start');"
        "console.error('stubborn-err');"
        "process.on('SIGTERM',()=>{});"
        "process.on('SIGBREAK',()=>{});"
        "setTimeout(()=>process.exit(0),4000);",
        encoding="utf-8",
    )
    monkeypatch.setenv("TASK8_PID_FILE", str(pid_path))
    monkeypatch.setattr(module, "COMMAND_TIMEOUT_S", 0.5)
    monkeypatch.setattr(module, "COMMAND_CLEANUP_TIMEOUT_S", 1.0, raising=False)
    real_spawn = module._spawn_command_tree
    readiness_waits: list[float] = []

    def spawn_after_fixture_ready(argv, cwd, stdout, stderr):
        tree = real_spawn(argv, cwd, stdout, stderr)
        readiness_started = time.perf_counter()
        readiness_deadline = readiness_started + 3.0
        while not pid_path.exists() and time.perf_counter() < readiness_deadline:
            time.sleep(0.01)
        readiness_waits.append(time.perf_counter() - readiness_started)
        return tree

    monkeypatch.setattr(module, "_spawn_command_tree", spawn_after_fixture_ready)
    npm = "npm.cmd" if os.name == "nt" else "npm"
    calls: list[tuple[str, tuple[str, ...], Path]] = []
    passing = _passing_runner(module, calls)

    def runner(name, argv, cwd):
        if name == "backend_rehearsal":
            return module.run_command(name, (npm, "run", "hang"), npm_root)
        return passing(name, argv, cwd)

    pids: list[int] = []
    started = time.perf_counter()
    try:
        report = module.run_gate(
            tmp_path,
            runner,
            provenance_provider=lambda _root: {
                "git": {"commit": "d" * 40, "dirty": False, "dirty_paths": []},
                "model": {
                    "kinematics_sha256": "a" * 64,
                    "fake_config_sha256": "c" * 64,
                    "glb_sha256": "b" * 64,
                },
            },
            browser_report_provider=lambda _root: None,
        )
        elapsed = time.perf_counter() - started
        assert pid_path.exists()
        assert readiness_waits and readiness_waits[0] < 3.0
        pids = json.loads(pid_path.read_text(encoding="utf-8"))
        assert elapsed < 4.5
        assert all(not _pid_is_alive(pid) for pid in pids)
        timed_out = report.commands[0]
        assert timed_out.returncode == 124
        assert timed_out.passed_count == 0
        assert timed_out.failed_count == 1
        assert "stubborn-start" in timed_out.output_tail
        assert "stubborn-err" in timed_out.output_tail
        assert "command_timeout:0.5s" in timed_out.output_tail
        assert report.passed is False
        assert report.to_dict()["hardware_verified"] is False

        output = tmp_path / "offline-rehearsal-latest.json"
        output.write_text(json.dumps({"passed": True}), encoding="utf-8")
        module.write_report(report, output)
        payload = json.loads(output.read_text(encoding="utf-8"))
        assert payload["passed"] is False
        assert payload["hardware_verified"] is False
        assert payload["gates"][2]["passed"] is False
        assert "backend_rehearsal" in payload["gates"][2]["errors"]
        assert not output.with_name(f"{output.name}.tmp").exists()
    finally:
        if pid_path.exists() and not pids:
            pids = json.loads(pid_path.read_text(encoding="utf-8"))
        for pid in reversed(pids):
            if _pid_is_alive(pid):
                try:
                    os.kill(pid, signal.SIGTERM)
                except OSError:
                    pass


def test_gate_composes_virtual_fake_and_offline_checks_without_claiming_browser(
    tmp_path: Path,
) -> None:
    """Targeted tests must not be mislabeled as a completed browser smoke."""
    module = _load_module()
    _write_input_reports(tmp_path)
    calls: list[tuple[str, tuple[str, ...], Path]] = []

    report = module.run_gate(
        tmp_path,
        _passing_runner(module, calls),
        provenance_provider=lambda _root: {
            "git": {"commit": "d" * 40, "dirty": False, "dirty_paths": []},
            "model": {
                "kinematics_sha256": "a" * 64,
                "fake_config_sha256": "c" * 64,
                "glb_sha256": "b" * 64,
            },
        },
        browser_report_provider=lambda _root: None,
    )
    payload = report.to_dict()
    npm = "npm.cmd" if os.name == "nt" else "npm"

    assert calls == [
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
            tmp_path / "backend",
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
            tmp_path / "web",
        ),
        (
            "frontend_build",
            (npm, "run", "build"),
            tmp_path / "web",
        ),
    ]
    assert [gate["name"] for gate in payload["gates"]] == [
        "Virtual",
        "Fake Lebai",
        "Offline Rehearsal",
    ]
    assert all(gate["passed"] for gate in payload["gates"])
    assert payload["browser_smoke"] == {
        "status": "pending",
        "report_path": None,
        "reason": "browser_smoke_not_run",
    }
    assert payload["hardware_verified"] is False
    assert payload["hardware_pending"] == HARDWARE_PENDING
    assert payload["passed"] is True


def test_browser_smoke_is_done_only_for_a_valid_authoritative_rehearsal_report(
    tmp_path: Path,
) -> None:
    """A malformed or failed browser report must never become smoke evidence."""
    module = _load_module()
    _write_input_reports(tmp_path)
    browser_path = (
        tmp_path / "artifacts" / "acceptance" / "offline-rehearsal" / "run.json"
    )
    browser_path.parent.mkdir(parents=True)
    browser_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "runtime": "LEBAI_FAKE",
                "hardware_verified": False,
                "hardware_pending": HARDWARE_PENDING,
                "outcome": "passed",
                "model": {
                    "kinematics_sha256": "a" * 64,
                    "fake_config_sha256": "c" * 64,
                    "glb_sha256": "b" * 64,
                },
                "phases": [
                    {"phase": phase, "status": "passed"}
                    for phase in REHEARSAL_PHASES
                ],
            }
        ),
        encoding="utf-8",
    )
    calls: list[tuple[str, tuple[str, ...], Path]] = []

    report = module.run_gate(
        tmp_path,
        _passing_runner(module, calls),
        provenance_provider=lambda _root: {
            "git": {"commit": "d" * 40, "dirty": False, "dirty_paths": []},
            "model": {
                "kinematics_sha256": "a" * 64,
                "fake_config_sha256": "c" * 64,
                "glb_sha256": "b" * 64,
            },
        },
        browser_report_provider=lambda _root: browser_path,
    )

    assert report.to_dict()["browser_smoke"] == {
        "status": "done",
        "report_path": "artifacts/acceptance/offline-rehearsal/run.json",
        "reason": None,
    }
    assert report.passed is True


def test_gate_fails_closed_on_command_hardware_or_full_soak_regression(
    tmp_path: Path,
) -> None:
    """Any failed command, hardware claim, or shortened soak must fail aggregation."""
    module = _load_module()
    virtual_path, fake_path = _write_input_reports(tmp_path)
    virtual = json.loads(virtual_path.read_text(encoding="utf-8"))
    virtual["soak"]["control_steps"] = 29_999
    virtual_path.write_text(json.dumps(virtual), encoding="utf-8")
    fake = json.loads(fake_path.read_text(encoding="utf-8"))
    fake["hardware_verified"] = True
    fake_path.write_text(json.dumps(fake), encoding="utf-8")
    calls: list[tuple[str, tuple[str, ...], Path]] = []
    passing = _passing_runner(module, calls)

    def failing_runner(name, argv, cwd):
        result = passing(name, argv, cwd)
        if name != "frontend_rehearsal":
            return result
        return module.CommandResult(
            **{**result.__dict__, "returncode": 1, "failed_count": 1}
        )

    report = module.run_gate(
        tmp_path,
        failing_runner,
        provenance_provider=lambda _root: {
            "git": {"commit": "d" * 40, "dirty": False, "dirty_paths": []},
            "model": {
                "kinematics_sha256": "a" * 64,
                "fake_config_sha256": "c" * 64,
                "glb_sha256": "b" * 64,
            },
        },
        browser_report_provider=lambda _root: None,
    )
    payload = report.to_dict()

    assert report.passed is False
    assert module.report_exit_code(report) == 1
    assert payload["hardware_verified"] is False
    assert payload["gates"][0]["passed"] is False
    assert "soak.control_steps" in payload["gates"][0]["errors"]
    assert payload["gates"][1]["passed"] is False
    assert "hardware_verified" in payload["gates"][1]["errors"]
    assert payload["gates"][2]["passed"] is False
    assert "frontend_rehearsal" in payload["gates"][2]["errors"]


def test_cli_atomically_replaces_stale_success_with_clean_failure_report(
    monkeypatch,
    tmp_path: Path,
) -> None:
    """A construction error must not leave stale passing acceptance evidence."""
    module = _load_module()
    output = tmp_path / "offline-rehearsal-latest.json"
    output.write_text(json.dumps({"passed": True}), encoding="utf-8")
    monkeypatch.setattr(
        module,
        "run_gate",
        lambda _root: (_ for _ in ()).throw(RuntimeError("construction exploded")),
    )

    assert module.main(["--output", str(output)]) == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["passed"] is False
    assert payload["hardware_verified"] is False
    assert payload["hardware_pending"] == HARDWARE_PENDING
    assert payload["failure"] == {
        "stage": "gate",
        "message": "construction exploded",
    }
    assert not output.with_name(f"{output.name}.tmp").exists()
