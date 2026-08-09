from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT_PATH = ROOT / "scripts" / "run_offline_rehearsal.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "run_offline_rehearsal_test_module",
        SCRIPT_PATH,
    )
    if spec is None or spec.loader is None:
        raise AssertionError("could not load offline rehearsal launcher")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_launch_spec_forces_the_loopback_fake_stack() -> None:
    """A launcher that inherits a real profile or public bind must fail."""
    module = _load_module()
    npm = "npm.cmd" if os.name == "nt" else "npm"

    spec = module.build_launch_spec({}, os.name)

    assert spec.backend == (
        sys.executable,
        "scripts/run_fake_lebai_stack.py",
    )
    assert spec.frontend == (
        npm,
        "run",
        "dev",
        "--",
        "--host",
        "127.0.0.1",
    )
    assert spec.environment["VR4ARM_CONFIG"] == "config/fake-lebai.yaml"
    assert spec.backend_url == "http://127.0.0.1:8000/health"
    assert spec.frontend_url == "http://127.0.0.1:5173/"
    assert spec.runtime == "LEBAI_FAKE"


@pytest.mark.parametrize(
    "unsafe_environment",
    [
        {"VR4ARM_CONFIG": "config/real-robot.local.yaml"},
        {"VR4ARM_REAL_ROBOT_CONFIRM": "I_UNDERSTAND_REAL_ROBOT_MOTION"},
        {
            "VR4ARM_CONFIG": "config/fake-lebai.yaml",
            "VR4ARM_REAL_ROBOT_CONFIRM": "anything",
        },
    ],
)
def test_launch_spec_rejects_inherited_real_robot_environment(
    unsafe_environment: dict[str, str],
) -> None:
    """A stale shell confirmation must never cross into offline rehearsal."""
    module = _load_module()

    with pytest.raises(ValueError, match="unsafe_offline_environment"):
        module.build_launch_spec(unsafe_environment, os.name)


class _Process:
    def __init__(
        self,
        name: str,
        events: list[str],
        *,
        alive_after_wait: bool = False,
        interrupt_after_polls: int | None = None,
    ) -> None:
        self.name = name
        self.events = events
        self.alive = True
        self.alive_after_wait = alive_after_wait
        self.interrupt_after_polls = interrupt_after_polls
        self.poll_count = 0

    def poll(self):
        self.poll_count += 1
        if self.interrupt_after_polls == self.poll_count:
            raise KeyboardInterrupt
        return None if self.alive else 0

    def terminate(self) -> None:
        self.events.append(f"terminate:{self.name}")
        if not self.alive_after_wait:
            self.alive = False

    def wait(self, timeout: float | None = None) -> int:
        self.events.append(f"wait:{self.name}:{timeout}")
        if self.alive_after_wait:
            raise TimeoutError("still running")
        self.alive = False
        return 0

    def kill(self) -> None:
        self.events.append(f"kill:{self.name}")
        self.alive = False


def test_ctrl_c_cleans_up_frontend_then_backend_and_prints_exact_ready_block(
    capsys: pytest.CaptureFixture[str],
) -> None:
    """Interrupt cleanup must not orphan either child or hide the hardware boundary."""
    module = _load_module()
    spec = module.build_launch_spec({}, os.name)
    events: list[str] = []
    children: list[_Process] = []

    def process_factory(argv, cwd, env):
        assert cwd == ROOT
        assert env["VR4ARM_CONFIG"] == "config/fake-lebai.yaml"
        name = "backend" if not children else "frontend"
        child = _Process(
            name,
            events,
            interrupt_after_polls=2 if name == "frontend" else None,
        )
        children.append(child)
        return child

    probes: list[tuple[str, str | None]] = []

    def probe(url: str, expected_runtime: str | None) -> bool:
        probes.append((url, expected_runtime))
        return True

    assert module.run_launcher(
        spec,
        process_factory,
        probe,
        sleep=lambda _seconds: None,
    ) == 0
    assert probes == [
        ("http://127.0.0.1:8000/health", "LEBAI_FAKE"),
        ("http://127.0.0.1:5173/", None),
    ]
    assert events == [
        "terminate:frontend",
        "wait:frontend:5.0",
        "terminate:backend",
        "wait:backend:5.0",
    ]
    output = capsys.readouterr().out
    assert output.splitlines() == [
        "Offline rehearsal ready: http://127.0.0.1:5173/",
        "Runtime: LEBAI_FAKE / DIGITAL_TWIN / hardware_verified=false",
        "Press Ctrl+C to stop.",
    ]


def test_frontend_startup_failure_stops_and_force_kills_only_live_children() -> None:
    """A failed Vite probe must bound cleanup and kill only unresponsive children."""
    module = _load_module()
    spec = module.build_launch_spec({}, os.name)
    events: list[str] = []
    children: list[_Process] = []

    def process_factory(_argv, _cwd, _env):
        name = "backend" if not children else "frontend"
        child = _Process(
            name,
            events,
            alive_after_wait=name == "backend",
        )
        children.append(child)
        return child

    def probe(url: str, _expected_runtime: str | None) -> bool:
        if url.endswith(":5173/"):
            raise RuntimeError("vite_probe_failed")
        return True

    assert module.run_launcher(
        spec,
        process_factory,
        probe,
        sleep=lambda _seconds: None,
    ) == 1
    assert events == [
        "terminate:frontend",
        "wait:frontend:5.0",
        "terminate:backend",
        "wait:backend:5.0",
        "kill:backend",
        "wait:backend:5.0",
    ]
