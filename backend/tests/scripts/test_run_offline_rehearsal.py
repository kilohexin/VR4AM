from __future__ import annotations

import importlib.util
import json
import os
import signal
import socket
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

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
    assert spec.frontend_url == "https://127.0.0.1:5173/"
    assert spec.runtime == "LEBAI_FAKE"


def test_probe_accepts_fake_control_health_without_hardware_verification() -> None:
    """Fake control mode must not be confused with verified real hardware."""
    module = _load_module()
    payload = json.dumps(
        {
            "status": "ok",
            "backend": "LEBAI_FAKE",
            "real_robot_mode": "control",
            "real_robot_enabled": True,
            "preflight_ready": True,
            "preflight_reason": None,
            "hardware_verified": False,
        }
    ).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def log_message(self, _format: str, *_args: object) -> None:
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address
        assert module.probe_url(
            f"http://{host}:{port}/health",
            "LEBAI_FAKE",
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_https_probe_relaxes_certificates_only_for_loopback(monkeypatch) -> None:
    """The generated local certificate must not weaken arbitrary HTTPS probes."""
    module = _load_module()
    calls: list[tuple[str, object | None]] = []

    class Response:
        status = 200

        def __enter__(self):
            return self

        def __exit__(self, *_args: object) -> None:
            return None

    def urlopen(url: str, *, timeout: float, context=None):
        assert timeout == 1.0
        calls.append((url, context))
        return Response()

    monkeypatch.setattr(module.urllib.request, "urlopen", urlopen)

    assert module.probe_url("https://127.0.0.1:5173/", None)
    assert module.probe_url("https://example.invalid/", None)
    assert calls[0][1] is not None
    assert calls[0][1].check_hostname is False
    assert calls[1][1] is None


def test_default_child_logs_are_routed_out_of_launcher_console(
    capfd: pytest.CaptureFixture[str],
    tmp_path: Path,
) -> None:
    """A real child must not contaminate the exact launcher stdout block."""
    module = _load_module()
    child = module._default_process_factory(
        (
            sys.executable,
            "-c",
            "import sys; print('child-out'); print('child-err', file=sys.stderr)",
        ),
        tmp_path,
        os.environ,
    )

    assert child.wait(timeout=5.0) == 0
    captured = capfd.readouterr()
    assert captured.out == ""
    assert captured.err == ""
    log = tmp_path / "logs" / "offline-rehearsal-backend.log"
    content = log.read_text(encoding="utf-8")
    assert "child-out" in content
    assert "child-err" in content


def _port_is_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.1):
            return True
    except OSError:
        return False


def _wait_for(predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def test_default_process_cleanup_terminates_descendant_listener(
    tmp_path: Path,
) -> None:
    """Stopping a wrapper must also release a listener owned by its descendant."""
    module = _load_module()
    with socket.socket() as reservation:
        reservation.bind(("127.0.0.1", 0))
        port = reservation.getsockname()[1]
    pid_path = tmp_path / "descendant.pid"
    ignored_signal = "signal.SIGBREAK" if os.name == "nt" else "signal.SIGTERM"
    descendant_code = (
        "import http.server,signal;"
        f"signal.signal({ignored_signal},signal.SIG_IGN);"
        "http.server.ThreadingHTTPServer(('127.0.0.1',"
        f"{port}),http.server.SimpleHTTPRequestHandler).serve_forever()"
    )
    parent_code = (
        "import pathlib,subprocess,sys,time;"
        f"child=subprocess.Popen([sys.executable,'-c',{descendant_code!r}],"
        "stdout=subprocess.DEVNULL,"
        "stderr=subprocess.DEVNULL);"
        f"pathlib.Path({str(pid_path)!r}).write_text(str(child.pid));"
        "time.sleep(60)"
    )
    parent = module._default_process_factory(
        (sys.executable, "-c", parent_code),
        tmp_path,
        os.environ,
    )
    descendant_pid: int | None = None
    try:
        assert _wait_for(pid_path.exists, 5.0)
        descendant_pid = int(pid_path.read_text(encoding="utf-8"))
        assert _wait_for(lambda: _port_is_open(port), 5.0)

        module._stop_children([parent])

        assert parent.poll() is not None
        assert _wait_for(lambda: not _port_is_open(port), 2.0)
    finally:
        if descendant_pid is not None and _port_is_open(port):
            if os.name == "nt":
                os.kill(descendant_pid, signal.SIGTERM)
            else:
                try:
                    os.kill(descendant_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=5.0)


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
        assert env["VR4ARM_CONFIG"] == "config/fake-lebai.yaml"
        name = "backend" if not children else "frontend"
        assert cwd == (ROOT if name == "backend" else ROOT / "web")
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
        ("https://127.0.0.1:5173/", None),
    ]
    assert events == [
        "terminate:frontend",
        "wait:frontend:5.0",
        "terminate:backend",
        "wait:backend:5.0",
    ]
    output = capsys.readouterr().out
    assert output.splitlines() == [
        "Offline rehearsal ready: https://127.0.0.1:5173/",
        "Runtime: LEBAI_FAKE / SIMULATION / hardware_verified=false",
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
