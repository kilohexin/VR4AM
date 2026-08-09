from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import urllib.error
import urllib.request
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, TextIO


ROOT = Path(__file__).resolve().parents[1]
FAKE_CONFIG = "config/fake-lebai.yaml"


class ChildProcess(Protocol):
    def poll(self) -> int | None: ...

    def terminate(self) -> None: ...

    def wait(self, timeout: float | None = None) -> int: ...

    def kill(self) -> None: ...


@dataclass(frozen=True)
class LaunchSpec:
    root: Path
    backend: tuple[str, ...]
    frontend: tuple[str, ...]
    environment: Mapping[str, str]
    backend_url: str
    frontend_url: str
    runtime: str
    readiness_timeout_s: float = 30.0


ProcessFactory = Callable[
    [Sequence[str], Path, Mapping[str, str]],
    ChildProcess,
]
Probe = Callable[[str, str | None], bool]


def build_launch_spec(env: Mapping[str, str], os_name: str) -> LaunchSpec:
    inherited_config = env.get("VR4ARM_CONFIG")
    if (
        inherited_config is not None
        and inherited_config.replace("\\", "/") != FAKE_CONFIG
    ):
        raise ValueError("unsafe_offline_environment:VR4ARM_CONFIG")
    if "VR4ARM_REAL_ROBOT_CONFIRM" in env:
        raise ValueError("unsafe_offline_environment:VR4ARM_REAL_ROBOT_CONFIRM")

    environment = dict(env)
    environment["VR4ARM_CONFIG"] = FAKE_CONFIG
    npm = "npm.cmd" if os_name == "nt" else "npm"
    return LaunchSpec(
        root=ROOT,
        backend=(sys.executable, "scripts/run_fake_lebai_stack.py"),
        frontend=(npm, "run", "dev", "--", "--host", "127.0.0.1"),
        environment=environment,
        backend_url="http://127.0.0.1:8000/health",
        frontend_url="http://127.0.0.1:5173/",
        runtime="LEBAI_FAKE",
    )


def _default_process_factory(
    argv: Sequence[str],
    cwd: Path,
    env: Mapping[str, str],
) -> ChildProcess:
    return subprocess.Popen(list(argv), cwd=cwd, env=dict(env))


def probe_url(url: str, expected_runtime: str | None) -> bool:
    try:
        with urllib.request.urlopen(url, timeout=1.0) as response:
            if response.status < 200 or response.status >= 400:
                return False
            if expected_runtime is None:
                return True
            payload = json.loads(response.read().decode("utf-8"))
    except (
        OSError,
        UnicodeDecodeError,
        json.JSONDecodeError,
        urllib.error.URLError,
    ):
        return False
    return bool(
        isinstance(payload, dict)
        and payload.get("status") == "ok"
        and payload.get("backend") == expected_runtime
        and payload.get("hardware_verified") is False
        and payload.get("real_robot_enabled") is False
    )


def _wait_until_ready(
    child: ChildProcess,
    url: str,
    expected_runtime: str | None,
    probe: Probe,
    *,
    timeout_s: float,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
) -> None:
    deadline = monotonic() + timeout_s
    while True:
        returncode = child.poll()
        if returncode is not None:
            raise RuntimeError(f"child_exited_before_ready:{returncode}")
        if probe(url, expected_runtime):
            return
        if monotonic() >= deadline:
            raise RuntimeError(f"readiness_timeout:{url}")
        sleep(0.1)


def _stop_children(children: Sequence[ChildProcess]) -> None:
    for child in reversed(children):
        if child.poll() is not None:
            continue
        child.terminate()
        try:
            child.wait(timeout=5.0)
        except (subprocess.TimeoutExpired, TimeoutError):
            if child.poll() is None:
                child.kill()
                try:
                    child.wait(timeout=5.0)
                except (subprocess.TimeoutExpired, TimeoutError):
                    pass


def run_launcher(
    spec: LaunchSpec,
    process_factory: ProcessFactory,
    probe: Probe,
    *,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    stdout: TextIO = sys.stdout,
    stderr: TextIO = sys.stderr,
) -> int:
    children: list[ChildProcess] = []
    try:
        backend = process_factory(spec.backend, spec.root, spec.environment)
        children.append(backend)
        _wait_until_ready(
            backend,
            spec.backend_url,
            spec.runtime,
            probe,
            timeout_s=spec.readiness_timeout_s,
            monotonic=monotonic,
            sleep=sleep,
        )

        frontend = process_factory(spec.frontend, spec.root, spec.environment)
        children.append(frontend)
        _wait_until_ready(
            frontend,
            spec.frontend_url,
            None,
            probe,
            timeout_s=spec.readiness_timeout_s,
            monotonic=monotonic,
            sleep=sleep,
        )
        print(f"Offline rehearsal ready: {spec.frontend_url}", file=stdout)
        print(
            "Runtime: LEBAI_FAKE / DIGITAL_TWIN / hardware_verified=false",
            file=stdout,
        )
        print("Press Ctrl+C to stop.", file=stdout)
        while True:
            for child in children:
                returncode = child.poll()
                if returncode is not None:
                    raise RuntimeError(f"child_exited:{returncode}")
            sleep(0.2)
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print(f"Offline rehearsal failed: {error}", file=stderr)
        return 1
    finally:
        _stop_children(children)


def main() -> int:
    try:
        spec = build_launch_spec(os.environ, os.name)
    except ValueError as error:
        print(f"Offline rehearsal refused: {error}", file=sys.stderr)
        return 1
    return run_launcher(spec, _default_process_factory, probe_url)


if __name__ == "__main__":
    raise SystemExit(main())
