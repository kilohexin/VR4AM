from __future__ import annotations

import json
import os
import signal
import ssl
import subprocess
import sys
import time
import urllib.error
import urllib.parse
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


def _windows_process_parents() -> dict[int, int]:
    import ctypes
    from ctypes import wintypes

    class ProcessEntry32W(ctypes.Structure):
        _fields_ = [
            ("dwSize", wintypes.DWORD),
            ("cntUsage", wintypes.DWORD),
            ("th32ProcessID", wintypes.DWORD),
            ("th32DefaultHeapID", ctypes.c_size_t),
            ("th32ModuleID", wintypes.DWORD),
            ("cntThreads", wintypes.DWORD),
            ("th32ParentProcessID", wintypes.DWORD),
            ("pcPriClassBase", wintypes.LONG),
            ("dwFlags", wintypes.DWORD),
            ("szExeFile", wintypes.WCHAR * 260),
        ]

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    snapshot_fn = kernel32.CreateToolhelp32Snapshot
    snapshot_fn.argtypes = [wintypes.DWORD, wintypes.DWORD]
    snapshot_fn.restype = wintypes.HANDLE
    first_fn = kernel32.Process32FirstW
    first_fn.argtypes = [wintypes.HANDLE, ctypes.POINTER(ProcessEntry32W)]
    first_fn.restype = wintypes.BOOL
    next_fn = kernel32.Process32NextW
    next_fn.argtypes = [wintypes.HANDLE, ctypes.POINTER(ProcessEntry32W)]
    next_fn.restype = wintypes.BOOL
    close_fn = kernel32.CloseHandle
    close_fn.argtypes = [wintypes.HANDLE]
    close_fn.restype = wintypes.BOOL

    snapshot = snapshot_fn(0x00000002, 0)
    if snapshot == ctypes.c_void_p(-1).value:
        return {}
    parents: dict[int, int] = {}
    try:
        entry = ProcessEntry32W()
        entry.dwSize = ctypes.sizeof(entry)
        if not first_fn(snapshot, ctypes.byref(entry)):
            return parents
        while True:
            parents[int(entry.th32ProcessID)] = int(entry.th32ParentProcessID)
            if not next_fn(snapshot, ctypes.byref(entry)):
                return parents
    finally:
        close_fn(snapshot)


def _windows_pid_alive(pid: int) -> bool:
    import ctypes
    from ctypes import wintypes

    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    open_fn = kernel32.OpenProcess
    open_fn.argtypes = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD]
    open_fn.restype = wintypes.HANDLE
    wait_fn = kernel32.WaitForSingleObject
    wait_fn.argtypes = [wintypes.HANDLE, wintypes.DWORD]
    wait_fn.restype = wintypes.DWORD
    close_fn = kernel32.CloseHandle
    close_fn.argtypes = [wintypes.HANDLE]
    close_fn.restype = wintypes.BOOL

    handle = open_fn(0x00100000, False, pid)
    if not handle:
        return False
    try:
        return wait_fn(handle, 0) == 0x00000102
    finally:
        close_fn(handle)


class ProcessTree:
    """A child process whose lifecycle includes descendants it starts."""

    def __init__(self, process: subprocess.Popen[object], os_name: str) -> None:
        self._process = process
        self._os_name = os_name
        self._tracked_pids = {process.pid}

    def _refresh_windows_tree(self) -> None:
        parents = _windows_process_parents()
        changed = True
        while changed:
            changed = False
            for pid, parent in parents.items():
                if parent in self._tracked_pids and pid not in self._tracked_pids:
                    self._tracked_pids.add(pid)
                    changed = True

    def _windows_alive_pids(self) -> list[int]:
        self._refresh_windows_tree()
        return sorted(pid for pid in self._tracked_pids if _windows_pid_alive(pid))

    def _posix_group_alive(self) -> bool:
        try:
            os.killpg(self._process.pid, 0)
        except ProcessLookupError:
            return False
        return True

    def poll(self) -> int | None:
        returncode = self._process.poll()
        if self._os_name == "nt":
            return None if self._windows_alive_pids() else returncode
        return None if self._posix_group_alive() else returncode

    def terminate(self) -> None:
        if self.poll() is not None:
            return
        if self._os_name == "nt":
            try:
                self._process.send_signal(signal.CTRL_BREAK_EVENT)
            except (OSError, ValueError):
                self._process.terminate()
            return
        try:
            os.killpg(self._process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    def wait(self, timeout: float | None = None) -> int:
        deadline = None if timeout is None else time.monotonic() + timeout
        returncode = self._process.wait(timeout=timeout)
        while self.poll() is None:
            if deadline is not None and time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(self._process.args, timeout)
            time.sleep(0.05)
        return returncode

    def kill(self) -> None:
        if self._os_name == "nt":
            for pid in self._windows_alive_pids():
                try:
                    os.kill(pid, signal.SIGTERM)
                except (OSError, ProcessLookupError):
                    pass
            if self._process.poll() is None:
                self._process.kill()
            return
        try:
            os.killpg(self._process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


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
        frontend_url="https://127.0.0.1:5173/",
        runtime="LEBAI_FAKE",
    )


def _default_process_factory(
    argv: Sequence[str],
    cwd: Path,
    env: Mapping[str, str],
) -> ChildProcess:
    root = cwd.parent if cwd.name == "web" else cwd
    label = "frontend" if cwd.name == "web" else "backend"
    log_path = root / "logs" / f"offline-rehearsal-{label}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w", encoding="utf-8")
    try:
        process = subprocess.Popen(
            list(argv),
            cwd=cwd,
            env=dict(env),
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=os.name != "nt",
            creationflags=(
                subprocess.CREATE_NEW_PROCESS_GROUP
                if os.name == "nt"
                else 0
            ),
        )
        return ProcessTree(process, os.name)
    finally:
        log.close()


def probe_url(url: str, expected_runtime: str | None) -> bool:
    context: ssl.SSLContext | None = None
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme == "https" and parsed.hostname in {
        "127.0.0.1",
        "localhost",
        "::1",
    }:
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE
    try:
        with urllib.request.urlopen(
            url,
            timeout=1.0,
            context=context,
        ) as response:
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

        frontend = process_factory(
            spec.frontend,
            spec.root / "web",
            spec.environment,
        )
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
