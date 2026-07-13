import builtins
import asyncio
import contextlib
import socket
import warnings

import pytest
from starlette.exceptions import StarletteDeprecationWarning

with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore",
        message=(
            r"Using `httpx` with `starlette\.testclient` is deprecated; "
            r"install `httpx2` instead\."
        ),
        category=StarletteDeprecationWarning,
    )
    from fastapi.testclient import TestClient

from app.main import create_app


def _install_lifecycle_fakes(
    monkeypatch: pytest.MonkeyPatch,
    *,
    failure_stage: str | None = None,
    stop_error: bool = False,
):
    import app.main as main_module

    class FakeBackend:
        instances: list["FakeBackend"] = []

        def __init__(self) -> None:
            self.connect_count = 0
            self.disconnect_count = 0
            self.task: asyncio.Task[None] | None = None
            self.instances.append(self)

        async def connect(self) -> None:
            self.connect_count += 1
            self.task = asyncio.create_task(asyncio.Event().wait())

        async def disconnect(self) -> None:
            self.disconnect_count += 1
            task, self.task = self.task, None
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task

    class FakeControl:
        instances: list["FakeControl"] = []

        def __init__(self, **kwargs) -> None:
            self.recorder = kwargs["recorder"]
            self.connect_count = 0
            self.start_count = 0
            self.stop_count = 0
            self.task: asyncio.Task[None] | None = None
            self.instances.append(self)

        async def connect(self) -> None:
            self.connect_count += 1
            if failure_stage == "connect":
                raise RuntimeError("connect_failed")

        async def start(self) -> None:
            self.start_count += 1
            self.task = asyncio.create_task(asyncio.Event().wait())
            if failure_stage == "start":
                raise RuntimeError("start_failed")

        async def stop(self) -> None:
            self.stop_count += 1
            task, self.task = self.task, None
            if task is not None:
                task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await task
            if stop_error:
                raise RuntimeError("stop_failed")

    monkeypatch.setattr(main_module, "SimRobotAdapter", FakeBackend)
    monkeypatch.setattr(main_module, "RobotControl", FakeControl)
    return FakeBackend, FakeControl


def test_health_is_simulator_only() -> None:
    with TestClient(create_app()) as client:
        assert client.get("/health").json() == {
            "status": "ok",
            "backend": "SIMULATOR",
            "real_robot_enabled": False,
        }


def test_non_simulator_backend_is_rejected_without_sdk_or_network(
    tmp_path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config_path = tmp_path / "real.yaml"
    config_path.write_text("backend: lebai\n", encoding="utf-8")
    monkeypatch.setenv("VR4ARM_CONFIG", str(config_path))
    real_import = builtins.__import__

    def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name.startswith("lebai"):
            raise AssertionError("不得导入真实机器人 SDK")
        return real_import(name, globals, locals, fromlist, level)

    def reject_network(*args, **kwargs):
        raise AssertionError("不得连接真实机器人 IP")

    monkeypatch.setattr(builtins, "__import__", guarded_import)
    monkeypatch.setattr(socket, "create_connection", reject_network)

    with pytest.raises(RuntimeError, match="^real_robot_disabled$"):
        create_app()


@pytest.mark.parametrize("failure_stage", ["connect", "start"])
def test_startup_control_failure_stops_control_and_disconnects_backend(
    monkeypatch: pytest.MonkeyPatch, failure_stage: str
) -> None:
    backend_type, control_type = _install_lifecycle_fakes(
        monkeypatch, failure_stage=failure_stage
    )

    with pytest.raises(RuntimeError, match=f"^{failure_stage}_failed$"):
        with TestClient(create_app()):
            pass

    backend = backend_type.instances[0]
    control = control_type.instances[0]
    assert control.stop_count == 1
    assert control.task is None
    assert backend.disconnect_count == 1
    assert backend.task is None


def test_startup_error_is_not_masked_by_cleanup_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend_type, control_type = _install_lifecycle_fakes(
        monkeypatch, failure_stage="connect", stop_error=True
    )

    with pytest.raises(RuntimeError, match="^connect_failed$"):
        with TestClient(create_app()):
            pass

    assert control_type.instances[0].stop_count == 1
    assert backend_type.instances[0].disconnect_count == 1
    assert backend_type.instances[0].task is None


def test_shutdown_stop_failure_still_disconnects_backend(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    backend_type, control_type = _install_lifecycle_fakes(
        monkeypatch, stop_error=True
    )

    with pytest.raises(RuntimeError, match="^stop_failed$"):
        with TestClient(create_app()):
            pass

    assert control_type.instances[0].task is None
    assert backend_type.instances[0].disconnect_count == 1
    assert backend_type.instances[0].task is None
