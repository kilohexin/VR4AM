import builtins
import asyncio
import contextlib
import socket
import warnings

import pytest
try:
    from starlette.exceptions import StarletteDeprecationWarning
except ImportError:
    class StarletteDeprecationWarning(DeprecationWarning):
        """Compatibility category for Starlette versions that removed it."""


STARLETTE_HTTPX_WARNING_TEXT = (
    "Using `httpx` with `starlette.testclient` is deprecated; "
    "install `httpx2` instead."
)
STARLETTE_HTTPX_WARNING_PATTERN = (
    r"\AUsing `httpx` with `starlette\.testclient` is deprecated; "
    r"install `httpx2` instead\.\Z"
)

with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore",
        message=STARLETTE_HTTPX_WARNING_PATTERN,
        category=StarletteDeprecationWarning,
    )
    from fastapi.testclient import TestClient

from app.config import Settings
from app.main import create_app
from app.recording.noop import NoopRecorder
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient


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

        async def preflight(self):
            from app.robots.base import BackendPreflight
            from app.schemas.messages import BackendState, Pose

            return BackendPreflight(
                ready=True,
                reason=None,
                robot_state=BackendState.IDLE,
                actual_tcp=Pose(p=(0.3, 0.0, 0.3), q=(0, 0, 0, 1)),
                actual_q=(0, 0, 0, 0, 0, 0),
                tcp_matches=True,
                capabilities=("command_tcp", "home", "gripper"),
            )

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


def test_simulator_app_never_imports_lebai_sdk(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_import = builtins.__import__

    def guarded(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "lebai_sdk":
            raise AssertionError("simulator imported real SDK")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", guarded)
    with TestClient(create_app()) as client:
        assert client.get("/health").json() == {
            "status": "ok",
            "backend": "SIMULATOR",
            "real_robot_mode": None,
            "real_robot_enabled": False,
            "preflight_ready": True,
            "preflight_reason": None,
        }


def test_starlette_warning_filter_does_not_hide_extended_message() -> None:
    extended = f"{STARLETTE_HTTPX_WARNING_TEXT} extra context"

    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        warnings.filterwarnings(
            "ignore",
            message=STARLETTE_HTTPX_WARNING_PATTERN,
            category=StarletteDeprecationWarning,
        )
        warnings.warn(
            STARLETTE_HTTPX_WARNING_TEXT,
            StarletteDeprecationWarning,
            stacklevel=1,
        )
        warnings.warn(extended, StarletteDeprecationWarning, stacklevel=1)

    assert [str(item.message) for item in caught] == [extended]


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

    with pytest.raises(RuntimeError, match="^invalid_config:real_robot$"):
        create_app()


def test_readonly_health_reports_not_enabled(
    tmp_path, monkeypatch: pytest.MonkeyPatch
) -> None:
    config_path = tmp_path / "readonly-real.yaml"
    config_path.write_text(
        REAL_CONFIG_TEMPLATE.format(mode="readonly"),
        encoding="utf-8",
    )
    monkeypatch.setenv("VR4ARM_CONFIG", str(config_path))
    settings = Settings.load()
    fake = FakeLebaiClient.idle()

    async def fake_client_factory(ip: str):
        assert ip == "192.168.10.20"
        return fake

    monkeypatch.setattr(
        "app.main.build_recorder",
        lambda _settings: NoopRecorder(),
        raising=False,
    )
    with TestClient(
        create_app(settings=settings, client_factory=fake_client_factory)
    ) as client:
        payload = client.get("/health").json()

    assert payload == {
        "status": "ok",
        "backend": "LEBAI",
        "real_robot_mode": "readonly",
        "real_robot_enabled": False,
        "preflight_ready": False,
        "preflight_reason": "real_robot_readonly",
    }
    assert fake.write_calls == []


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
