import builtins
import socket
import warnings

import pytest
from starlette.exceptions import StarletteDeprecationWarning

with warnings.catch_warnings():
    warnings.simplefilter("ignore", StarletteDeprecationWarning)
    from fastapi.testclient import TestClient

from app.main import create_app


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
