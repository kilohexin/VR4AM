from __future__ import annotations

import importlib

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_sdk_bridge import (
    connect_real_client,
    detect_capabilities,
)
from tests.robots.fake_lebai import FakeLebaiModule


@pytest.mark.asyncio
async def test_connect_real_client_initializes_and_awaits_async_connect(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = FakeLebaiModule()
    imported: list[str] = []

    def import_module(name: str):
        imported.append(name)
        return module

    monkeypatch.setattr(importlib, "import_module", import_module)
    client = await connect_real_client("192.168.10.20")

    assert imported == ["lebai_sdk"]
    assert module.init_count == 1
    assert module.connect_calls == [("192.168.10.20", False)]
    assert client is module.client


@pytest.mark.asyncio
async def test_missing_sdk_becomes_public_backend_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def missing(name: str):
        raise ModuleNotFoundError(name)

    monkeypatch.setattr(importlib, "import_module", missing)
    with pytest.raises(BackendCommandError, match="^lebai_sdk_unavailable$"):
        await connect_real_client("192.168.10.20")


def test_capability_detection_requires_every_method_for_control() -> None:
    module = FakeLebaiModule()
    capabilities = detect_capabilities(module.client)
    assert capabilities.control_ready is True
    assert capabilities.names == (
        "state",
        "tcp",
        "kinematics_inverse",
        "pvat",
        "stop_move",
        "stop_sys",
        "home",
        "gripper",
        "running_motion",
    )

    module.client.move_pvat = None  # type: ignore[method-assign]
    missing_pvat = detect_capabilities(module.client)
    assert missing_pvat.control_ready is False
    assert "pvat" not in missing_pvat.names
