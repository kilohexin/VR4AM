from __future__ import annotations

import os
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient

from app.config import Settings
from app.digital_twin.runtime import (
    DIGITAL_TWIN_CONFIG,
    create_digital_twin_app,
    load_digital_twin_settings,
)
from app.recording.noop import NoopRecorder


def test_digital_twin_runtime_promotes_only_the_in_memory_copy() -> None:
    source = Settings.load(DIGITAL_TWIN_CONFIG)
    runtime = load_digital_twin_settings()
    assert source.lebai is not None
    assert source.lebai.mode == "readonly"
    assert runtime.lebai is not None
    assert runtime.lebai.mode == "control"
    assert os.environ.get("VR4ARM_REAL_ROBOT_CONFIRM") is None


def test_digital_twin_app_reports_exact_fake_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_connect = Mock(side_effect=AssertionError("real connector called"))
    monkeypatch.setattr(
        "app.robots.lebai_sdk_bridge.connect_real_client",
        real_connect,
    )
    monkeypatch.setattr(
        "app.main.build_recorder",
        lambda _settings: NoopRecorder(),
    )
    app = create_digital_twin_app()
    with TestClient(app) as client:
        health = client.get("/health").json()
    assert health["backend"] == "LEBAI_FAKE"
    assert health["real_robot_mode"] == "control"
    assert health["hardware_verified"] is False
    real_connect.assert_not_called()
