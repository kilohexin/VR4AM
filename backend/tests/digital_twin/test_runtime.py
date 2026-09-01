from __future__ import annotations

import asyncio
import json
import os
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient

import app.main as app_main
from app.config import Settings
from app.digital_twin.runtime import (
    DIGITAL_TWIN_CONFIG,
    create_digital_twin_app,
    load_digital_twin_settings,
)
from app.recording.noop import NoopRecorder
from app.recording.commissioning import CommissioningRecorder


def test_digital_twin_runtime_promotes_only_the_in_memory_copy() -> None:
    source = Settings.load(DIGITAL_TWIN_CONFIG)
    runtime = load_digital_twin_settings()
    assert source.lebai is not None
    assert source.lebai.mode == "readonly"
    assert runtime.lebai is not None
    assert runtime.lebai.mode == "control"
    assert os.environ.get("VR4ARM_REAL_ROBOT_CONFIRM") is None


def test_runtime_selects_axis_clamp_only_for_fake_backend() -> None:
    settings = load_digital_twin_settings()

    fake = app_main._build_limiter(settings, "LEBAI_FAKE")
    real = app_main._build_limiter(settings, "LEBAI")

    assert fake.workspace_boundary_mode == "axis_clamp"
    assert real.workspace_boundary_mode == "hold"


def test_fake_profile_uses_responsive_but_bounded_motion_values() -> None:
    settings = Settings.load(DIGITAL_TWIN_CONFIG)
    assert settings.lebai is not None
    control = settings.lebai.control

    assert (control.loop_hz, control.state_hz, control.pvat_send_hz) == (
        50,
        50,
        50,
    )
    assert control.pvat_horizon_s == pytest.approx(0.04)
    assert (
        control.max_tcp_speed_mps,
        control.max_tcp_rotation_radps,
        control.max_tcp_acceleration_mps2,
        control.max_tcp_angular_acceleration_radps2,
        control.max_joint_speed_radps,
        control.max_joint_acceleration_radps2,
        control.max_joint_step_rad,
        control.max_tcp_step_m,
        control.max_tcp_rotation_step_deg,
        control.max_relative_translation_m,
        control.max_relative_rotation_deg,
        control.translation_scale,
    ) == pytest.approx(
        (
            0.60,
            1.50,
            2.40,
            4.00,
            1.50,
            4.00,
            0.060,
            0.012,
            2.0,
            0.16,
            45.0,
            1.50,
        )
    )


def test_digital_twin_app_reports_exact_fake_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    real_connect = Mock(side_effect=AssertionError("real connector called"))
    recorder_runtime: list[str] = []
    monkeypatch.setattr(
        "app.robots.lebai_sdk_bridge.connect_real_client",
        real_connect,
    )
    monkeypatch.setattr(
        "app.main.connect_real_client",
        real_connect,
    )

    def fake_recorder(_settings: Settings, runtime: str) -> NoopRecorder:
        recorder_runtime.append(runtime)
        return NoopRecorder()

    monkeypatch.setattr(
        "app.main.build_recorder",
        fake_recorder,
    )
    app = create_digital_twin_app()
    with TestClient(app) as client:
        health = client.get("/health").json()
    assert health["backend"] == "LEBAI_FAKE"
    assert health["real_robot_mode"] == "control"
    assert health["hardware_verified"] is False
    assert recorder_runtime == ["LEBAI_FAKE"]
    real_connect.assert_not_called()


def test_fake_runtime_recorder_durably_identifies_digital_twin(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(app_main, "REPOSITORY_ROOT", tmp_path)
    recorder = app_main.build_recorder(
        load_digital_twin_settings(),
        "LEBAI_FAKE",
    )
    assert isinstance(recorder, CommissioningRecorder)

    async def record_session() -> None:
        await recorder.start()
        await recorder.close()

    asyncio.run(record_session())
    first_entry = json.loads(
        recorder.jsonl_path.read_text(encoding="utf-8").splitlines()[0]
    )

    assert first_entry["kind"] == "session_started"
    assert first_entry["metadata"]["backend"] == "LEBAI_FAKE"
    assert first_entry["metadata"]["runtime"] == "SIMULATION"
    assert first_entry["metadata"]["hardware_verified"] is False
    assert first_entry["metadata"]["real_robot_mode"] == "control"
