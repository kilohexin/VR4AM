from __future__ import annotations

import builtins
import os
from pathlib import Path

import pytest
import yaml

from app.config import Settings


ROOT = Path(__file__).resolve().parents[3]


REAL_CONFIG_TEMPLATE = """
backend: lebai
real_robot:
  mode: {mode}
  ip: 192.168.10.20
  expected_tcp:
    x: 0
    y: 0
    z: 0.175
    rz: 0
    ry: 0
    rx: 0
  home_q: [0, -1.0, 1.0, 0, 1.57, 0]
  teleop_ready_q: [0, -1.0, 1.0, 0, 0.20, 0]
  soft_joint_min_rad: [-3.0, -2.5, -2.5, -3.0, -2.5, -6.0]
  soft_joint_max_rad: [3.0, 2.5, 2.5, 3.0, 2.5, 6.0]
  joint_limit_margin_rad: 0.05
  startup_tcp_min_m: [0.15, -0.30, 0.10]
  startup_tcp_max_m: [0.50, 0.30, 0.60]
  tcp_position_tolerance_m: 0.001
  tcp_rotation_tolerance_deg: 0.5
  control:
    loop_hz: 50
    state_hz: 25
    pvat_send_hz: 25
    pvat_horizon_s: 0.08
    max_tcp_speed_mps: 0.03
    max_tcp_rotation_radps: 0.25
    max_tcp_acceleration_mps2: 0.10
    max_tcp_angular_acceleration_radps2: 0.5
    max_joint_speed_radps: 0.15
    max_joint_acceleration_radps2: 0.5
    max_joint_step_rad: 0.05
    max_joint_tracking_error_rad: 0.25
    max_tcp_step_m: 0.002
    max_tcp_rotation_step_deg: 1.0
    max_relative_translation_m: 0.10
    max_relative_rotation_deg: 30
    translation_scale: 0.5
  gripper:
    max_force_percent: 30
    command_hz: 10
    open_amplitude_percent: 100
    closed_amplitude_percent: 0
"""


def _write_config(tmp_path: Path, text: str) -> Path:
    path = tmp_path / "real.yaml"
    path.write_text(text, encoding="utf-8")
    return path


def _load(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    text: str,
) -> Settings:
    path = _write_config(tmp_path, text)
    monkeypatch.setenv("VR4ARM_CONFIG", str(path))
    return Settings.load()


def test_readonly_lebai_config_loads_without_importing_sdk(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _write_config(
        tmp_path,
        REAL_CONFIG_TEMPLATE.format(mode="readonly"),
    )
    monkeypatch.setenv("VR4ARM_CONFIG", str(path))
    real_import = builtins.__import__

    def reject_sdk(name, globals=None, locals=None, fromlist=(), level=0):
        if name == "lebai_sdk":
            raise AssertionError("config loading imported the real SDK")
        return real_import(name, globals, locals, fromlist, level)

    monkeypatch.setattr(builtins, "__import__", reject_sdk)
    settings = Settings.load()

    assert settings.backend == "lebai"
    assert settings.lebai is not None
    assert settings.lebai.mode == "readonly"
    assert settings.lebai.control.pvat_horizon_s == pytest.approx(0.08)
    assert settings.lebai.control.max_joint_tracking_error_rad == pytest.approx(
        0.25
    )
    assert settings.lebai.soft_joint_min_rad == (
        -3.0,
        -2.5,
        -2.5,
        -3.0,
        -2.5,
        -6.0,
    )
    assert settings.lebai.teleop_ready_q == (
        0.0,
        -1.0,
        1.0,
        0.0,
        0.2,
        0.0,
    )


def test_real_robot_example_keeps_conservative_motion_values() -> None:
    payload = yaml.safe_load(
        (ROOT / "config" / "real-robot.example.yaml").read_text(
            encoding="utf-8"
        )
    )
    control = payload["real_robot"]["control"]

    assert control["max_tcp_speed_mps"] == pytest.approx(0.03)
    assert control["translation_scale"] == pytest.approx(0.5)
    assert control["max_joint_step_rad"] == pytest.approx(0.05)
    assert control["max_joint_tracking_error_rad"] == pytest.approx(0.25)


def test_tracking_error_limit_must_cover_solution_step(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    text = REAL_CONFIG_TEMPLATE.format(mode="readonly").replace(
        "max_joint_tracking_error_rad: 0.25",
        "max_joint_tracking_error_rad: 0.04",
    )

    with pytest.raises(
        RuntimeError,
        match="^invalid_config:max_joint_tracking_error_rad$",
    ):
        _load(tmp_path, monkeypatch, text)


def test_tracking_error_limit_rejects_values_above_half_radian(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    text = REAL_CONFIG_TEMPLATE.format(mode="readonly").replace(
        "max_joint_tracking_error_rad: 0.25",
        "max_joint_tracking_error_rad: 0.51",
    )

    with pytest.raises(
        RuntimeError,
        match="^invalid_config:max_joint_tracking_error_rad$",
    ):
        _load(tmp_path, monkeypatch, text)


def test_tracking_error_limit_is_required(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    text = REAL_CONFIG_TEMPLATE.format(mode="readonly").replace(
        "    max_joint_tracking_error_rad: 0.25\n",
        "",
    )

    with pytest.raises(
        RuntimeError,
        match="^invalid_config:max_joint_tracking_error_rad$",
    ):
        _load(tmp_path, monkeypatch, text)


def test_control_mode_requires_exact_environment_confirmation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _write_config(
        tmp_path,
        REAL_CONFIG_TEMPLATE.format(mode="control"),
    )
    monkeypatch.setenv("VR4ARM_CONFIG", str(path))
    monkeypatch.delenv("VR4ARM_REAL_ROBOT_CONFIRM", raising=False)

    with pytest.raises(RuntimeError, match="^real_robot_confirmation_required$"):
        Settings.load()

    monkeypatch.setenv(
        "VR4ARM_REAL_ROBOT_CONFIRM",
        "I_UNDERSTAND_REAL_ROBOT_MOTION",
    )
    settings = Settings.load()
    assert settings.lebai is not None
    assert settings.lebai.mode == "control"


def test_tracked_default_remains_simulator(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("VR4ARM_CONFIG", raising=False)
    settings = Settings.load()
    assert settings.backend == "simulator"
    assert settings.lebai is None


def test_settings_load_accepts_explicit_path_without_mutating_environment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = _write_config(
        tmp_path,
        REAL_CONFIG_TEMPLATE.format(mode="readonly"),
    )
    monkeypatch.delenv("VR4ARM_CONFIG", raising=False)

    settings = Settings.load(path)

    assert settings.backend == "lebai"
    assert settings.lebai is not None
    assert settings.lebai.mode == "readonly"
    assert "VR4ARM_CONFIG" not in os.environ


@pytest.mark.parametrize(
    ("old", "new", "field"),
    [
        (
            "soft_joint_max_rad: [3.0, 2.5, 2.5, 3.0, 2.5, 6.0]",
            "soft_joint_max_rad: [-3.0, 2.5, 2.5, 3.0, 2.5, 6.0]",
            "soft_joint_limits",
        ),
        (
            "home_q: [0, -1.0, 1.0, 0, 1.57, 0]",
            "home_q: [3.0, -1.0, 1.0, 0, 1.57, 0]",
            "home_q",
        ),
        (
            "teleop_ready_q: [0, -1.0, 1.0, 0, 0.20, 0]",
            "teleop_ready_q: [0, -1.0, 0.0, 0, 0.20, 0]",
            "teleop_ready_q_singular",
        ),
        (
            "teleop_ready_q: [0, -1.0, 1.0, 0, 0.20, 0]",
            "teleop_ready_q: [0, -1.0, 0.08726646259971647, 0, 0.20, 0]",
            "teleop_ready_q_singular",
        ),
        (
            "startup_tcp_max_m: [0.50, 0.30, 0.60]",
            "startup_tcp_max_m: [0.10, 0.30, 0.60]",
            "startup_tcp_bounds",
        ),
    ],
)
def test_invalid_real_safety_envelope_is_rejected(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    old: str,
    new: str,
    field: str,
) -> None:
    text = REAL_CONFIG_TEMPLATE.format(mode="readonly").replace(old, new)
    with pytest.raises(RuntimeError, match=rf"^invalid_config:{field}$"):
        _load(tmp_path, monkeypatch, text)
