from __future__ import annotations

from dataclasses import replace

import numpy as np

from app.acceptance.profile import (
    compare_profiles,
    onsite_profile,
    simulation_profile,
)
from app.sim.lm3_model import LM3Model
from tests.robots.real_settings import readonly_settings


def test_simulation_profile_uses_versioned_model_and_tool_transform() -> None:
    model = LM3Model()

    profile = simulation_profile()

    assert profile.source == "simulation"
    assert profile.hardware_verified is False
    assert profile.home_q == model.home_q
    assert profile.joint_min_rad == tuple(
        value - model.joint_window_rad for value in model.home_q
    )
    assert profile.joint_max_rad == tuple(
        value + model.joint_window_rad for value in model.home_q
    )
    assert np.linalg.norm(profile.tcp.p) > 0
    assert profile.gripper_closed_amplitude_is_lower is True


def test_onsite_profile_preserves_configured_limits_tcp_and_gripper_polarity() -> None:
    settings = readonly_settings()

    profile = onsite_profile(settings)

    assert profile.source == "onsite_config"
    assert profile.hardware_verified is False
    assert profile.home_q == settings.home_q
    assert profile.joint_min_rad == settings.soft_joint_min_rad
    assert profile.joint_max_rad == settings.soft_joint_max_rad
    assert profile.tcp.p == (0.0, 0.0, 0.175)
    assert profile.gripper_closed_amplitude_is_lower is True


def test_profile_comparison_allows_missing_onsite_config_without_claiming_match() -> None:
    comparison = compare_profiles(simulation_profile(), None)

    assert comparison.onsite_configured is False
    assert comparison.differences == ()
    assert comparison.to_dict() == {
        "onsite_configured": False,
        "differences": [],
    }


def test_profile_comparison_reports_gripper_direction_and_pose_differences() -> None:
    settings = readonly_settings()
    reversed_gripper = replace(
        settings.gripper,
        open_amplitude_percent=0,
        closed_amplitude_percent=100,
    )
    onsite = onsite_profile(
        replace(settings, gripper=reversed_gripper)
    )

    comparison = compare_profiles(simulation_profile(), onsite)
    fields = {difference.field for difference in comparison.differences}

    assert comparison.onsite_configured is True
    assert "gripper_direction" in fields
    assert "tcp.position" in fields
    assert any(field.startswith("home_q[") for field in fields)
    assert all(
        difference.severity in {"info", "warning"}
        for difference in comparison.differences
    )
