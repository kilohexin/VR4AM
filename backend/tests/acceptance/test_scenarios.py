from __future__ import annotations

import math

import pytest

from app.acceptance.scenarios import (
    run_mapping_scenario,
    run_servo_tracking_scenario,
    run_soft_constraint_scenario,
    run_virtual_scenarios,
)


def test_mapping_scenario_checks_all_translation_and_rotation_axes() -> None:
    result = run_mapping_scenario()

    assert result.passed is True
    assert result.failures == ()
    assert result.metrics["translation_axes_checked"] == 3
    assert result.metrics["rotation_axes_checked"] == 3
    assert result.metrics["max_translation_error_m"] <= 1e-9
    assert result.metrics["max_rotation_error_rad"] <= 1e-9


@pytest.mark.asyncio
async def test_servo_tracking_scenario_converges_without_joint_violations() -> None:
    result = await run_servo_tracking_scenario()

    assert result.passed is True
    assert result.failures == ()
    assert result.metrics["cycles"] == 150
    assert result.metrics["final_position_error_m"] <= 0.005
    assert (
        result.metrics["final_position_error_m"]
        < result.metrics["initial_position_error_m"]
    )
    assert result.metrics["joint_window_violations"] == 0
    assert result.metrics["nan_count"] == 0


@pytest.mark.asyncio
async def test_soft_constraint_scenario_retreats_without_fault_or_home() -> None:
    result = await run_soft_constraint_scenario()

    assert result.passed is True
    assert result.failures == ()
    assert result.metrics == {
        "constraint_observed": True,
        "constraint_cleared": True,
        "fault_stop_count": 0,
        "final_mode": "ACTIVE",
    }


@pytest.mark.asyncio
async def test_virtual_scenarios_are_ordered_json_safe_and_all_pass() -> None:
    results = await run_virtual_scenarios()

    assert [result.name for result in results] == [
        "six_axis_mapping",
        "servo_tracking",
        "soft_constraint_retreat",
    ]
    assert all(result.passed for result in results)
    for result in results:
        payload = result.to_dict()
        assert payload["name"] == result.name
        assert isinstance(payload["failures"], list)
        for value in payload["metrics"].values():
            if isinstance(value, float):
                assert math.isfinite(value)
