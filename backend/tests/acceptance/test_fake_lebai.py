import asyncio
import math

import pytest

from app.acceptance.fake_lebai import (
    fake_real_harness,
    run_fake_lebai_scenarios,
    run_fault_scenario,
    run_translation_scenario,
)
from app.commissioning.actions import RotationAction, TranslationAction
from app.control.robot_control import RobotControl
from app.robots.lebai_adapter import RealLebaiAdapter


def test_fake_lebai_scenarios_cover_every_required_axis_and_action() -> None:
    results = asyncio.run(run_fake_lebai_scenarios())
    by_name = {result.name: result for result in results}
    assert set(by_name) == {
        "translation",
        "rotation",
        "gripper_home_stop",
        "faults",
    }
    assert by_name["translation"].metrics["axes"] == [
        "+x",
        "-x",
        "+y",
        "-y",
        "+z",
        "-z",
    ]
    assert by_name["rotation"].metrics["axes"] == [
        "+roll",
        "-roll",
        "+pitch",
        "-pitch",
        "+yaw",
        "-yaw",
    ]
    assert (
        by_name["gripper_home_stop"].metrics[
            "no_writes_after_final_stop"
        ]
        is True
    )
    assert by_name["faults"].metrics["injected"] == len(
        by_name["faults"].metrics["cases"]
    )
    assert by_name["faults"].metrics["verified"] == len(
        by_name["faults"].metrics["cases"]
    )
    assert all(result.passed for result in results)


def test_translation_result_reports_motion_verification_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    async def too_small_motion(_action: object) -> tuple[float, float, int]:
        return 1e-8, 0.0, 1

    monkeypatch.setattr(
        "app.acceptance.fake_lebai._run_signed_motion",
        too_small_motion,
    )

    result = asyncio.run(run_translation_scenario())

    assert result.passed is False
    assert len(result.failures) == 6
    assert all("magnitude" in failure for failure in result.failures)


def test_fault_result_derives_verified_count_from_successful_cases(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    async def failed_pvat_case() -> int:
        raise AssertionError("missing hard stop")

    monkeypatch.setattr(
        "app.acceptance.fake_lebai._run_pvat_failure_case",
        failed_pvat_case,
    )

    result = asyncio.run(run_fault_scenario())

    assert result.passed is False
    assert result.metrics["injected"] == len(result.metrics["cases"])
    assert result.metrics["verified"] == 4
    assert result.failures == ("pvat_failure:missing hard stop",)


def test_full_write_log_quiescence_detects_any_late_write() -> None:
    from app.acceptance.fake_lebai import _write_log_is_quiescent

    final_log = (("set_claw", 30, 0), ("stop_move",))

    assert _write_log_is_quiescent(final_log, final_log) is True
    assert (
        _write_log_is_quiescent(
            final_log,
            final_log + (("set_claw", 30, 100),),
        )
        is False
    )
    assert (
        _write_log_is_quiescent(
            final_log,
            final_log + (("stop_move",),),
        )
        is False
    )


def test_motion_quality_requires_commanded_axis_dominance() -> None:
    from app.acceptance.fake_lebai import _motion_quality_failures

    translation_failures = _motion_quality_failures(
        TranslationAction("x", 0.005),
        commanded=6e-6,
        uncommanded=4e-7,
    )
    rotation_failures = _motion_quality_failures(
        RotationAction("yaw", 2.0),
        commanded=math.radians(0.001),
        uncommanded=math.radians(0.00003),
    )

    assert translation_failures == ["commanded_axis_not_dominant"]
    assert rotation_failures == ["commanded_axis_not_dominant"]


def test_harness_disconnects_when_control_connect_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    disconnect_calls = 0
    real_disconnect = RealLebaiAdapter.disconnect

    async def failed_control_connect(_control: RobotControl) -> None:
        raise RuntimeError("control connect failed")

    async def observed_disconnect(adapter: RealLebaiAdapter) -> None:
        nonlocal disconnect_calls
        disconnect_calls += 1
        await real_disconnect(adapter)

    monkeypatch.setattr(RobotControl, "connect", failed_control_connect)
    monkeypatch.setattr(
        RealLebaiAdapter,
        "disconnect",
        observed_disconnect,
    )

    async def exercise() -> None:
        with pytest.raises(RuntimeError, match="control connect failed"):
            async with fake_real_harness():
                raise AssertionError("harness must not yield")

    asyncio.run(exercise())

    assert disconnect_calls == 1
