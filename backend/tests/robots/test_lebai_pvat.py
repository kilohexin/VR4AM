from __future__ import annotations

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_pvat import PvatLimits, build_pvat_point


LIMITS = PvatLimits(
    horizon_s=0.08,
    max_joint_speed_radps=0.15,
    max_joint_acceleration_radps2=0.5,
    max_joint_tracking_error_rad=0.25,
    soft_joint_min_rad=(-3.0, -2.5, -2.5, -3.0, -2.5, -6.0),
    soft_joint_max_rad=(3.0, 2.5, 2.5, 3.0, 2.5, 6.0),
)
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_small_continuous_solution_produces_consistent_pvat() -> None:
    point = build_pvat_point(
        solution_q=[0.0032, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=LIMITS,
    )

    assert point.q[0] == pytest.approx(0.0032)
    assert point.qd[0] == pytest.approx(0.04)
    assert point.qdd[0] == pytest.approx(0.5)
    assert point.horizon_s == pytest.approx(0.08)


@pytest.mark.parametrize(
    ("solution", "reason"),
    [
        ([0, 0, 0, 0, 0], "ik_invalid"),
        ([float("nan"), 0, 0, 0, 0, 0], "ik_invalid"),
        ([3.01, 0, 0, 0, 0, 0], "ik_joint_limit"),
    ],
)
def test_invalid_or_out_of_bounds_ik_is_rejected(
    solution: list[float],
    reason: str,
) -> None:
    with pytest.raises(BackendCommandError, match=rf"^{reason}$"):
        build_pvat_point(
            solution_q=solution,
            actual_q=ZERO,
            actual_qd=ZERO,
            previous_qd=None,
            limits=LIMITS,
        )


def test_large_safe_tracking_target_is_physically_bounded() -> None:
    point = build_pvat_point(
        solution_q=[0.196, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=LIMITS,
    )

    assert point.q[0] == pytest.approx(0.0032)
    assert point.qd[0] == pytest.approx(0.04)
    assert point.qdd[0] == pytest.approx(0.5)


def test_solution_beyond_tracking_envelope_is_rejected() -> None:
    with pytest.raises(
        BackendCommandError,
        match="^ik_tracking_diverged$",
    ):
        build_pvat_point(
            solution_q=[0.251, 0, 0, 0, 0, 0],
            actual_q=ZERO,
            actual_qd=ZERO,
            previous_qd=None,
            limits=LIMITS,
        )


@pytest.mark.parametrize("solution_delta", [0.0231, -0.0214, -0.0467])
def test_continuous_field_ik_solution_is_speed_and_acceleration_bounded(
    solution_delta: float,
) -> None:
    point = build_pvat_point(
        solution_q=[solution_delta, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=LIMITS,
    )

    expected_sign = 1.0 if solution_delta > 0 else -1.0
    assert point.q[0] == pytest.approx(expected_sign * 0.0032)
    assert point.qd[0] == pytest.approx(expected_sign * 0.04)
    assert point.qdd[0] == pytest.approx(expected_sign * 0.5)


def test_speed_cap_applies_before_acceleration_limit_from_previous_command() -> None:
    point = build_pvat_point(
        solution_q=[0.0231, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=(0.12, 0, 0, 0, 0, 0),
        limits=LIMITS,
    )

    assert point.q[0] == pytest.approx(0.012)
    assert point.qd[0] == pytest.approx(0.15)
    assert point.qdd[0] == pytest.approx(0.375)


def test_speed_limit_preserves_multi_joint_direction() -> None:
    speed_limited = PvatLimits(
        horizon_s=0.08,
        max_joint_speed_radps=0.15,
        max_joint_acceleration_radps2=100.0,
        max_joint_tracking_error_rad=0.25,
        soft_joint_min_rad=LIMITS.soft_joint_min_rad,
        soft_joint_max_rad=LIMITS.soft_joint_max_rad,
    )

    point = build_pvat_point(
        solution_q=[0.02, 0.01, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=speed_limited,
    )

    assert point.q[:2] == pytest.approx((0.012, 0.006))
    assert point.qd[:2] == pytest.approx((0.15, 0.075))
    assert point.qdd[:2] == pytest.approx((1.875, 0.9375))


def test_acceleration_limit_preserves_multi_joint_direction() -> None:
    acceleration_limited = PvatLimits(
        horizon_s=0.08,
        max_joint_speed_radps=1.0,
        max_joint_acceleration_radps2=0.5,
        max_joint_tracking_error_rad=0.25,
        soft_joint_min_rad=LIMITS.soft_joint_min_rad,
        soft_joint_max_rad=LIMITS.soft_joint_max_rad,
    )

    point = build_pvat_point(
        solution_q=[0.008, 0.004, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=acceleration_limited,
    )

    assert point.q[:2] == pytest.approx((0.0032, 0.0016))
    assert point.qd[:2] == pytest.approx((0.04, 0.02))
    assert point.qdd[:2] == pytest.approx((0.5, 0.25))


def test_acceleration_limit_preserves_correction_direction_from_previous_speed() -> None:
    point = build_pvat_point(
        solution_q=[0.012, 0.006, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=(0.12, 0.03, 0, 0, 0, 0),
        limits=LIMITS,
    )

    velocity_change = (
        point.qd[0] - 0.12,
        point.qd[1] - 0.03,
    )
    assert velocity_change == pytest.approx((0.0266666667, 0.04))
    assert point.qd[:2] == pytest.approx((0.1466666667, 0.07))
    assert max(abs(component) for component in point.qd) <= 0.15
    assert max(abs(component) for component in point.qdd) <= 0.5


def test_speed_limit_is_strict_at_floating_point_boundary() -> None:
    speed_limited = PvatLimits(
        horizon_s=0.08,
        max_joint_speed_radps=0.15,
        max_joint_acceleration_radps2=100.0,
        max_joint_tracking_error_rad=0.25,
        soft_joint_min_rad=LIMITS.soft_joint_min_rad,
        soft_joint_max_rad=LIMITS.soft_joint_max_rad,
    )

    point = build_pvat_point(
        solution_q=[-0.045112659100558075, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=speed_limited,
    )

    assert max(abs(component) for component in point.qd) <= 0.15


def test_acceleration_limit_is_strict_at_floating_point_boundary() -> None:
    acceleration_limited = PvatLimits(
        horizon_s=0.13974763565931475,
        max_joint_speed_radps=20.0,
        max_joint_acceleration_radps2=36.52484392478274,
        max_joint_tracking_error_rad=2.0,
        soft_joint_min_rad=(-20.0,) * 6,
        soft_joint_max_rad=(20.0,) * 6,
    )

    point = build_pvat_point(
        solution_q=[1.2781016720224045, -1.3435364708943174, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=acceleration_limited,
    )

    assert max(abs(component) for component in point.qdd) <= 36.52484392478274


def test_acceleration_limit_shortens_target_instead_of_exceeding_limit() -> None:
    point = build_pvat_point(
        solution_q=[0.008, 0, 0, 0, 0, 0],
        actual_q=ZERO,
        actual_qd=ZERO,
        previous_qd=None,
        limits=LIMITS,
    )

    assert point.q[0] == pytest.approx(0.0032)
    assert point.qd[0] == pytest.approx(0.04)
    assert point.qdd[0] == pytest.approx(0.5)


@pytest.mark.parametrize(
    "reference_qd",
    [
        (0.16, 0, 0, 0, 0, 0),
        (float("nan"), 0, 0, 0, 0, 0),
    ],
)
def test_invalid_reference_speed_is_rejected(reference_qd) -> None:
    with pytest.raises(BackendCommandError, match="^joint_speed_limit$"):
        build_pvat_point(
            solution_q=[0.001, 0, 0, 0, 0, 0],
            actual_q=ZERO,
            actual_qd=ZERO,
            previous_qd=reference_qd,
            limits=LIMITS,
        )


def test_measured_joint_speed_over_limit_is_rejected() -> None:
    with pytest.raises(BackendCommandError, match="^joint_speed_limit$"):
        build_pvat_point(
            solution_q=[0.001, 0, 0, 0, 0, 0],
            actual_q=ZERO,
            actual_qd=(0.16, 0, 0, 0, 0, 0),
            previous_qd=ZERO,
            limits=LIMITS,
        )
