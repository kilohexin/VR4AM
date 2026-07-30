from __future__ import annotations

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_pvat import PvatLimits, build_pvat_point


LIMITS = PvatLimits(
    horizon_s=0.08,
    max_joint_speed_radps=0.15,
    max_joint_acceleration_radps2=0.5,
    max_joint_step_rad=0.01,
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
        ([0.02, 0, 0, 0, 0, 0], "ik_joint_jump"),
    ],
)
def test_invalid_or_discontinuous_ik_is_rejected(
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


def test_speed_over_limit_is_rejected_before_send() -> None:
    limits = PvatLimits(
        horizon_s=0.08,
        max_joint_speed_radps=0.15,
        max_joint_acceleration_radps2=0.5,
        max_joint_step_rad=0.02,
        soft_joint_min_rad=LIMITS.soft_joint_min_rad,
        soft_joint_max_rad=LIMITS.soft_joint_max_rad,
    )

    with pytest.raises(BackendCommandError, match="^joint_speed_limit$"):
        build_pvat_point(
            solution_q=[0.013, 0, 0, 0, 0, 0],
            actual_q=ZERO,
            actual_qd=ZERO,
            previous_qd=None,
            limits=limits,
        )


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
