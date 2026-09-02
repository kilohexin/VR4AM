from __future__ import annotations

import math

import pytest

from app.robots.base import BackendCommandError
from app.robots.lebai_ik_policy import (
    evaluate_ik_candidate,
    proportional_recovery_fraction,
)


ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_tracking_lag_is_not_adjacent_solution_step() -> None:
    metrics = evaluate_ik_candidate(
        solution_q=(0.0, -1.52, 0.1622, -1.5568, 0.4041, 0.0294),
        actual_q=(0.0, -1.567, 0.2495, -1.5721, 0.4016, -0.0011),
        previous_solution_q=(
            0.0,
            -1.54,
            0.1983,
            -1.5648,
            0.4032,
            0.0189,
        ),
    )

    assert metrics.solution_step_rad == pytest.approx(0.0361, abs=1e-4)
    assert metrics.tracking_error_rad == pytest.approx(0.0873, abs=1e-4)


def test_initial_solution_uses_actual_joint_reference() -> None:
    metrics = evaluate_ik_candidate(
        solution_q=(0.04, 0.0, 0.0, 0.0, 0.0, 0.0),
        actual_q=ZERO,
        previous_solution_q=None,
    )

    assert metrics.solution_q == (0.04, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert metrics.solution_step_rad == pytest.approx(0.04)
    assert metrics.tracking_error_rad == pytest.approx(0.04)


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    [
        ("solution_q", (0.0,) * 5, "ik_invalid"),
        (
            "solution_q",
            (0.0, 0.0, 0.0, 0.0, 0.0, math.nan),
            "ik_invalid",
        ),
        (
            "actual_q",
            (0.0, 0.0, 0.0, 0.0, math.inf, 0.0),
            "invalid_actual_joint_state",
        ),
        (
            "previous_solution_q",
            (0.0, 0.0, 0.0, 0.0, 0.0, math.nan),
            "invalid_previous_ik_solution",
        ),
    ],
)
def test_invalid_joint_vector_is_rejected(
    field: str,
    value: object,
    reason: str,
) -> None:
    arguments: dict[str, object] = {
        "solution_q": ZERO,
        "actual_q": ZERO,
        "previous_solution_q": ZERO,
    }
    arguments[field] = value

    with pytest.raises(BackendCommandError, match=rf"^{reason}$"):
        evaluate_ik_candidate(**arguments)  # type: ignore[arg-type]


def test_recovery_fraction_has_inward_margin() -> None:
    assert proportional_recovery_fraction(0.10, 0.05) == pytest.approx(0.4)
    assert proportional_recovery_fraction(0.08, 0.05, 0.4) == pytest.approx(
        0.2
    )


def test_recovery_fraction_is_capped_for_a_small_measured_step() -> None:
    assert proportional_recovery_fraction(0.01, 0.05) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("step", "limit", "previous", "message"),
    [
        (0.0, 0.05, None, "solution_step_must_be_positive"),
        (math.nan, 0.05, None, "solution_step_must_be_positive"),
        (0.10, 0.0, None, "max_solution_step_must_be_positive"),
        (0.10, math.inf, None, "max_solution_step_must_be_positive"),
        (0.10, 0.05, 0.0, "previous_fraction_out_of_range"),
        (0.10, 0.05, 1.0, "previous_fraction_out_of_range"),
        (0.10, 0.05, math.nan, "previous_fraction_out_of_range"),
    ],
)
def test_recovery_fraction_rejects_invalid_inputs(
    step: float,
    limit: float,
    previous: float | None,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=rf"^{message}$"):
        proportional_recovery_fraction(step, limit, previous)
