from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from app.robots.base import BackendCommandError
from app.schemas.messages import JointVector


@dataclass(frozen=True)
class IkCandidateMetrics:
    solution_q: JointVector
    solution_step_rad: float
    tracking_error_rad: float


def evaluate_ik_candidate(
    solution_q: object,
    actual_q: JointVector,
    previous_solution_q: JointVector | None,
) -> IkCandidateMetrics:
    solution = _vector(solution_q, "ik_invalid")
    actual = _vector(actual_q, "invalid_actual_joint_state")
    reference = (
        actual
        if previous_solution_q is None
        else _vector(
            previous_solution_q,
            "invalid_previous_ik_solution",
        )
    )
    return IkCandidateMetrics(
        solution_q=_joint_tuple(solution),
        solution_step_rad=float(np.max(np.abs(solution - reference))),
        tracking_error_rad=float(np.max(np.abs(solution - actual))),
    )


def proportional_recovery_fraction(
    solution_step_rad: float,
    max_solution_step_rad: float,
    previous_fraction: float | None = None,
) -> float:
    step = _positive_finite(
        solution_step_rad,
        "solution_step_must_be_positive",
    )
    limit = _positive_finite(
        max_solution_step_rad,
        "max_solution_step_must_be_positive",
    )
    if previous_fraction is None:
        return min(1.0, 0.8 * limit / step)
    try:
        prior = float(previous_fraction)
    except (TypeError, ValueError):
        raise ValueError("previous_fraction_out_of_range") from None
    if not math.isfinite(prior) or not 0.0 < prior < 1.0:
        raise ValueError("previous_fraction_out_of_range")
    return prior * 0.5


def _vector(value: object, reason: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        raise BackendCommandError(reason) from None
    if vector.shape != (6,) or not np.all(np.isfinite(vector)):
        raise BackendCommandError(reason)
    return vector


def _positive_finite(value: object, message: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(message) from None
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(message)
    return result


def _joint_tuple(value: np.ndarray) -> JointVector:
    return tuple(float(component) for component in value)  # type: ignore[return-value]
