from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from app.sim.kinematics import chain_points
from app.sim.lm3_model import LM3Model


def _point(value: Sequence[float]) -> np.ndarray:
    point = np.asarray(value, dtype=float)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("invalid_self_collision_geometry")
    return point


def segment_distance(
    a0: Sequence[float],
    a1: Sequence[float],
    b0: Sequence[float],
    b1: Sequence[float],
) -> float:
    try:
        first_start = _point(a0)
        first_end = _point(a1)
        second_start = _point(b0)
        second_end = _point(b1)

        first_direction = first_end - first_start
        second_direction = second_end - second_start
        offset = first_start - second_start
        first_length_sq = float(np.dot(first_direction, first_direction))
        second_length_sq = float(
            np.dot(second_direction, second_direction)
        )
        epsilon = np.finfo(float).eps

        if first_length_sq <= epsilon and second_length_sq <= epsilon:
            distance = float(np.linalg.norm(offset))
        elif first_length_sq <= epsilon:
            second_fraction = float(
                np.clip(
                    np.dot(second_direction, offset)
                    / second_length_sq,
                    0.0,
                    1.0,
                )
            )
            distance = float(
                np.linalg.norm(
                    offset - second_fraction * second_direction
                )
            )
        elif second_length_sq <= epsilon:
            first_fraction = float(
                np.clip(
                    -np.dot(first_direction, offset) / first_length_sq,
                    0.0,
                    1.0,
                )
            )
            distance = float(
                np.linalg.norm(
                    offset + first_fraction * first_direction
                )
            )
        else:
            cross = float(np.dot(first_direction, second_direction))
            first_offset = float(np.dot(first_direction, offset))
            second_offset = float(np.dot(second_direction, offset))
            denominator = (
                first_length_sq * second_length_sq - cross * cross
            )
            if denominator > epsilon:
                first_fraction = float(
                    np.clip(
                        (
                            cross * second_offset
                            - second_length_sq * first_offset
                        )
                        / denominator,
                        0.0,
                        1.0,
                    )
                )
            else:
                first_fraction = 0.0

            second_fraction = (
                cross * first_fraction + second_offset
            ) / second_length_sq
            if second_fraction < 0.0:
                second_fraction = 0.0
                first_fraction = float(
                    np.clip(
                        -first_offset / first_length_sq,
                        0.0,
                        1.0,
                    )
                )
            elif second_fraction > 1.0:
                second_fraction = 1.0
                first_fraction = float(
                    np.clip(
                        (cross - first_offset) / first_length_sq,
                        0.0,
                        1.0,
                    )
                )
            distance = float(
                np.linalg.norm(
                    offset
                    + first_fraction * first_direction
                    - second_fraction * second_direction
                )
            )
        if not np.isfinite(distance):
            raise ValueError("invalid_self_collision_geometry")
        return distance
    except (TypeError, ValueError, FloatingPointError, OverflowError) as exc:
        raise ValueError("invalid_self_collision_geometry") from exc


def is_self_colliding(
    q: Sequence[float],
    model: LM3Model,
) -> bool:
    try:
        points = chain_points(q, model)
        segments = {
            segment.name: (
                points[segment.start],
                points[segment.end],
                segment.radius_m,
            )
            for segment in model.collision_segments
        }
        for first_name, second_name in model.collision_check_pairs:
            first_start, first_end, first_radius = segments[first_name]
            second_start, second_end, second_radius = segments[second_name]
            distance = segment_distance(
                first_start,
                first_end,
                second_start,
                second_end,
            )
            threshold = (
                first_radius
                + second_radius
                + model.collision_safety_margin_m
            )
            if not np.isfinite(threshold):
                raise ValueError("invalid_self_collision_geometry")
            if distance < threshold:
                return True
        return False
    except (KeyError, TypeError, ValueError, FloatingPointError, OverflowError) as exc:
        raise ValueError("invalid_self_collision_geometry") from exc
