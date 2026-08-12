from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from app.schemas.messages import Pose

RECOVERY_FRACTIONS: tuple[float, ...] = (1.0, 0.75, 0.5, 0.25)


def interpolate_pose(start: Pose, requested: Pose, fraction: float) -> Pose:
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("fraction_must_be_between_zero_and_one")
    start_position = np.asarray(start.p, dtype=float)
    requested_position = np.asarray(requested.p, dtype=float)
    position = start_position + fraction * (
        requested_position - start_position
    )
    rotation = Slerp(
        [0.0, 1.0],
        Rotation.from_quat([start.q, requested.q]),
    )([fraction])[0]
    return Pose(p=tuple(position), q=tuple(rotation.as_quat()))


def recovery_candidates(
    last_safe: Pose | None,
    requested: Pose,
    *,
    fake: bool,
) -> tuple[tuple[float, Pose], ...]:
    if not fake or last_safe is None:
        return ((1.0, requested),)
    return tuple(
        (fraction, interpolate_pose(last_safe, requested, fraction))
        for fraction in RECOVERY_FRACTIONS
    )
