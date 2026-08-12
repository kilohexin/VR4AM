from __future__ import annotations

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose


def test_fake_recovery_candidates_are_ordered_nearest_request_first() -> None:
    from app.robots.lebai_recovery import recovery_candidates

    start = Pose(p=(0.30, 0.00, 0.40), q=(0.0, 0.0, 0.0, 1.0))
    requested = Pose(
        p=(0.34, 0.04, 0.40),
        q=tuple(Rotation.from_euler("z", 40, degrees=True).as_quat()),
    )

    candidates = recovery_candidates(start, requested, fake=True)

    assert [fraction for fraction, _ in candidates] == [1.0, 0.75, 0.5, 0.25]
    assert candidates[1][1].p == pytest.approx((0.33, 0.03, 0.40))
    assert Rotation.from_quat(candidates[1][1].q).magnitude() == pytest.approx(
        np.deg2rad(30)
    )


def test_real_or_missing_history_attempts_only_original_target() -> None:
    from app.robots.lebai_recovery import recovery_candidates

    requested = Pose(p=(0.34, 0.04, 0.40), q=(0.0, 0.0, 0.0, 1.0))

    assert recovery_candidates(None, requested, fake=True) == (
        (1.0, requested),
    )
    assert recovery_candidates(requested, requested, fake=False) == (
        (1.0, requested),
    )
