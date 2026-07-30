import numpy as np
import pytest

from app.sim.lm3_model import LM3Model
from app.sim.self_collision import is_self_colliding, segment_distance


FOLDED_COLLISION_Q = np.asarray(
    (-2.9743, -3.6972, 2.7814, 1.3861, 2.4257, -3.7945)
)


def test_segment_distance_handles_crossing_and_parallel_segments() -> None:
    assert segment_distance(
        np.asarray((-1.0, 0.0, 0.0)),
        np.asarray((1.0, 0.0, 0.0)),
        np.asarray((0.0, -1.0, 0.0)),
        np.asarray((0.0, 1.0, 0.0)),
    ) == pytest.approx(0.0)
    assert segment_distance(
        np.asarray((0.0, 0.0, 0.0)),
        np.asarray((1.0, 0.0, 0.0)),
        np.asarray((0.0, 0.2, 0.0)),
        np.asarray((1.0, 0.2, 0.0)),
    ) == pytest.approx(0.2)


def test_home_is_clear_and_known_folded_pose_self_collides() -> None:
    model = LM3Model()

    assert is_self_colliding(model.home_q, model) is False
    assert is_self_colliding(FOLDED_COLLISION_Q, model) is True
