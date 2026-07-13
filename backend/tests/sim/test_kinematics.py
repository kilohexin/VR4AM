import numpy as np
import pytest

from app.sim.kinematics import forward_matrix, forward_pose
from app.sim.lm3_model import LM3Model


def test_fk_returns_rigid_transform_and_pose() -> None:
    model = LM3Model()
    matrix = forward_matrix(model.home_q, model)
    pose = forward_pose(model.home_q, model)
    assert matrix.shape == (4, 4)
    assert np.allclose(matrix[3], [0, 0, 0, 1])
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    assert sum(value * value for value in pose.q) == pytest.approx(1.0)


def test_fk_rejects_wrong_joint_count() -> None:
    with pytest.raises(ValueError, match="six joints"):
        forward_pose([0, 0], LM3Model())


def test_fk_is_repeatable() -> None:
    q = [0.2, -0.5, 0.8, -0.2, 0.4, 0.1]
    assert forward_pose(q, LM3Model()) == forward_pose(q, LM3Model())
