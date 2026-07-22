import numpy as np
import pytest

from app.sim.kinematics import forward_matrix, forward_pose, geometric_jacobian
from app.sim.lm3_model import LM3Model


def test_shared_glb_fk_and_jacobian_are_consistent() -> None:
    model = LM3Model()
    q = np.asarray(model.home_q)
    matrix = forward_matrix(q, model)
    jacobian = geometric_jacobian(q, model)
    dq = np.array([1e-5, -2e-5, 1e-5, 0.0, 1e-5, 0.0])
    actual_delta = np.asarray(forward_pose(q + dq, model).p) - matrix[:3, 3]

    assert model.joint_names == tuple(f"Joint{i}" for i in range(1, 7))
    assert model.joint_axes == ("y", "z", "z", "z", "y", "z")
    assert matrix.shape == (4, 4)
    assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
    assert jacobian.shape == (6, 6)
    np.testing.assert_allclose(actual_delta, jacobian[:3] @ dq, atol=1e-8)
