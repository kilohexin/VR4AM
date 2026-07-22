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


def test_home_keeps_tcp_and_forward_while_camera_is_up_and_jaws_are_horizontal() -> None:
    model = LM3Model()
    legacy_q = np.asarray((0.0, -np.pi / 4, np.pi / 2, -np.pi / 4, np.pi / 2, 0.0))
    home = forward_matrix(model.home_q, model)
    legacy = forward_matrix(legacy_q, model)
    camera_up = home[:3, :3] @ np.asarray((-1.0, 0.0, 0.0))
    jaw_axis = home[:3, :3] @ np.asarray((0.0, 0.0, 1.0))
    forward = home[:3, :3] @ np.asarray((0.0, -1.0, 0.0))
    legacy_forward = legacy[:3, :3] @ np.asarray((0.0, -1.0, 0.0))

    assert model.home_q[5] == pytest.approx(-np.pi / 2)
    np.testing.assert_allclose(home[:3, 3], legacy[:3, 3], atol=1e-9)
    np.testing.assert_allclose(forward, legacy_forward, atol=1e-9)
    np.testing.assert_allclose(camera_up, (0.0, 1.0, 0.0), atol=1e-8)
    assert jaw_axis[1] == pytest.approx(0.0, abs=1e-8)
