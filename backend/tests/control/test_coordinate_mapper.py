import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper, DEFAULT_R_BX
from app.schemas.messages import Pose

IDENTITY = (0.0, 0.0, 0.0, 1.0)


def pose(p=(0.0, 0.0, 0.0), q=IDENTITY) -> Pose:
    return Pose(p=p, q=q)


def test_default_mapping_is_proper_rotation() -> None:
    assert np.allclose(DEFAULT_R_BX.T @ DEFAULT_R_BX, np.eye(3))
    assert np.linalg.det(DEFAULT_R_BX) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("hand_delta", "robot_delta"),
    [((0, 0, -0.1), (0.08, 0, 0)), ((0.1, 0, 0), (0, -0.08, 0)), ((0, 0.1, 0), (0, 0, 0.08))],
)
def test_default_axes_and_translation_scale(hand_delta, robot_delta) -> None:
    mapper = CoordinateMapper(translation_scale=0.8)
    mapper.capture(pose(), pose((0.3, 0.0, 0.2)))
    assert mapper.target(pose(hand_delta)).p == pytest.approx(tuple(np.add((0.3, 0.0, 0.2), robot_delta)))


def test_capture_makes_current_hand_equal_current_tcp() -> None:
    hand = pose((1, 2, 3), Rotation.from_euler("x", 20, degrees=True).as_quat())
    tcp = pose((0.2, 0.1, 0.4), Rotation.from_euler("z", 30, degrees=True).as_quat())
    mapper = CoordinateMapper()
    mapper.capture(hand, tcp)
    target = mapper.target(hand)
    assert target.p == pytest.approx(tcp.p)
    assert abs(np.dot(target.q, tcp.q)) == pytest.approx(1.0)


def test_target_requires_anchor() -> None:
    with pytest.raises(RuntimeError, match="anchor_not_captured"):
        CoordinateMapper().target(pose())
