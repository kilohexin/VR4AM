import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper
from app.schemas.messages import Pose


def test_mapper_applies_fixed_one_to_one_world_pose_delta_without_a_jump() -> None:
    mapper = CoordinateMapper(
        translation_scale=1.0,
        rotation_scale=1.0,
        rotation_dead_zone_deg=0.0,
    )
    hand_rotation = Rotation.from_euler("xyz", (15, -20, 10), degrees=True)
    tcp_rotation = Rotation.from_euler("xyz", (-30, 5, 40), degrees=True)
    hand_anchor = Pose(p=(0.0, 1.2, -0.3), q=tuple(hand_rotation.as_quat()))
    tcp_anchor = Pose(p=(0.3, 0.4, -0.2), q=tuple(tcp_rotation.as_quat()))
    mapper.capture(hand_anchor, tcp_anchor)
    delta = Rotation.from_euler("xyz", (12, -8, 20), degrees=True)

    target = mapper.target(
        Pose(
            p=(0.10, 1.15, -0.22),
            q=tuple((delta * hand_rotation).as_quat()),
        )
    )

    assert target.p == pytest.approx((0.40, 0.35, -0.12))
    np.testing.assert_allclose(
        Rotation.from_quat(target.q).as_matrix(),
        (delta * tcp_rotation).as_matrix(),
        atol=1e-8,
    )
