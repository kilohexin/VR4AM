import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.control.coordinate_mapper import CoordinateMapper
from app.schemas.messages import Pose


def test_mapper_uses_scene_translation_and_camera_tool_roll() -> None:
    mapper = CoordinateMapper(
        translation_scale=0.5,
        rotation_scale=1.0,
        rotation_dead_zone_deg=0.0,
    )
    hand_anchor = Pose(p=(0.0, 1.2, -0.3), q=(0.0, 0.0, 0.0, 1.0))
    tcp_rotation = Rotation.from_euler("x", 30.0, degrees=True)
    tcp_anchor = Pose(p=(0.3, 0.4, -0.2), q=tuple(tcp_rotation.as_quat()))
    mapper.capture(
        hand_anchor,
        tcp_anchor,
        tuple(Rotation.from_euler("y", 80.0, degrees=True).as_quat()),
    )
    controller_roll = Rotation.from_rotvec((0.0, 0.0, -0.2))

    target = mapper.target(
        Pose(p=(0.02, 1.16, -0.36), q=tuple(controller_roll.as_quat()))
    )

    assert target.p == pytest.approx((0.31, 0.38, -0.23))
    local_tcp_delta = tcp_rotation.inv() * Rotation.from_quat(target.q)
    np.testing.assert_allclose(local_tcp_delta.as_rotvec(), (0.0, -0.2, 0.0), atol=1e-8)
