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


def test_fake_half_scale_inverse_hand_target_maps_to_twenty_millimetres() -> None:
    mapper = CoordinateMapper(
        translation_scale=0.5,
        rotation_scale=1.0,
        rotation_dead_zone_deg=0.0,
    )
    anchor = Pose(p=(0.30, 0.20, -0.20), q=(0.0, 0.0, 0.0, 1.0))
    mapper.capture(anchor, anchor)

    target = mapper.target(
        Pose(p=(0.34, 0.20, -0.20), q=(0.0, 0.0, np.sin(np.deg2rad(4)), np.cos(np.deg2rad(4))))
    )

    assert target.p == pytest.approx((0.32, 0.20, -0.20))
    assert Rotation.from_quat(target.q).magnitude() == pytest.approx(np.deg2rad(8))


def test_translation_scale_changes_only_without_an_active_anchor() -> None:
    mapper = CoordinateMapper(translation_scale=1.0, rotation_dead_zone_deg=0.0)
    anchor = Pose(p=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))

    mapper.set_translation_scale(1.5)
    mapper.capture(anchor, anchor)
    target = mapper.target(
        Pose(p=(0.1, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))
    )

    assert mapper.translation_scale == pytest.approx(1.5)
    assert target.p == pytest.approx((0.15, 0.0, 0.0))
    with pytest.raises(RuntimeError, match="translation_scale_requires_no_anchor"):
        mapper.set_translation_scale(1.6)

    mapper.clear()
    mapper.set_translation_scale(1.6)
    assert mapper.translation_scale == pytest.approx(1.6)
