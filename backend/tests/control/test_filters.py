import pytest
from scipy.spatial.transform import Rotation

from app.control.filters import PoseFilter
from app.schemas.messages import Pose


def test_pose_filter_reset_has_no_jump() -> None:
    value = Pose(p=(1, 2, 3), q=(0, 0, 0, 1))
    pose_filter = PoseFilter()

    pose_filter.reset(value)

    assert pose_filter.update(value, 0.02) == value


def test_pose_filter_moves_toward_position_and_rotation() -> None:
    pose_filter = PoseFilter(cutoff_hz=8)
    pose_filter.reset(Pose(p=(0, 0, 0), q=(0, 0, 0, 1)))
    target = Pose(
        p=(1, 0, 0),
        q=tuple(Rotation.from_euler("z", 90, degrees=True).as_quat()),
    )

    actual = pose_filter.update(target, 0.02)

    assert 0 < actual.p[0] < 1
    assert 0 < Rotation.from_quat(actual.q).magnitude() < Rotation.from_quat(target.q).magnitude()


@pytest.mark.parametrize("invalid_dt", [0.0, -0.02, float("nan"), float("inf")])
def test_pose_filter_rejects_invalid_dt(invalid_dt: float) -> None:
    pose_filter = PoseFilter()
    pose_filter.reset(Pose(p=(0, 0, 0), q=(0, 0, 0, 1)))

    with pytest.raises(ValueError, match="dt_must_be_positive_finite"):
        pose_filter.update(Pose(p=(1, 0, 0), q=(0, 0, 0, 1)), invalid_dt)
