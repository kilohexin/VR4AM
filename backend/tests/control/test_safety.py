import pytest
from scipy.spatial.transform import Rotation

from app.control.safety import SafetyLimiter, SafetyViolation
from app.schemas.messages import Pose


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def test_first_translation_tick_obeys_acceleration_limit() -> None:
    limiter = SafetyLimiter()
    previous = Pose(p=(0, 0, 0), q=IDENTITY)
    requested = Pose(p=(1, 0, 0), q=IDENTITY)
    assert limiter.limit(previous, requested, 0.02).p == pytest.approx((0.00016, 0, 0))


def test_first_rotation_tick_obeys_angular_acceleration_limit() -> None:
    limiter = SafetyLimiter()
    previous = Pose(p=(0, 0, 0), q=IDENTITY)
    requested = Pose(p=(0, 0, 0), q=tuple(Rotation.from_rotvec([0, 0, 1]).as_quat()))
    actual = Rotation.from_quat(limiter.limit(previous, requested, 0.02).q)
    assert actual.magnitude() == pytest.approx(0.00048)


def test_linear_and_angular_speed_limits_apply_on_same_tick() -> None:
    limiter = SafetyLimiter(max_linear_accel=100, max_angular_accel=100)
    previous = Pose(p=(0, 0, 0), q=IDENTITY)
    requested = Pose(p=(1, 0, 0), q=tuple(Rotation.from_rotvec([0, 0, 1]).as_quat()))

    actual = limiter.limit(previous, requested, 0.02)

    assert actual.p == pytest.approx((0.003, 0, 0))
    assert Rotation.from_quat(actual.q).magnitude() == pytest.approx(0.012)


@pytest.mark.parametrize("invalid_dt", [0.0, -0.02, float("nan"), float("inf")])
def test_rejects_invalid_dt_without_polluting_velocity_state(invalid_dt: float) -> None:
    limiter = SafetyLimiter()
    previous = Pose(p=(0, 0, 0), q=IDENTITY)
    requested = Pose(p=(1, 0, 0), q=tuple(Rotation.from_rotvec([0, 0, 1]).as_quat()))

    with pytest.raises(ValueError, match="dt_must_be_positive_finite"):
        limiter.limit(previous, requested, invalid_dt)

    actual = limiter.limit(previous, requested, 0.02)
    expected = SafetyLimiter().limit(previous, requested, 0.02)
    assert actual.p == pytest.approx(expected.p)
    assert abs(sum(a * b for a, b in zip(actual.q, expected.q, strict=True))) == pytest.approx(1.0)


def test_rejects_anchor_envelope_violation() -> None:
    limiter = SafetyLimiter(anchor=(0, 0, 0))
    with pytest.raises(SafetyViolation, match="workspace_violation") as exc_info:
        limiter.limit(Pose(p=(0, 0, 0), q=IDENTITY), Pose(p=(0.26, 0, 0), q=IDENTITY), 0.02)
    assert exc_info.value.code == "workspace_violation"
