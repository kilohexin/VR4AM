import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.control.safety import SafetyLimiter
from app.schemas.messages import Pose


IDENTITY = (0.0, 0.0, 0.0, 1.0)


def test_safety_limiter_clear_removes_anchor_and_motion_history_in_place() -> None:
    limiter = SafetyLimiter(anchor=(0.3, 0.0, 0.3))
    linear_velocity = limiter.linear_velocity
    angular_velocity = limiter.angular_velocity
    limiter.linear_velocity[:] = (0.1, 0.2, 0.3)
    limiter.angular_velocity[:] = (0.4, 0.5, 0.6)

    limiter.clear()
    limiter.clear()

    assert limiter.anchor is None
    assert limiter.linear_velocity is linear_velocity
    assert limiter.angular_velocity is angular_velocity
    assert np.allclose(limiter.linear_velocity, 0)
    assert np.allclose(limiter.angular_velocity, 0)


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


def test_projects_anchor_envelope_violation_to_soft_boundary() -> None:
    limiter = SafetyLimiter(anchor=(0, 0, 0), workspace_radius=0.25)
    projection = limiter.project_workspace(
        Pose(p=(0.26, 0, 0), q=IDENTITY)
    )

    assert projection.constrained is True
    assert projection.hold is False
    assert projection.pose.p == pytest.approx((0.25, 0.0, 0.0))


def test_optional_box_envelope_constrains_each_axis_without_changing_defaults() -> None:
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.10,
        max_rotation_from_anchor_rad=np.deg2rad(30),
    )
    limiter.set_pose_anchor(Pose(p=(0.3, 0.0, 0.3), q=IDENTITY))

    inside = limiter.project_workspace(
        Pose(p=(0.4, -0.1, 0.2), q=IDENTITY),
    )
    outside = limiter.project_workspace(
        Pose(p=(0.4001, 0.0, 0.3), q=IDENTITY),
    )
    default = SafetyLimiter(anchor=(0.3, 0.0, 0.3)).project_workspace(
        Pose(p=(0.4001, 0.0, 0.3), q=IDENTITY),
    )

    assert inside.constrained is False
    assert outside.constrained is True
    assert outside.hold is True
    assert default.constrained is False


def test_axis_clamp_limits_only_violating_axes_and_preserves_rotation() -> None:
    anchor = Pose(p=(0.3, 0.0, 0.3), q=IDENTITY)
    requested_q = tuple(
        Rotation.from_euler("z", 20, degrees=True).as_quat()
    )
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.10,
        max_rotation_from_anchor_rad=np.deg2rad(45),
        workspace_boundary_mode="axis_clamp",
    )
    limiter.set_pose_anchor(anchor)

    projection = limiter.project_workspace(
        Pose(p=(0.45, 0.04, 0.27), q=requested_q),
    )

    assert projection.constrained is True
    assert projection.hold is False
    assert projection.pose.p == pytest.approx((0.4, 0.04, 0.27))
    assert abs(
        sum(
            actual * expected
            for actual, expected in zip(
                projection.pose.q,
                requested_q,
                strict=True,
            )
        )
    ) == pytest.approx(1.0)


def test_axis_clamp_projects_rotation_and_reverse_request_is_unconstrained() -> None:
    limiter = SafetyLimiter(
        workspace_half_extent_m=0.10,
        max_rotation_from_anchor_rad=np.deg2rad(30),
        workspace_boundary_mode="axis_clamp",
    )
    limiter.set_pose_anchor(Pose(p=(0.3, 0.0, 0.3), q=IDENTITY))
    outside = limiter.project_workspace(
        Pose(
            p=(0.3, 0.0, 0.3),
            q=tuple(
                Rotation.from_euler("z", 45, degrees=True).as_quat()
            ),
        )
    )
    reverse = limiter.project_workspace(
        Pose(
            p=(0.39, 0.0, 0.3),
            q=tuple(
                Rotation.from_euler("z", 25, degrees=True).as_quat()
            ),
        )
    )

    assert outside.constrained is True
    assert outside.hold is False
    assert Rotation.from_quat(outside.pose.q).magnitude() == pytest.approx(
        np.deg2rad(30)
    )
    assert reverse.constrained is False


def test_optional_orientation_envelope_uses_captured_tcp_orientation() -> None:
    anchor_q = tuple(Rotation.from_euler("z", 20, degrees=True).as_quat())
    limiter = SafetyLimiter(max_rotation_from_anchor_rad=np.deg2rad(30))
    limiter.set_pose_anchor(Pose(p=(0.3, 0.0, 0.3), q=anchor_q))

    inside_q = tuple(Rotation.from_euler("z", 50, degrees=True).as_quat())
    outside_q = tuple(Rotation.from_euler("z", 50.1, degrees=True).as_quat())

    assert limiter.project_workspace(
        Pose(p=(0.3, 0.0, 0.3), q=inside_q),
    ).constrained is False
    assert limiter.project_workspace(
        Pose(p=(0.3, 0.0, 0.3), q=outside_q),
    ).constrained is True


def test_optional_hard_per_cycle_caps_apply_even_after_long_dt() -> None:
    limiter = SafetyLimiter(
        max_linear_speed=100,
        max_angular_speed=100,
        max_linear_accel=100,
        max_angular_accel=100,
        max_linear_step_m=0.002,
        max_angular_step_rad=np.deg2rad(1),
    )
    previous = Pose(p=(0, 0, 0), q=IDENTITY)
    requested = Pose(
        p=(1, 0, 0),
        q=tuple(Rotation.from_euler("z", 90, degrees=True).as_quat()),
    )

    actual = limiter.limit_motion(previous, requested, 1.0)

    assert np.linalg.norm(np.asarray(actual.p) - np.asarray(previous.p)) <= 0.002
    delta = Rotation.from_quat(actual.q) * Rotation.from_quat(previous.q).inv()
    assert delta.magnitude() <= np.deg2rad(1)


def test_angular_step_cap_holds_from_non_identity_arbitrary_axis() -> None:
    cap = np.deg2rad(1)
    rng = np.random.default_rng(42)
    for _ in range(256):
        limiter = SafetyLimiter(
            max_linear_speed=100,
            max_angular_speed=100,
            max_linear_accel=100,
            max_angular_accel=100,
            max_angular_step_rad=cap,
        )
        previous_rotation = Rotation.random(random_state=rng)
        axis = rng.normal(size=3)
        axis /= np.linalg.norm(axis)
        requested_rotation = (
            Rotation.from_rotvec(axis * rng.uniform(cap * 2, 3.0))
            * previous_rotation
        )
        previous = Pose(p=(0, 0, 0), q=tuple(previous_rotation.as_quat()))
        requested = Pose(p=(0, 0, 0), q=tuple(requested_rotation.as_quat()))

        actual = limiter.limit_motion(previous, requested, 1.0)

        delta = Rotation.from_quat(actual.q) * previous_rotation.inv()
        assert delta.magnitude() <= cap


def test_high_response_fake_limits_reach_three_hundred_mm_per_second() -> None:
    translated = Pose(p=(0.0, 0.0, 0.0), q=IDENTITY)
    translate_target = Pose(p=(1.0, 0.0, 0.0), q=IDENTITY)
    translation_limiter = SafetyLimiter(
        max_linear_speed=0.30,
        max_linear_accel=1.20,
        max_linear_step_m=0.006,
    )
    positions = []
    for _ in range(50):
        translated = translation_limiter.limit_motion(
            translated,
            translate_target,
            0.02,
        )
        positions.append(translated.p[0])

    rotated = Pose(p=(0.0, 0.0, 0.0), q=IDENTITY)
    rotate_target = Pose(
        p=(0.0, 0.0, 0.0),
        q=tuple(Rotation.from_euler("z", 20, degrees=True).as_quat()),
    )
    rotation_limiter = SafetyLimiter(
        max_angular_speed=1.50,
        max_angular_accel=4.00,
        max_angular_step_rad=np.deg2rad(2.0),
    )
    for _ in range(100):
        rotated = rotation_limiter.limit_motion(
            rotated,
            rotate_target,
            0.02,
        )

    per_tick = np.diff([0.0, *positions])
    assert max(per_tick) <= 0.006 + 1e-12
    assert positions[-1] >= 0.26
    assert per_tick[-1] == pytest.approx(0.006)
    rotation_error = (
        Rotation.from_quat(rotate_target.q)
        * Rotation.from_quat(rotated.q).inv()
    )
    assert rotation_error.magnitude() <= np.deg2rad(2)
