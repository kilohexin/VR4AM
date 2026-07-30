import numpy as np
import pytest

from app.sim.lm3_model import LM3Model
from app.sim.virtual_robot import VirtualRobot


def test_virtual_robot_moves_over_time_without_jumping() -> None:
    model = LM3Model()
    robot = VirtualRobot(model)
    goal_q = np.asarray(model.home_q) + 0.2
    dt = 0.02

    robot.set_target_q(goal_q)
    robot.step(dt)

    position_acceleration = model.max_joint_accel_radps2 / 2.0
    assert np.max(np.abs(np.asarray(robot.q) - np.asarray(model.home_q))) <= (
        position_acceleration * dt**2 + 1e-12
    )
    for _ in range(600):
        robot.step(0.02)
    assert robot.q == pytest.approx(goal_q, abs=2e-3)


def test_velocity_mode_accelerates_to_and_integrates_the_requested_speed_once() -> None:
    robot = VirtualRobot(LM3Model())
    requested = np.full(6, 0.6)
    robot.set_target_qd(requested)

    for _ in range(25):
        robot.step(0.02)

    assert robot.target_q is None
    assert robot.target_qd == pytest.approx(requested)
    assert robot.qd == pytest.approx(requested)
    assert np.min(robot.q - np.asarray(robot.model.home_q)) > 0.15


def test_virtual_robot_respects_speed_and_acceleration_limits() -> None:
    model = LM3Model()
    robot = VirtualRobot(model)
    robot.set_target_q(np.asarray(model.home_q) + 1.0)
    previous_qd = np.asarray(robot.qd).copy()

    for _ in range(200):
        robot.step(0.02)
        qd = np.asarray(robot.qd)
        assert np.max(np.abs(qd)) <= model.max_joint_speed_radps + 1e-12
        acceleration = (qd - previous_qd) / 0.02
        assert np.max(np.abs(acceleration)) <= model.max_joint_accel_radps2 + 1e-12
        previous_qd = qd.copy()


def test_retarget_and_final_convergence_remain_acceleration_limited() -> None:
    model = LM3Model()
    robot = VirtualRobot(model)
    dt = 0.02
    position_acceleration = model.max_joint_accel_radps2 / 2.0
    max_acceleration = position_acceleration + 1e-12
    robot.set_target_q(np.asarray(model.home_q) + 1.0)

    for _ in range(24):
        previous_qd = robot.qd.copy()
        robot.step(dt)
        assert np.max(np.abs((robot.qd - previous_qd) / dt)) <= max_acceleration

    expected_velocity = min(
        24 * position_acceleration * dt,
        model.max_joint_speed_radps,
    )
    assert robot.qd == pytest.approx([expected_velocity] * 6)
    next_limited_qd = robot.qd - position_acceleration * dt
    retarget = robot.q + next_limited_qd * dt
    robot.set_target_q(retarget)

    for _ in range(1000):
        previous_qd = robot.qd.copy()
        robot.step(dt)
        assert np.max(np.abs((robot.qd - previous_qd) / dt)) <= max_acceleration
        if np.max(np.abs(robot.q - retarget)) < 1e-12 and np.all(robot.qd == 0.0):
            break
    else:
        pytest.fail("virtual robot did not converge after retargeting")


def test_virtual_robot_is_deterministic_for_the_same_steps() -> None:
    model = LM3Model()
    first = VirtualRobot(model)
    second = VirtualRobot(model)
    target = np.asarray(model.home_q) + np.linspace(-0.3, 0.3, 6)
    first.set_target_q(target)
    second.set_target_q(target)

    for _ in range(150):
        first.step(0.02)
        second.step(0.02)

    assert np.array_equal(first.q, second.q)
    assert np.array_equal(first.qd, second.qd)


@pytest.mark.parametrize(
    "target",
    [
        np.asarray(LM3Model().home_q) + np.array([np.pi + 1e-9, 0, 0, 0, 0, 0]),
        [0.0] * 5,
        [float("nan")] * 6,
    ],
)
def test_virtual_robot_rejects_targets_outside_session_window(target: object) -> None:
    robot = VirtualRobot(LM3Model())

    with pytest.raises(ValueError, match="joint_safety_window"):
        robot.set_target_q(target)


def test_stop_invalidates_target_and_decelerates_to_zero() -> None:
    robot = VirtualRobot(LM3Model())
    robot.set_target_q(np.asarray(robot.model.home_q) + 0.5)
    for _ in range(20):
        robot.step(0.02)

    robot.stop()
    for _ in range(30):
        robot.step(0.02)

    assert robot.target_q is None
    assert robot.qd == pytest.approx([0] * 6, abs=1e-8)
