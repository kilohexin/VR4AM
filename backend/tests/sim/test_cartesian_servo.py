import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import Pose
from app.sim.cartesian_servo import cartesian_servo_step
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


def test_position_priority_servo_reaches_one_small_six_dof_target() -> None:
    model = LM3Model()
    q = np.asarray(model.home_q)
    start = forward_pose(q, model)
    start_rotation = Rotation.from_quat(start.q)
    target = Pose(
        p=(start.p[0] + 0.025, start.p[1] - 0.015, start.p[2] - 0.020),
        q=tuple(
            (
                start_rotation
                * Rotation.from_rotvec((0.0, -np.deg2rad(8.0), 0.0))
            ).as_quat()
        ),
    )

    max_step = model.max_joint_speed_radps * 0.02
    result = None
    for _ in range(80):
        previous = q.copy()
        result = cartesian_servo_step(target, q, model, dt=0.02)
        q = np.asarray(result.q)
        assert np.max(np.abs(q - previous)) <= max_step + 1e-12
        assert np.all(np.abs(q - np.asarray(model.home_q)) <= model.joint_window_rad)

    assert result is not None
    assert result.position_error_m < 0.01
    assert result.orientation_error_rad < 0.08


@pytest.mark.asyncio
async def test_sim_adapter_commands_velocity_without_a_second_position_servo() -> None:
    adapter = SimRobotAdapter()
    start = forward_pose(adapter.robot.q, adapter.model)
    target = start.model_copy(
        update={"p": (start.p[0] + 0.03, start.p[1], start.p[2] - 0.02)}
    )

    await adapter.command_tcp(target, command_id=7)

    assert adapter.robot.target_q is None
    assert adapter.robot.target_qd is not None
    assert np.max(np.abs(adapter.robot.target_qd)) <= (
        adapter.model.max_joint_speed_radps + 1e-12
    )
    assert adapter.command_id == 7
