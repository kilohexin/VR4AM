import json
from unittest.mock import AsyncMock

import pytest

from app.robots.lebai_adapter import RealLebaiAdapter
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings


@pytest.mark.parametrize("enabled", [False, True])
async def test_raw_tail_is_opt_in_and_reuses_existing_read_without_writes(enabled):
    client = FakeLebaiClient.idle()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
                               stop_diagnostic_sampling=enabled)
    await adapter.connect()
    adapter._latch_unverified_stop()
    client.robot_state = "STOP"
    client.kin_data["actual_joint_torque"] = [1., 2., 3., 4., 5., 6.]
    client.kin_data["target_joint_torque"] = [6., 5., 4., 3., 2., 1.]
    client.read_calls.clear()
    try:
        state = await adapter.read_stop_observation()
        assert client.read_calls == ["is_connected", "get_robot_state", "get_estop_reason",
                                     "get_kin_data", "get_tcp", "get_claw", "get_running_motion"]
        assert client.write_calls == []
        assert state["latched_fault"] == "stop_unverified"
        assert not adapter._preflight_ready
        assert ("raw_kinematics" in state) is enabled
        if enabled:
            assert state["raw_kinematics"]["actual_joint_torque"] == [1., 2., 3., 4., 5., 6.]
            assert state["raw_kinematics"]["target_joint_torque"] == [6., 5., 4., 3., 2., 1.]
            json.dumps(state, allow_nan=False)
            client.kin_data["actual_joint_torque"][0] = 99.
            assert state["raw_kinematics"]["actual_joint_torque"][0] == 1.
    finally:
        await adapter.disconnect()


async def test_missing_torque_fields_are_preserved_as_missing_not_zero():
    client = FakeLebaiClient.idle()
    del client.kin_data["actual_joint_torque"]
    del client.kin_data["target_joint_torque"]
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
                               stop_diagnostic_sampling=True)
    await adapter.connect()
    try:
        state = await adapter.read_stop_observation()
        assert "actual_joint_torque" not in state["raw_kinematics"]
        assert "target_joint_torque" not in state["raw_kinematics"]
    finally:
        await adapter.disconnect()


async def test_raw_read_is_copied_before_later_sdk_calls_can_mutate_shared_data():
    client = FakeLebaiClient.idle()
    adapter = RealLebaiAdapter(control_settings(), client_factory=AsyncMock(return_value=client),
                               stop_diagnostic_sampling=True)
    await adapter.connect()
    original = client.get_tcp

    async def mutate_after_kinematics():
        client.kin_data["actual_joint_torque"][0] = 17.
        return await original()

    client.get_tcp = mutate_after_kinematics
    try:
        state = await adapter.read_stop_observation()
        assert state["raw_kinematics"]["actual_joint_torque"][0] == 0.
    finally:
        await adapter.disconnect()
