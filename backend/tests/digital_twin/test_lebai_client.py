from __future__ import annotations

import numpy as np
import pytest

from app.digital_twin.lebai_client import DigitalTwinFaults, DigitalTwinLebaiClient
from app.robots.lebai_sdk_bridge import detect_capabilities
from tests.robots.fake_lebai import ACTUAL_TCP, IDLE_Q
from tests.robots.real_settings import control_settings


class FakeNanosecondClock:
    def __init__(self) -> None:
        self.value = 0

    def now_ns(self) -> int:
        return self.value

    def advance_seconds(self, seconds: float) -> None:
        self.value += round(seconds * 1_000_000_000)


@pytest.mark.asyncio
async def test_pvat_changes_actual_state_only_after_virtual_time_advances() -> None:
    clock = FakeNanosecondClock()
    settings = control_settings()
    client = DigitalTwinLebaiClient.idle(settings, clock=clock.now_ns)
    before = await client.get_kin_data()
    q = np.asarray(before["actual_joint_pose"], dtype=float)
    target_q = (q + np.array([0.01, -0.01, 0.01, 0, 0, 0])).tolist()

    await client.move_pvat(target_q, [0.1] * 6, [0.0] * 6, 0.08)
    immediate = await client.get_kin_data()
    assert immediate["actual_joint_pose"] == pytest.approx(q)

    clock.advance_seconds(0.04)
    halfway = await client.get_kin_data()
    assert np.linalg.norm(np.asarray(halfway["actual_joint_pose"]) - q) > 0
    assert np.linalg.norm(np.asarray(halfway["actual_joint_pose"]) - target_q) > 0

    clock.advance_seconds(0.04)
    final = await client.get_kin_data()
    assert final["actual_joint_pose"] == pytest.approx(target_q)
    assert final["actual_joint_speed"] == pytest.approx([0.0] * 6)


@pytest.mark.asyncio
async def test_digital_twin_implements_every_detected_sdk_capability() -> None:
    client = DigitalTwinLebaiClient.idle(control_settings())
    assert detect_capabilities(client).control_ready is True


@pytest.mark.asyncio
async def test_ik_failure_returns_none_without_mutating_state() -> None:
    client = DigitalTwinLebaiClient.idle(
        control_settings(),
        faults=DigitalTwinFaults(ik_failure=True),
    )
    before = await client.get_kin_data()
    assert await client.kinematics_inverse(ACTUAL_TCP, list(IDLE_Q)) is None
    assert await client.get_kin_data() == before


@pytest.mark.asyncio
async def test_disconnect_rejects_reads_and_writes() -> None:
    client = DigitalTwinLebaiClient.idle(
        control_settings(),
        faults=DigitalTwinFaults(disconnect=True),
    )
    assert await client.is_connected() is False
    with pytest.raises(RuntimeError, match="digital_twin_disconnected"):
        await client.get_kin_data()


@pytest.mark.asyncio
async def test_stop_move_fault_retains_nonzero_velocity_after_zero_velocity_pvat() -> None:
    client = DigitalTwinLebaiClient.idle(
        control_settings(),
        faults=DigitalTwinFaults(stop_failure=True),
    )
    await client.move_pvat(list(IDLE_Q), [0.0] * 6, [0.0] * 6, 0.08)

    await client.stop_move()

    kin_data = await client.get_kin_data()
    assert np.linalg.norm(kin_data["actual_joint_speed"]) > 0


@pytest.mark.asyncio
async def test_stop_sys_fault_retains_nonzero_velocity_after_zero_velocity_pvat() -> None:
    client = DigitalTwinLebaiClient.idle(
        control_settings(),
        faults=DigitalTwinFaults(stop_failure=True),
    )
    await client.move_pvat(list(IDLE_Q), [0.0] * 6, [0.0] * 6, 0.08)

    await client.stop_sys()

    kin_data = await client.get_kin_data()
    assert np.linalg.norm(kin_data["actual_joint_speed"]) > 0
