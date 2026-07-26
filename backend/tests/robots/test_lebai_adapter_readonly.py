from __future__ import annotations

from unittest.mock import AsyncMock

import pytest

from app.robots.base import BackendCommandError, HomeOptions, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.schemas.messages import BackendState
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings, readonly_settings


HOME_OPTIONS = HomeOptions(
    max_speed_radps=0.1,
    timeout_s=10.0,
    position_tolerance_rad=0.01,
    velocity_tolerance_radps=0.02,
    stable_seconds=0.3,
)


async def _connected_readonly(
    client: FakeLebaiClient | None = None,
) -> tuple[RealLebaiAdapter, FakeLebaiClient]:
    fake = client or FakeLebaiClient.idle()
    adapter = RealLebaiAdapter(
        readonly_settings(),
        client_factory=AsyncMock(return_value=fake),
        clock=lambda: 123_000_000,
    )
    await adapter.connect()
    return adapter, fake


@pytest.mark.asyncio
async def test_readonly_preflight_reads_state_without_writes() -> None:
    adapter, client = await _connected_readonly()

    result = await adapter.preflight()

    assert result.ready is False
    assert result.reason == "real_robot_readonly"
    assert result.robot_state is BackendState.IDLE
    assert result.actual_q == (0.0, -1.0, 1.0, 0.0, 1.57, 0.0)
    assert result.actual_tcp.p == pytest.approx((0.3, 0.0, 0.4))
    assert result.tcp_matches is True
    assert "pvat" in result.capabilities
    assert client.write_calls == []


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "operation",
    [
        lambda adapter: adapter.command_tcp(None, 1),
        lambda adapter: adapter.set_gripper(0.5),
        lambda adapter: adapter.home(HOME_OPTIONS, lambda phase: None),
    ],
)
async def test_readonly_rejects_state_changes(operation) -> None:
    adapter, client = await _connected_readonly()
    with pytest.raises(BackendCommandError, match="^real_robot_readonly$"):
        await operation(adapter)
    assert client.write_calls == []


@pytest.mark.asyncio
async def test_readonly_shutdown_stop_and_disconnect_never_write() -> None:
    adapter, client = await _connected_readonly()

    await adapter.stop(StopReason.SHUTDOWN)
    await adapter.disconnect()

    assert client.write_calls == []


@pytest.mark.asyncio
async def test_readonly_state_normalizes_gripper_without_writing() -> None:
    client = FakeLebaiClient.idle()
    client.claw["amplitude"] = 75
    adapter, client = await _connected_readonly(client)

    state = await adapter.get_state()

    assert state.robot_state is BackendState.IDLE
    assert state.gripper == pytest.approx(0.25)
    assert client.write_calls == []


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("mutate", "reason"),
    [
        (lambda client: setattr(client, "robot_state", "MOVING"), "robot_not_idle"),
        (lambda client: setattr(client, "running_motion", 7), "motion_running"),
        (lambda client: setattr(client, "estop_reason", 4), "estop:hard_estop"),
        (
            lambda client: client.tcp.update({"z": 0.20}),
            "tcp_mismatch",
        ),
        (
            lambda client: client.kin_data.update(
                {"actual_joint_pose": [2.99, -1, 1, 0, 1.57, 0]}
            ),
            "joint_outside_soft_limits",
        ),
        (
            lambda client: client.kin_data.update(
                {
                    "actual_tcp_pose": {
                        "x": 0.8,
                        "y": 0,
                        "z": 0.4,
                        "rx": 0,
                        "ry": 0,
                        "rz": 0,
                    }
                }
            ),
            "tcp_outside_startup_envelope",
        ),
    ],
)
async def test_preflight_reports_each_safety_rejection(mutate, reason: str) -> None:
    client = FakeLebaiClient.idle()
    mutate(client)
    adapter, _ = await _connected_readonly(client)

    result = await adapter.preflight()

    assert result.ready is False
    assert result.reason == reason
    assert client.write_calls == []


@pytest.mark.asyncio
async def test_connect_rejects_disconnected_or_malformed_sdk_data() -> None:
    disconnected = FakeLebaiClient.idle()
    disconnected.connected = False
    adapter = RealLebaiAdapter(
        readonly_settings(),
        client_factory=AsyncMock(return_value=disconnected),
    )
    with pytest.raises(BackendCommandError, match="^robot_disconnected$"):
        await adapter.connect()

    malformed = FakeLebaiClient.idle()
    malformed.kin_data["actual_joint_speed"] = [0, 0, 0, 0, 0]
    adapter = RealLebaiAdapter(
        readonly_settings(),
        client_factory=AsyncMock(return_value=malformed),
    )
    with pytest.raises(
        BackendCommandError,
        match="^invalid_sdk_joint_vector:actual_joint_speed$",
    ):
        await adapter.connect()


@pytest.mark.asyncio
async def test_control_preflight_can_be_ready_without_writing() -> None:
    client = FakeLebaiClient.idle()
    adapter = RealLebaiAdapter(
        control_settings(),
        client_factory=AsyncMock(return_value=client),
    )
    await adapter.connect()

    result = await adapter.preflight()

    assert result.ready is True
    assert result.reason is None
    assert client.write_calls == []
