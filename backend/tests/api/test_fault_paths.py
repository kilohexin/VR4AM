from __future__ import annotations

import asyncio
import json
import warnings
from pathlib import Path
from typing import Literal
from unittest.mock import AsyncMock

import pytest
try:
    from starlette.exceptions import StarletteDeprecationWarning
except ImportError:
    class StarletteDeprecationWarning(DeprecationWarning):
        """Compatibility category for Starlette versions that removed it."""


from app.control.robot_control import LatestVRFrame, RobotControl
from app.main import create_app
from app.recording.noop import NoopRecorder
from app.robots.base import StopReason
from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import ControllerState, TeleopMode, VRFrame
from app.sim.ik import IKError

STARLETTE_HTTPX_WARNING_PATTERN = (
    r"\AUsing `httpx` with `starlette\.testclient` is deprecated; "
    r"install `httpx2` instead\.\Z"
)

with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore",
        message=STARLETTE_HTTPX_WARNING_PATTERN,
        category=StarletteDeprecationWarning,
    )
    from fastapi.testclient import TestClient

ROOT = Path(__file__).resolve().parents[3]


class FakeClock:
    def __init__(self) -> None:
        self.value = 1_000_000_000

    def now_ns(self) -> int:
        return self.value

    def advance_ms(self, milliseconds: float) -> None:
        self.value += int(milliseconds * 1_000_000)


class RecordingSimAdapter(SimRobotAdapter):
    def __init__(self) -> None:
        super().__init__()
        self.stop_reasons: list[StopReason] = []
        self.command_count = 0

    async def command_tcp(self, target, command_id: int) -> None:
        await super().command_tcp(target, command_id)
        self.command_count += 1

    async def stop(self, reason: StopReason) -> None:
        self.stop_reasons.append(reason)
        await super().stop(reason)


def _frame(
    seq: int,
    grip: bool,
    *,
    session_id: str = "integration",
    p: tuple[float, float, float] = (0.0, 1.2, -0.3),
    tracking_valid: bool = True,
    visibility: Literal["visible", "visible-blurred", "hidden"] = "visible",
) -> VRFrame:
    return VRFrame(
        v=1,
        type="vr_frame",
        session_id=session_id,
        seq=seq,
        client_mono_ms=float(seq),
        tracking_valid=tracking_valid,
        visibility=visibility,
        right=ControllerState(
            p=p,
            q=(0.0, 0.0, 0.0, 1.0),
            grip=grip,
            trigger=0.25,
        ),
    )


def _payload(**updates: object) -> dict[str, object]:
    payload = json.loads((ROOT / "schemas/fixtures/vr-frame-valid.json").read_text())
    payload.update(updates)
    return payload


def _receive_until(ws, message_type: str, limit: int = 12) -> dict[str, object]:
    for _ in range(limit):
        message = ws.receive_json()
        if message.get("type") == message_type:
            return message
    raise AssertionError(f"did not receive {message_type}")


def _receive_ack(ws, seq: int, limit: int = 12) -> dict[str, object]:
    for _ in range(limit):
        state = _receive_until(ws, "robot_state")
        if state["ack_seq"] == seq:
            return state
    raise AssertionError(f"did not receive ack {seq}")


async def _active_control() -> tuple[
    RobotControl, LatestVRFrame, RecordingSimAdapter, FakeClock
]:
    latest = LatestVRFrame()
    adapter = RecordingSimAdapter()
    clock = FakeClock()
    control = RobotControl(
        backend=adapter,
        latest=latest,
        clock=clock,
        recorder=NoopRecorder(),
    )
    await control.connect()
    latest.publish(_frame(1, False), clock.now_ns())
    await control.tick()
    await control.arm()
    latest.publish(_frame(2, True), clock.now_ns())
    await control.tick()
    latest.publish(_frame(3, True, p=(0.0, 1.2, -0.305)), clock.now_ns())
    await control.tick()
    assert control.mode is TeleopMode.ACTIVE
    assert adapter.command_count == 1
    return control, latest, adapter, clock


@pytest.mark.asyncio
@pytest.mark.parametrize(
    ("tracking_valid", "visibility"),
    [
        (False, "visible"),
        (True, "hidden"),
        (True, "visible-blurred"),
    ],
)
async def test_tracking_and_visibility_loss_publish_stale_before_disarming(
    tracking_valid: bool,
    visibility: Literal["visible", "visible-blurred", "hidden"],
) -> None:
    control, latest, adapter, clock = await _active_control()
    latest.publish(
        _frame(
            4,
            True,
            tracking_valid=tracking_valid,
            visibility=visibility,
        ),
        clock.now_ns(),
    )

    await control.tick()

    assert control.mode is TeleopMode.STALE
    assert adapter.stop_reasons.count(StopReason.STALE) == 1
    published = await control.state_message()
    assert published.mode is TeleopMode.STALE
    assert control.mode is TeleopMode.DISARMED


@pytest.mark.asyncio
async def test_same_session_rollback_never_reaches_real_simulator_control() -> None:
    latest = LatestVRFrame()
    adapter = RecordingSimAdapter()
    clock = FakeClock()
    control = RobotControl(
        backend=adapter,
        latest=latest,
        clock=clock,
        recorder=NoopRecorder(),
    )
    await control.connect()
    latest.publish(_frame(2, False), clock.now_ns())
    clock.advance_ms(20)
    latest.publish(_frame(1, True), clock.now_ns())

    await control.tick()

    newest = latest.snapshot()
    assert newest is not None
    assert newest.frame.seq == 2
    assert newest.received_ns == 1_000_000_000
    assert control.last_seq == 2
    assert control.mode is TeleopMode.READY
    assert adapter.command_count == 0


def test_nan_pose_is_rejected_before_control_and_protocol_cleanup_is_complete() -> None:
    app = create_app()
    with TestClient(app) as client:
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "hello", "request_id": "hello"})
            assert ws.receive_json()["type"] == "hello_ack"
            frame = _payload()
            frame["right"]["p"][0] = float("nan")  # type: ignore[index]
            ws.send_json(frame)

            assert _receive_until(ws, "protocol_error")["type"] == "protocol_error"
            close = ws.receive()
            assert close["type"] == "websocket.close"
            assert close["code"] == 1008

        assert app.state.latest.snapshot() is None
        assert app.state.backend.command_id is None
        assert app.state.teleop_sender_tasks == set()
        assert app.state.teleop_owner is None


def test_disconnect_clears_session_and_reconnect_requires_release_plus_arm() -> None:
    app = create_app()
    with TestClient(app) as client:
        stop = AsyncMock(wraps=app.state.backend.stop)
        app.state.backend.stop = stop
        with client.websocket_connect("/ws/v1/teleop") as first:
            first.send_json(_payload(session_id="first", seq=1))
            assert _receive_ack(first, 1)["ack_seq"] == 1
            first.send_json({"v": 1, "type": "arm_request", "request_id": "arm-1"})
            assert _receive_until(first, "arm_ack")["request_id"] == "arm-1"
            retired_latest = app.state.latest

        stop.assert_any_await(StopReason.DISCONNECT)
        assert app.state.control.mode is TeleopMode.DISCONNECTED
        assert app.state.latest is not retired_latest
        assert app.state.latest.snapshot() is None
        assert app.state.teleop_sender_tasks == set()

        retired_latest.publish(_frame(99, True, session_id="first"), 9_999_999_999)
        assert app.state.control.latest.snapshot() is None

        with client.websocket_connect("/ws/v1/teleop") as second:
            second.send_json(
                {"v": 1, "type": "arm_request", "request_id": "too-early"}
            )
            assert _receive_until(second, "arm_rejected")["request_id"] == "too-early"
            second.send_json(_payload(session_id="second", seq=2))
            assert _receive_ack(second, 2)["ack_seq"] == 2
            second.send_json(
                {"v": 1, "type": "arm_request", "request_id": "explicit-arm"}
            )
            assert _receive_until(second, "arm_ack")["request_id"] == "explicit-arm"


@pytest.mark.asyncio
async def test_simulator_ik_error_publishes_soft_constraint_without_disarming(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    control, latest, adapter, clock = await _active_control()

    def fail_ik(*args, **kwargs):
        raise IKError("ik_unreachable")

    monkeypatch.setattr(
        "app.robots.sim_adapter.cartesian_servo_step",
        fail_ik,
    )
    latest.publish(_frame(4, True, p=(0.0, 1.2, -0.31)), clock.now_ns())

    await control.tick()

    assert control.mode is TeleopMode.ACTIVE
    assert adapter.stop_reasons.count(StopReason.FAULT) == 0
    published = await control.state_message()
    assert published.mode is TeleopMode.ACTIVE
    assert published.constraint == "ik_boundary"
    assert published.fault is None
    assert control.mode is TeleopMode.ACTIVE


@pytest.mark.asyncio
async def test_absolute_deadline_lateness_faults_real_simulator_chain(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    latest = LatestVRFrame()
    adapter = RecordingSimAdapter()
    clock = FakeClock()
    control = RobotControl(
        backend=adapter,
        latest=latest,
        clock=clock,
        recorder=NoopRecorder(),
    )
    tick_count = 0
    real_tick = control.tick

    async def seventy_ms_tick() -> None:
        nonlocal tick_count
        tick_count += 1
        clock.advance_ms(70)
        await real_tick()
        if tick_count == 3:
            control._running = False

    async def no_sleep(_delay: float) -> None:
        return None

    monkeypatch.setattr("app.control.robot_control.asyncio.sleep", no_sleep)
    control.tick = seventy_ms_tick  # type: ignore[method-assign]
    control._running = True

    await control.run()

    assert adapter.stop_reasons.count(StopReason.FAULT) == 1
    assert control.mode is TeleopMode.FAULT
    published = await control.state_message()
    assert published.mode is TeleopMode.FAULT
    assert published.fault == "control_overrun"
    assert control.mode is TeleopMode.DISARMED
