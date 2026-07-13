import asyncio
import json
import warnings
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from starlette.exceptions import StarletteDeprecationWarning

with warnings.catch_warnings():
    warnings.filterwarnings(
        "ignore",
        message=(
            r"Using `httpx` with `starlette\.testclient` is deprecated; "
            r"install `httpx2` instead\."
        ),
        category=StarletteDeprecationWarning,
    )
    from fastapi.testclient import TestClient

from app.main import create_app
from app.schemas.messages import TeleopMode

ROOT = Path(__file__).resolve().parents[3]


def _valid_frame(**updates: object) -> dict[str, object]:
    payload = json.loads((ROOT / "schemas/fixtures/vr-frame-valid.json").read_text())
    payload.update(updates)
    return payload


def _receive_until(ws, message_type: str, limit: int = 8) -> dict[str, object]:
    for _ in range(limit):
        message = ws.receive_json()
        if message.get("type") == message_type:
            return message
    raise AssertionError(f"未收到 {message_type}")


def test_hello_and_frame_ack() -> None:
    with TestClient(create_app()) as client, client.websocket_connect(
        "/ws/v1/teleop"
    ) as ws:
        ws.send_json({"v": 1, "type": "hello", "request_id": "h1"})
        assert ws.receive_json()["type"] == "hello_ack"
        ws.send_json(_valid_frame())

        states = [_receive_until(ws, "robot_state") for _ in range(3)]

        assert any(state["ack_seq"] == 1 for state in states)


def test_arm_before_grip_release_is_rejected_with_chinese_reason() -> None:
    with TestClient(create_app()) as client, client.websocket_connect(
        "/ws/v1/teleop"
    ) as ws:
        ws.send_json({"v": 1, "type": "arm_request", "request_id": "a1"})
        message = ws.receive_json()

        assert message["type"] == "arm_rejected"
        assert "松开" in message["message"]


@pytest.mark.parametrize(
    "payload",
    [
        {"v": 1, "type": "hello", "request_id": "h1", "unexpected": True},
        {"v": 2, "type": "hello", "request_id": "h1"},
    ],
)
def test_invalid_version_or_unknown_field_reports_error_then_closes_policy_violation(
    payload: dict[str, object],
) -> None:
    with TestClient(create_app()) as client, client.websocket_connect(
        "/ws/v1/teleop"
    ) as ws:
        ws.send_json(payload)
        error = ws.receive_json()
        close = ws.receive()

        assert error["type"] == "protocol_error"
        assert "消息格式无效" in error["message"]
        assert close["type"] == "websocket.close"
        assert close["code"] == 1008


def test_pc_monotonic_receive_timestamp_is_authoritative(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    received_ns = 987_654_321
    monkeypatch.setattr("app.api.teleop_ws.monotonic_ns", lambda: received_ns)
    app = create_app()

    with TestClient(app) as client, client.websocket_connect("/ws/v1/teleop") as ws:
        ws.send_json({"v": 1, "type": "hello", "request_id": "h1"})
        assert ws.receive_json()["type"] == "hello_ack"
        ws.send_json(_valid_frame(client_mono_ms=999_999_999.0))
        _receive_until(ws, "robot_state")
        snapshot = app.state.latest.snapshot()

        assert snapshot is not None
        assert snapshot.received_ns == received_ns


def test_disarm_is_delegated_to_robot_control() -> None:
    app = create_app()
    with TestClient(app) as client:
        original = app.state.control.disarm
        app.state.control.disarm = AsyncMock(wraps=original)
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "disarm", "request_id": "d1"})
            assert ws.receive_json()["type"] == "disarm_ack"
        assert app.state.control.disarm.await_count == 1


def test_socket_close_stops_immediately_and_cleans_sender_task() -> None:
    app = create_app()
    with TestClient(app) as client:
        original = app.state.control.on_disconnect
        app.state.control.on_disconnect = AsyncMock(wraps=original)
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "hello", "request_id": "h1"})
            assert ws.receive_json()["type"] == "hello_ack"

        assert app.state.control.on_disconnect.await_count == 1
        assert app.state.teleop_sender_tasks == set()


def test_reconnect_does_not_automatically_arm() -> None:
    app = create_app()
    with TestClient(app) as client:
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "hello", "request_id": "first"})
            assert ws.receive_json()["type"] == "hello_ack"
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "hello", "request_id": "second"})
            assert ws.receive_json()["type"] == "hello_ack"
            assert app.state.control.mode == TeleopMode.READY


@pytest.mark.parametrize(
    "intruder_message",
    [
        _valid_frame(session_id="intruder", seq=999),
        {"v": 1, "type": "arm_request", "request_id": "intruder-arm"},
        {"v": 1, "type": "disarm", "request_id": "intruder-disarm"},
    ],
)
def test_second_socket_is_rejected_without_affecting_owner(
    intruder_message: dict[str, object],
) -> None:
    app = create_app()
    with TestClient(app) as client:
        original_disconnect = app.state.control.on_disconnect
        original_arm = app.state.control.arm
        original_disarm = app.state.control.disarm
        app.state.control.on_disconnect = AsyncMock(wraps=original_disconnect)
        app.state.control.arm = AsyncMock(wraps=original_arm)
        app.state.control.disarm = AsyncMock(wraps=original_disarm)

        with client.websocket_connect("/ws/v1/teleop") as owner:
            owner.send_json({"v": 1, "type": "hello", "request_id": "owner"})
            assert owner.receive_json()["type"] == "hello_ack"

            with client.websocket_connect("/ws/v1/teleop") as intruder:
                intruder.send_json(intruder_message)
                rejection = intruder.receive_json()

                assert rejection["type"] == "connection_rejected"
                assert "已有控制连接" in rejection["message"]
                close = intruder.receive()
                assert close["type"] == "websocket.close"
                assert close["code"] == 4409

            assert app.state.control.on_disconnect.await_count == 0
            assert app.state.control.arm.await_count == 0
            assert app.state.control.disarm.await_count == 0
            assert app.state.latest.snapshot() is None
            owner.send_json({"v": 1, "type": "ping", "request_id": "still-owner"})
            assert _receive_until(owner, "pong")["request_id"] == "still-owner"

        assert app.state.control.on_disconnect.await_count == 1


def test_new_owner_after_disconnect_still_requires_release_and_explicit_arm() -> None:
    app = create_app()
    with TestClient(app) as client:
        with client.websocket_connect("/ws/v1/teleop") as first:
            first.send_json({"v": 1, "type": "hello", "request_id": "first"})
            assert first.receive_json()["type"] == "hello_ack"

        with client.websocket_connect("/ws/v1/teleop") as second:
            second.send_json({"v": 1, "type": "arm_request", "request_id": "too-early"})
            assert second.receive_json()["type"] == "arm_rejected"
            second.send_json(
                _valid_frame(
                    session_id="second",
                    seq=1,
                    right={
                        "p": [0.0, 1.2, -0.3],
                        "q": [0.0, 0.0, 0.0, 1.0],
                        "grip": False,
                        "trigger": 0.0,
                    },
                )
            )
            for _ in range(3):
                if _receive_until(second, "robot_state")["ack_seq"] == 1:
                    break
            else:
                raise AssertionError("新 owner 的松开帧未被处理")
            second.send_json({"v": 1, "type": "arm_request", "request_id": "explicit"})

            assert _receive_until(second, "arm_ack")["request_id"] == "explicit"


def test_state_sender_runs_at_20hz_without_blocking_ping_receiver() -> None:
    with TestClient(create_app()) as client, client.websocket_connect(
        "/ws/v1/teleop"
    ) as ws:
        ws.send_json({"v": 1, "type": "hello", "request_id": "h1"})
        assert ws.receive_json()["type"] == "hello_ack"
        first_state = _receive_until(ws, "robot_state")
        ws.send_json({"v": 1, "type": "ping", "request_id": "p1"})
        pong = _receive_until(ws, "pong")
        second_state = _receive_until(ws, "robot_state")

        assert pong["request_id"] == "p1"
        delta = (second_state["server_mono_ns"] - first_state["server_mono_ns"]) / 1e9
        assert 0.03 <= delta <= 0.12


@pytest.mark.asyncio
async def test_state_sender_sleeps_exactly_fifty_ms(monkeypatch: pytest.MonkeyPatch) -> None:
    from app.api import teleop_ws

    sent: list[dict[str, object]] = []
    sleeps: list[float] = []

    class FakeWebSocket:
        async def send_json(self, payload: dict[str, object]) -> None:
            sent.append(payload)

    class FakeMessage:
        def model_dump(self, *, mode: str) -> dict[str, object]:
            return {"type": "robot_state", "mode": mode}

    class FakeControl:
        async def state_message(self) -> FakeMessage:
            return FakeMessage()

    async def fake_sleep(delay: float) -> None:
        sleeps.append(delay)
        if len(sleeps) == 3:
            raise asyncio.CancelledError

    monkeypatch.setattr(teleop_ws.asyncio, "sleep", fake_sleep)

    with pytest.raises(asyncio.CancelledError):
        await teleop_ws.state_sender(FakeWebSocket(), FakeControl())

    assert len(sent) == 3
    assert sleeps == [0.05, 0.05, 0.05]


@pytest.mark.asyncio
async def test_sender_failure_cancels_receiver_and_propagates_after_owner_cleanup() -> None:
    from app.api.teleop_ws import teleop_websocket

    receive_cancelled = asyncio.Event()

    class FailingControl:
        mode = TeleopMode.READY

        def __init__(self) -> None:
            self.on_disconnect = AsyncMock()

        async def state_message(self) -> None:
            raise RuntimeError("sender_failed")

    class BlockingWebSocket:
        def __init__(self, control: FailingControl) -> None:
            state = SimpleNamespace(
                control=control,
                latest=object(),
                teleop_sender_tasks=set(),
                teleop_owner=None,
                teleop_owner_lock=asyncio.Lock(),
            )
            self.app = SimpleNamespace(state=state)
            self.receive_count = 0

        async def accept(self) -> None:
            return None

        async def receive_json(self) -> dict[str, object]:
            self.receive_count += 1
            if self.receive_count == 1:
                return {"v": 1, "type": "hello", "request_id": "h1"}
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                receive_cancelled.set()
                raise
            raise AssertionError("unreachable")

        async def send_json(self, payload: dict[str, object]) -> None:
            assert payload["type"] == "hello_ack"

    control = FailingControl()
    websocket = BlockingWebSocket(control)

    with pytest.raises(RuntimeError, match="sender_failed"):
        await asyncio.wait_for(teleop_websocket(websocket), timeout=0.2)

    assert receive_cancelled.is_set()
    assert control.on_disconnect.await_count == 1
    assert websocket.app.state.teleop_sender_tasks == set()
