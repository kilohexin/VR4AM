import asyncio
import gc
import json
import warnings
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from starlette.exceptions import StarletteDeprecationWarning

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

from app.main import create_app
from app.control.robot_control import FaultResetResult, HomeResult
from app.schemas.messages import BackendState, Pose, RobotStateMessage, TeleopMode

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


def _disarmed_robot_state() -> RobotStateMessage:
    return RobotStateMessage(
        server_mono_ns=123,
        mode=TeleopMode.DISARMED,
        robot_state=BackendState.IDLE,
        actual_tcp=Pose(p=(0.3, 0.0, 0.3), q=(0.0, 0.0, 0.0, 1.0)),
        actual_q=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        gripper=0.0,
        fault=None,
    )


def _fault_robot_state() -> RobotStateMessage:
    return _disarmed_robot_state().model_copy(
        update={"mode": TeleopMode.FAULT, "fault": "workspace_violation"}
    )


def _receive_reset_sequence(ws, limit: int = 8) -> list[dict[str, object]]:
    messages: list[dict[str, object]] = []
    for _ in range(limit):
        message = ws.receive_json()
        if message.get("type") in {"robot_state", "fault_reset_result"}:
            messages.append(message)
        if message.get("type") == "fault_reset_result":
            return messages
    raise AssertionError("未收到 fault_reset_result")


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


def test_home_request_returns_correlated_result() -> None:
    app = create_app()
    with TestClient(app) as client:
        app.state.control.home = AsyncMock(return_value=HomeResult(True))
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "home_request", "request_id": "home-1"})
            result = ws.receive_json()

    assert result == {
        "v": 1,
        "type": "home_result",
        "request_id": "home-1",
        "accepted": True,
        "mode": "DISARMED",
    }
    app.state.control.home.assert_awaited_once_with()


def test_accepted_fault_reset_sends_authoritative_state_before_exact_result() -> None:
    app = create_app()
    with TestClient(app) as client:
        app.state.control.reset_fault = AsyncMock(return_value=FaultResetResult(True))
        app.state.control.state_message = AsyncMock(
            return_value=_disarmed_robot_state()
        )
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "reset_fault", "request_id": "复位-1"})
            ws.send_json({"invalid": True})
            messages = _receive_reset_sequence(ws)

    assert [message["type"] for message in messages] == [
        "robot_state",
        "fault_reset_result",
    ]
    assert messages[0]["mode"] == "DISARMED"
    assert messages[0]["fault"] is None
    assert messages[1] == {
        "v": 1,
        "type": "fault_reset_result",
        "request_id": "复位-1",
        "accepted": True,
        "mode": "DISARMED",
    }
    app.state.control.reset_fault.assert_awaited_once_with()
    app.state.control.state_message.assert_awaited_once_with()


def test_rejected_fault_reset_sends_exact_reason_and_message() -> None:
    app = create_app()
    with TestClient(app) as client:
        app.state.control.reset_fault = AsyncMock(
            return_value=FaultResetResult(
                False,
                "backend_moving",
                "仿真仍在运动，请稍后重试。",
            )
        )
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "reset_fault", "request_id": "r2"})
            ws.send_json({"invalid": True})
            result = _receive_until(ws, "fault_reset_result")

    assert result == {
        "v": 1,
        "type": "fault_reset_result",
        "request_id": "r2",
        "accepted": False,
        "reason": "backend_moving",
        "message": "仿真仍在运动，请稍后重试。",
    }


def test_fault_reset_exception_is_contained_without_raw_details() -> None:
    app = create_app()
    with TestClient(app) as client:
        app.state.control.reset_fault = AsyncMock(
            side_effect=RuntimeError("private backend failure")
        )
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "reset_fault", "request_id": "r3"})
            ws.send_json({"invalid": True})
            result = _receive_until(ws, "fault_reset_result")

    assert result == {
        "v": 1,
        "type": "fault_reset_result",
        "request_id": "r3",
        "accepted": False,
        "reason": "unrecoverable_fault",
        "message": "该故障无法在线复位，请重启后端并重新检查。",
    }
    assert "private backend failure" not in str(result)


@pytest.mark.asyncio
async def test_reset_serializes_periodic_state_and_authoritative_result() -> None:
    from app.api import teleop_ws

    old_state_started = asyncio.Event()
    reset_done = asyncio.Event()
    post_result_state_sent = asyncio.Event()
    sent: list[dict[str, object]] = []

    class RacingControl:
        mode = TeleopMode.READY

        def __init__(self) -> None:
            self.state_calls = 0

        async def state_message(self) -> RobotStateMessage:
            self.state_calls += 1
            if self.state_calls == 1:
                old_state_started.set()
                await reset_done.wait()
                return _fault_robot_state()
            return _disarmed_robot_state()

        async def reset_fault(self) -> FaultResetResult:
            await old_state_started.wait()
            reset_done.set()
            return FaultResetResult(True)

    class RacingWebSocket:
        def __init__(self, control: RacingControl) -> None:
            self.app = SimpleNamespace(state=SimpleNamespace(latest=object()))
            self.control = control
            self.incoming = [
                {"v": 1, "type": "hello", "request_id": "hello-race"},
                {"v": 1, "type": "reset_fault", "request_id": "reset-race"},
            ]
            self.result_sent = False

        async def receive_json(self) -> dict[str, object]:
            if self.incoming:
                return self.incoming.pop(0)
            await asyncio.Future()
            raise AssertionError("unreachable")

        async def send_json(self, payload: dict[str, object]) -> None:
            sent.append(payload)
            if payload.get("type") == "fault_reset_result":
                self.result_sent = True
            elif payload.get("type") == "robot_state" and self.result_sent:
                post_result_state_sent.set()

    control = RacingControl()
    websocket = RacingWebSocket(control)
    session = asyncio.create_task(
        teleop_ws._run_coupled_session(websocket, control, set())
    )
    try:
        await asyncio.wait_for(post_result_state_sent.wait(), timeout=0.3)
    finally:
        session.cancel()
        with pytest.raises(asyncio.CancelledError):
            await session

    result_index = next(
        index
        for index, message in enumerate(sent)
        if message.get("type") == "fault_reset_result"
    )
    states_before_result = [
        message for message in sent[:result_index] if message.get("type") == "robot_state"
    ]
    states_after_result = [
        message for message in sent[result_index + 1 :] if message.get("type") == "robot_state"
    ]

    assert states_before_result[-1]["mode"] == "DISARMED"
    assert states_before_result[-1]["fault"] is None
    assert states_after_result[0]["fault"] is None


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
        {"v": 1, "type": "reset_fault", "request_id": "intruder-reset"},
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
        original_reset_fault = app.state.control.reset_fault
        app.state.control.on_disconnect = AsyncMock(wraps=original_disconnect)
        app.state.control.arm = AsyncMock(wraps=original_arm)
        app.state.control.disarm = AsyncMock(wraps=original_disarm)
        app.state.control.reset_fault = AsyncMock(wraps=original_reset_fault)

        with client.websocket_connect("/ws/v1/teleop") as owner:
            owner.send_json({"v": 1, "type": "hello", "request_id": "owner"})
            assert owner.receive_json()["type"] == "hello_ack"

            with client.websocket_connect("/ws/v1/teleop") as intruder:
                intruder.send_json(intruder_message)
                rejection = intruder.receive_json()

                assert rejection == {
                    "v": 1,
                    "type": "connection_rejected",
                    "reason": "controller_occupied",
                    "message": "已有控制页面占用，请关闭电脑端网页后重试。",
                }
                close = intruder.receive()
                assert close["type"] == "websocket.close"
                assert close["code"] == 4409

            assert app.state.control.on_disconnect.await_count == 0
            assert app.state.control.arm.await_count == 0
            assert app.state.control.disarm.await_count == 0
            assert app.state.control.reset_fault.await_count == 0
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


def test_state_sender_runs_at_50hz_without_blocking_ping_receiver() -> None:
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
        assert 0.012 <= delta <= 0.08


@pytest.mark.asyncio
async def test_state_sender_sleeps_exactly_twenty_ms(monkeypatch: pytest.MonkeyPatch) -> None:
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
    assert sleeps == [0.02, 0.02, 0.02]


@pytest.mark.asyncio
async def test_sender_failure_cancels_receiver_and_propagates_after_owner_cleanup() -> None:
    from app.api.teleop_ws import teleop_websocket

    receive_cancelled = asyncio.Event()

    class FailingControl:
        mode = TeleopMode.READY

        def __init__(self) -> None:
            self.on_disconnect = AsyncMock(
                side_effect=RuntimeError("disconnect_cleanup_failed")
            )

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
    assert websocket.app.state.teleop_owner is None
    assert websocket.app.state.teleop_sender_tasks == set()


@pytest.mark.asyncio
async def test_simultaneous_receiver_and_sender_failures_choose_receiver_primary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from app.api import teleop_ws

    receiver_ready = asyncio.Event()
    sender_ready = asyncio.Event()
    release = asyncio.Event()
    loop_contexts: list[dict[str, object]] = []

    async def fail_receiver(*args) -> None:
        receiver_ready.set()
        await release.wait()
        raise RuntimeError("receiver_failed")

    async def fail_sender(*args) -> None:
        sender_ready.set()
        await release.wait()
        raise RuntimeError("sender_failed")

    monkeypatch.setattr(teleop_ws, "_receive_messages", fail_receiver)
    monkeypatch.setattr(teleop_ws, "_delayed_state_sender", fail_sender)
    sender_tasks: set[asyncio.Task[None]] = set()
    loop = asyncio.get_running_loop()
    previous_handler = loop.get_exception_handler()
    loop.set_exception_handler(lambda _loop, context: loop_contexts.append(context))
    try:
        session = asyncio.create_task(
            teleop_ws._run_coupled_session(object(), object(), sender_tasks)
        )
        await receiver_ready.wait()
        await sender_ready.wait()
        release.set()

        try:
            await session
        except RuntimeError as error:
            primary_message = str(error)
        else:
            raise AssertionError("双失败 session 未传播异常")

        del session
        gc.collect()
        await asyncio.sleep(0)
    finally:
        loop.set_exception_handler(previous_handler)

    assert sender_tasks == set()
    assert primary_message == "receiver_failed"
    assert not any(
        context.get("message") == "Task exception was never retrieved"
        for context in loop_contexts
    )


@pytest.mark.asyncio
async def test_external_session_cancellation_stays_primary_over_peer_cleanup_failure(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from app.api import teleop_ws

    receiver_ready = asyncio.Event()
    sender_ready = asyncio.Event()
    receiver_cancelled = asyncio.Event()
    sender_cancelled = asyncio.Event()
    loop_contexts: list[dict[str, object]] = []

    async def receiver_with_failing_cleanup(*args) -> None:
        receiver_ready.set()
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            receiver_cancelled.set()
            raise RuntimeError("cleanup_peer_failed")

    async def cancellable_sender(*args) -> None:
        sender_ready.set()
        try:
            await asyncio.Future()
        except asyncio.CancelledError:
            sender_cancelled.set()
            raise

    class FakeControl:
        mode = TeleopMode.READY

        def __init__(self) -> None:
            self.on_disconnect = AsyncMock()

    class FakeWebSocket:
        def __init__(self, control: FakeControl) -> None:
            self.app = SimpleNamespace(
                state=SimpleNamespace(
                    control=control,
                    latest=object(),
                    teleop_sender_tasks=set(),
                    teleop_owner=None,
                    teleop_owner_lock=asyncio.Lock(),
                )
            )

        async def accept(self) -> None:
            return None

    monkeypatch.setattr(teleop_ws, "_receive_messages", receiver_with_failing_cleanup)
    monkeypatch.setattr(teleop_ws, "_delayed_state_sender", cancellable_sender)
    control = FakeControl()
    websocket = FakeWebSocket(control)
    loop = asyncio.get_running_loop()
    previous_handler = loop.get_exception_handler()
    loop.set_exception_handler(lambda _loop, context: loop_contexts.append(context))
    try:
        session = asyncio.create_task(teleop_ws.teleop_websocket(websocket))
        await receiver_ready.wait()
        await sender_ready.wait()
        session.cancel()

        with pytest.raises(asyncio.CancelledError):
            await session

        assert receiver_cancelled.is_set()
        assert sender_cancelled.is_set()
        assert control.on_disconnect.await_count == 1
        assert websocket.app.state.teleop_owner is None
        assert websocket.app.state.teleop_sender_tasks == set()
        del session
        gc.collect()
    finally:
        loop.set_exception_handler(previous_handler)

    assert not any(
        context.get("message") == "Task exception was never retrieved"
        for context in loop_contexts
    )
