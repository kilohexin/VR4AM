from __future__ import annotations

import time
from typing import Any

from fastapi.testclient import TestClient
from starlette.websockets import WebSocketDisconnect

from app.digital_twin.lebai_client import DigitalTwinFaults, DigitalTwinLebaiClient
from app.digital_twin.runtime import create_digital_twin_app
from app.schemas.messages import TeleopMode


def _frame(seq: int, *, x: float, grip: bool = True) -> dict[str, object]:
    return {
        "v": 1,
        "type": "vr_frame",
        "session_id": "responsive-soak",
        "seq": seq,
        "client_mono_ms": time.monotonic() * 1_000.0,
        "tracking_valid": True,
        "visibility": "visible",
        "right": {
            "p": [x, 1.2, -0.3],
            "q": [0.0, 0.0, 0.0, 1.0],
            "grip": grip,
            "trigger": 0.0,
        },
    }


def _receive_type(ws: Any, message_type: str, *, limit: int = 40) -> dict[str, Any]:
    for _ in range(limit):
        message = ws.receive_json()
        if message.get("type") == message_type:
            return message
    raise AssertionError(f"did not receive {message_type}")


def _receive_ack(ws: Any, seq: int, *, limit: int = 40) -> dict[str, Any]:
    for _ in range(limit):
        state = _receive_type(ws, "robot_state")
        if state["ack_seq"] == seq:
            return state
    raise AssertionError(f"did not receive ack {seq}")


def _receive_ack_or_fault(
    ws: Any,
    seq: int,
    *,
    limit: int = 40,
) -> dict[str, Any]:
    for _ in range(limit):
        state = _receive_type(ws, "robot_state")
        if state["ack_seq"] == seq or state["fault"] is not None:
            return state
    raise AssertionError(f"did not receive ack {seq} or fault")


def _receive_arm_result(ws: Any, *, limit: int = 40) -> dict[str, Any]:
    for _ in range(limit):
        message = ws.receive_json()
        if message.get("type") in {"arm_ack", "arm_rejected"}:
            return message
    raise AssertionError("did not receive arm result")


def _send_until_state(
    ws: Any,
    seq: int,
    *,
    x: float,
    predicate: Any,
    limit: int = 100,
) -> tuple[int, dict[str, Any]]:
    for _ in range(limit):
        seq += 1
        ws.send_json(_frame(seq, x=x))
        state = _receive_type(ws, "robot_state")
        if predicate(state):
            return seq, state
        time.sleep(0.02)
    raise AssertionError("authoritative state condition not reached")


def test_single_socket_remains_responsive_through_soft_constraints_and_recovery() -> None:
    app = create_digital_twin_app()
    sent_frames = 0
    hello_ack_count = 0
    owner_change_count = 0
    unexpected_close_count = 0

    try:
        with TestClient(app) as client, client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json({"v": 1, "type": "hello", "request_id": "hello"})
            hello = _receive_type(ws, "hello_ack")
            hello_ack_count += 1
            assert hello["request_id"] == "hello"
            owner_token = app.state.teleop_owner
            assert owner_token is not None
            connected_started = time.monotonic()

            seq = 1
            ws.send_json(_frame(seq, x=0.0, grip=False))
            sent_frames += 1
            released = _receive_ack(ws, seq)
            assert released["mode"] == TeleopMode.READY
            ws.send_json({"v": 1, "type": "arm_request", "request_id": "arm"})
            arm_result = _receive_arm_result(ws)
            assert arm_result == {"v": 1, "type": "arm_ack", "request_id": "arm"}

            seq, active = _send_until_state(
                ws,
                seq,
                x=0.0,
                predicate=lambda state: state["mode"] == TeleopMode.ACTIVE,
            )
            sent_frames += seq - 1
            assert active["fault"] is None

            seq_before = seq
            seq, boundary = _send_until_state(
                ws,
                seq,
                x=0.30,
                predicate=lambda state: state["constraint"] == "workspace_boundary",
            )
            sent_frames += seq - seq_before
            assert boundary["mode"] == TeleopMode.ACTIVE
            assert boundary["fault"] is None

            seq_before = seq
            seq, recovered = _send_until_state(
                ws,
                seq,
                x=0.02,
                predicate=lambda state: state["constraint"] is None,
            )
            sent_frames += seq - seq_before
            assert recovered["mode"] == TeleopMode.ACTIVE

            backend = app.state.backend
            fake_client = backend._client
            assert isinstance(fake_client, DigitalTwinLebaiClient)
            fake_client.set_faults(DigitalTwinFaults(ik_failure=True))

            seq_before = seq
            seq, constrained = _send_until_state(
                ws,
                seq,
                x=-0.04,
                predicate=lambda state: state["constraint"] == "ik_boundary",
            )
            sent_frames += seq - seq_before
            assert constrained["mode"] == TeleopMode.ACTIVE
            assert constrained["fault"] is None

            fake_client.set_faults(DigitalTwinFaults())
            seq_before = seq
            seq, recovered = _send_until_state(
                ws,
                seq,
                x=0.0,
                predicate=lambda state: state["constraint"] is None,
            )
            sent_frames += seq - seq_before
            assert recovered["mode"] == TeleopMode.ACTIVE
            assert recovered["fault"] is None

            while time.monotonic() - connected_started < 15.0 or sent_frames < 200:
                seq += 1
                sent_frames += 1
                x = 0.025 if seq % 40 < 20 else -0.025
                ws.send_json(_frame(seq, x=x))
                state = _receive_type(ws, "robot_state")
                assert state["mode"] == TeleopMode.ACTIVE
                assert state["fault"] is None
                if app.state.teleop_owner is not owner_token:
                    owner_change_count += 1
                time.sleep(0.02)

            seq += 1
            ws.send_json(_frame(seq, x=0.0, grip=False))
            stopped = _receive_ack(ws, seq)
            assert stopped["mode"] == TeleopMode.HOLD
            assert stopped["fault"] is None

            seq += 1
            ws.send_json(_frame(seq, x=0.0, grip=True))
            regripped = _receive_ack_or_fault(ws, seq)
            assert regripped["mode"] == TeleopMode.ACTIVE
            assert regripped["fault"] is None

            seq += 1
            ws.send_json(_frame(seq, x=0.02, grip=True))
            resumed = _receive_ack_or_fault(ws, seq)
            assert resumed["mode"] == TeleopMode.ACTIVE
            assert resumed["fault"] is None
            assert resumed["ack_seq"] == seq

            seq += 1
            ws.send_json(_frame(seq, x=0.0, grip=False))
            stopped_again = _receive_ack(ws, seq)
            assert stopped_again["mode"] == TeleopMode.HOLD
            assert stopped_again["fault"] is None
            ws.send_json({"v": 1, "type": "disarm", "request_id": "disarm"})
            assert _receive_type(ws, "disarm_ack")["request_id"] == "disarm"

            assert app.state.teleop_owner is owner_token
            assert hello_ack_count == 1
            assert owner_change_count == 0
            assert unexpected_close_count == 0
            assert sent_frames >= 200
            assert time.monotonic() - connected_started >= 15.0
    except WebSocketDisconnect:
        unexpected_close_count += 1
        raise

    assert app.state.teleop_owner is None
    assert unexpected_close_count == 0
