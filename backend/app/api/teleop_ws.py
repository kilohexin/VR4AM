from __future__ import annotations

import asyncio
import contextlib
from time import monotonic_ns
from typing import Any

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from app.control.robot_control import LatestVRFrame, RobotControl
from app.schemas.messages import ClientControlMessage, TeleopMode, VRFrame

router = APIRouter()


async def state_sender(websocket: WebSocket, control: RobotControl) -> None:
    while True:
        state = await control.state_message()
        await websocket.send_json(state.model_dump(mode="json"))
        await asyncio.sleep(0.05)


async def _protocol_error(websocket: WebSocket) -> None:
    message = "消息格式无效，请检查协议版本和字段。"
    await websocket.send_json({"v": 1, "type": "protocol_error", "message": message})
    await websocket.close(code=1008, reason=message)


def _parse_message(payload: Any) -> VRFrame | ClientControlMessage:
    if not isinstance(payload, dict):
        raise ValueError("message_must_be_object")
    if payload.get("type") == "vr_frame":
        return VRFrame.model_validate(payload)
    return ClientControlMessage.model_validate(payload)


@router.websocket("/ws/v1/teleop")
async def teleop_websocket(websocket: WebSocket) -> None:
    await websocket.accept()
    app = websocket.app
    control: RobotControl = app.state.control
    if control.mode == TeleopMode.DISCONNECTED:
        await control.connect()
    sender: asyncio.Task[None] | None = None

    def ensure_sender() -> None:
        nonlocal sender
        if sender is None:
            sender = asyncio.create_task(
                state_sender(websocket, control), name="teleop-state-20hz"
            )
            app.state.teleop_sender_tasks.add(sender)

    try:
        while True:
            try:
                payload = await websocket.receive_json()
                message = _parse_message(payload)
            except WebSocketDisconnect:
                break
            except (ValidationError, ValueError, TypeError):
                await _protocol_error(websocket)
                break

            if isinstance(message, VRFrame):
                app.state.latest.publish(message, monotonic_ns())
                ensure_sender()
                continue

            if message.type == "hello":
                await websocket.send_json(
                    {"v": 1, "type": "hello_ack", "request_id": message.request_id}
                )
                ensure_sender()
            elif message.type == "arm_request":
                try:
                    await control.arm()
                except RuntimeError:
                    await websocket.send_json(
                        {
                            "v": 1,
                            "type": "arm_rejected",
                            "request_id": message.request_id,
                            "message": "请先松开手柄抓握键，再请求使能。",
                        }
                    )
                else:
                    await websocket.send_json(
                        {"v": 1, "type": "arm_ack", "request_id": message.request_id}
                    )
            elif message.type == "disarm":
                await control.disarm()
                await websocket.send_json(
                    {"v": 1, "type": "disarm_ack", "request_id": message.request_id}
                )
            elif message.type == "ping":
                await websocket.send_json(
                    {"v": 1, "type": "pong", "request_id": message.request_id}
                )
    finally:
        if sender is not None:
            sender.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await sender
            app.state.teleop_sender_tasks.discard(sender)
        await control.on_disconnect()
        latest = LatestVRFrame()
        control.latest = latest
        app.state.latest = latest
