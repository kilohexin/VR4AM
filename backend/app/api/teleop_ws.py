from __future__ import annotations

import asyncio
from time import monotonic_ns
from typing import Any

import anyio
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


async def _delayed_state_sender(
    websocket: WebSocket,
    control: RobotControl,
    start_sender: asyncio.Event,
) -> None:
    await start_sender.wait()
    await state_sender(websocket, control)


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


async def _receive_messages(
    websocket: WebSocket,
    control: RobotControl,
    start_sender: asyncio.Event,
) -> None:
    app = websocket.app
    while True:
        try:
            payload = await websocket.receive_json()
            message = _parse_message(payload)
        except WebSocketDisconnect:
            return
        except (ValidationError, ValueError, TypeError):
            await _protocol_error(websocket)
            return

        if isinstance(message, VRFrame):
            app.state.latest.publish(message, monotonic_ns())
            start_sender.set()
            continue

        if message.type == "hello":
            await websocket.send_json(
                {"v": 1, "type": "hello_ack", "request_id": message.request_id}
            )
            start_sender.set()
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


async def _run_coupled_session(
    websocket: WebSocket,
    control: RobotControl,
    sender_tasks: set[asyncio.Task[None]],
) -> None:
    start_sender = asyncio.Event()
    receiver = asyncio.create_task(
        _receive_messages(websocket, control, start_sender),
        name="teleop-receiver",
    )
    sender = asyncio.create_task(
        _delayed_state_sender(websocket, control, start_sender),
        name="teleop-state-20hz",
    )
    sender_tasks.add(sender)
    wait_error: BaseException | None = None
    try:
        with anyio.CancelScope(shield=True):
            await asyncio.wait(
                {receiver, sender}, return_when=asyncio.FIRST_COMPLETED
            )
    except BaseException as error:
        wait_error = error
    finally:
        try:
            for task in (receiver, sender):
                if not task.done():
                    task.cancel()
            with anyio.CancelScope(shield=True):
                results = await asyncio.gather(
                    receiver,
                    sender,
                    return_exceptions=True,
                )
        finally:
            sender_tasks.discard(sender)
    if wait_error is not None:
        raise wait_error
    for result in results:
        if isinstance(result, (asyncio.CancelledError, WebSocketDisconnect)):
            continue
        if isinstance(result, BaseException):
            raise result


@router.websocket("/ws/v1/teleop")
async def teleop_websocket(websocket: WebSocket) -> None:
    await websocket.accept()
    app = websocket.app
    control: RobotControl = app.state.control
    owner_token = object()
    async with app.state.teleop_owner_lock:
        owns_connection = app.state.teleop_owner is None
        if owns_connection:
            app.state.teleop_owner = owner_token
    if not owns_connection:
        message = "已有控制页面占用，请关闭电脑端网页后重试。"
        await websocket.send_json(
            {
                "v": 1,
                "type": "connection_rejected",
                "reason": "controller_occupied",
                "message": message,
            }
        )
        await websocket.close(code=4409, reason=message)
        return
    session_error: BaseException | None = None
    cleanup_error: BaseException | None = None
    try:
        if control.mode == TeleopMode.DISCONNECTED:
            await control.connect()
        await _run_coupled_session(
            websocket, control, app.state.teleop_sender_tasks
        )
    except BaseException as error:
        session_error = error
    finally:
        async with app.state.teleop_owner_lock:
            owns_connection = app.state.teleop_owner is owner_token
        if owns_connection:
            try:
                try:
                    await control.on_disconnect()
                except BaseException as error:
                    cleanup_error = error
                finally:
                    latest = LatestVRFrame()
                    control.latest = latest
                    app.state.latest = latest
            finally:
                async with app.state.teleop_owner_lock:
                    if app.state.teleop_owner is owner_token:
                        app.state.teleop_owner = None
    if session_error is not None:
        raise session_error
    if cleanup_error is not None:
        raise cleanup_error
