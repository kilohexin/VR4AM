from __future__ import annotations

import asyncio
from typing import Any

import anyio
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from app.control.robot_control import LatestVRFrame, RobotControl
from app.api.operator_jobs import OperatorJobs
from app.diagnostics.session_trace import SessionTrace
from app.diagnostics.store import DiagnosticsStore
from app.rehearsal.report import HARDWARE_PENDING, RehearsalReportStore
from app.robots.base import BackendCommandError
from app.schemas.messages import (
    ClientMessage,
    FinitePositionInputMessage,
    GripperRequestMessage,
    GripperCancelMessage,
    ClientControlMessage,
    OfflineRehearsalBeginMessage,
    OfflineRehearsalFinishMessage,
    OfflineRehearsalPhaseMessage,
    RuntimeBackend,
    SetSimulationScaleMessage,
    SimulationScaleResultMessage,
    TeleopMode,
    VRFrame,
)

router = APIRouter()


async def _send_json(
    websocket: WebSocket,
    payload: dict[str, Any],
    send_lock: asyncio.Lock,
) -> None:
    async with send_lock:
        await websocket.send_json(payload)


async def state_sender(
    websocket: WebSocket,
    control: RobotControl,
    send_lock: asyncio.Lock | None = None,
    *, close_requested: asyncio.Event | None = None,
    read_pending: asyncio.Event | None = None,
) -> None:
    send_lock = send_lock or asyncio.Lock()
    app_state = getattr(getattr(websocket, "app", None), "state", None)
    settings = getattr(app_state, "settings", None)
    state_hz = getattr(settings, "state_hz", 50)
    if (
        getattr(settings, "backend", None) == "lebai"
        and getattr(settings, "lebai", None) is not None
    ):
        state_hz = settings.lebai.control.state_hz
    period_s = 1.0 / state_hz
    clock = asyncio.get_running_loop().time
    while True:
        if close_requested is not None and close_requested.is_set():return
        async with send_lock:
            if close_requested is not None and close_requested.is_set():return
            started = clock()
            if read_pending is not None:read_pending.set()
            try:state = await control.state_message()
            finally:
                if read_pending is not None:read_pending.clear()
            if close_requested is not None and close_requested.is_set():return
            await websocket.send_json(state.model_dump(mode="json"))
        # The configured period includes reading/encoding/sending. Sleeping a
        # full period after that work ages feedback beyond the settling gate.
        # Drop missed slots rather than issuing a catch-up burst of SDK reads.
        now = clock()
        deadline = started + period_s if now < started + period_s else now + period_s
        while (remaining := deadline - clock()) > 0:
            # Recheck the remaining period after an early scheduler wakeup.
            await asyncio.sleep(remaining)


async def _delayed_state_sender(
    websocket: WebSocket,
    control: RobotControl,
    start_sender: asyncio.Event,
    send_lock: asyncio.Lock,
    *, close_requested: asyncio.Event | None = None,
    read_pending: asyncio.Event | None = None,
) -> None:
    await start_sender.wait()
    if close_requested is None:await state_sender(websocket, control, send_lock)
    else:await state_sender(websocket, control, send_lock,close_requested=close_requested,read_pending=read_pending)


async def diagnostics_sender(
    websocket: WebSocket,
    control: RobotControl,
    store: DiagnosticsStore,
    runtime: RuntimeBackend,
    log_session_dir: str | None,
    send_lock: asyncio.Lock,
) -> None:
    while True:
        message = store.message(
            runtime=runtime,
            hardware_verified=False,
            server_mono_ns=control.clock.now_ns(),
            control_generation=control.control_generation,
            log_session_dir=log_session_dir,
        )
        await _send_json(websocket, message.model_dump(mode="json"), send_lock)
        await asyncio.sleep(0.2)


async def _delayed_diagnostics_sender(
    websocket: WebSocket,
    control: RobotControl,
    store: DiagnosticsStore,
    runtime: RuntimeBackend,
    log_session_dir: str | None,
    start_sender: asyncio.Event,
    send_lock: asyncio.Lock,
) -> None:
    await start_sender.wait()
    await diagnostics_sender(
        websocket,
        control,
        store,
        runtime,
        log_session_dir,
        send_lock,
    )


async def _protocol_error(websocket: WebSocket, send_lock: asyncio.Lock) -> None:
    message = "消息格式无效，请检查协议版本和字段。"
    async with send_lock:
        await websocket.send_json(
            {"v": 1, "type": "protocol_error", "message": message}
        )
        await websocket.close(code=1008, reason=message)


def _parse_message(payload: Any) -> ClientMessage:
    if not isinstance(payload, dict):
        raise ValueError("message_must_be_object")
    message_type = payload.get("type")
    if message_type=='finite_position_input':
        return FinitePositionInputMessage.model_validate(payload)
    if message_type == "vr_frame":
        return VRFrame.model_validate(payload)
    if message_type == 'gripper_request':
        return GripperRequestMessage.model_validate(payload)
    if message_type == 'gripper_cancel':
        return GripperCancelMessage.model_validate(payload)
    if message_type == "offline_rehearsal_begin":
        return OfflineRehearsalBeginMessage.model_validate(payload)
    if message_type == "offline_rehearsal_phase":
        return OfflineRehearsalPhaseMessage.model_validate(payload)
    if message_type == "offline_rehearsal_finish":
        return OfflineRehearsalFinishMessage.model_validate(payload)
    if message_type == "set_simulation_scale":
        return SetSimulationScaleMessage.model_validate(payload)
    return ClientControlMessage.model_validate(payload)


async def _rehearsal_snapshot(
    websocket: WebSocket,
    control: RobotControl,
) -> tuple[object, object]:
    app_state = websocket.app.state
    diagnostics: DiagnosticsStore = app_state.diagnostics
    state = await control.state_message()
    diagnostic_message = diagnostics.message(
        runtime=app_state.runtime_backend,
        hardware_verified=False,
        server_mono_ns=control.clock.now_ns(),
        control_generation=control.control_generation,
        log_session_dir=app_state.log_session_dir,
    )
    return state, diagnostic_message


def _rehearsal_reject_reason(operation: str, error: Exception) -> str:
    if isinstance(error, PermissionError):
        return "owner_mismatch"
    detail = str(error)
    if "only for LEBAI_FAKE" in detail:
        return "not_fake_runtime"
    if "already has an active" in detail:
        return "run_already_active"
    if "run is not active" in detail:
        return "run_not_active"
    if "phase order" in detail:
        return "phase_out_of_order"
    if "all phases" in detail or "passed report requires" in detail:
        return "run_incomplete"
    if isinstance(error, OSError):
        return "report_write_failed"
    return f"{operation}_rejected"


async def _handle_rehearsal_message(
    websocket: WebSocket,
    control: RobotControl,
    owner_token: object,
    message: (
        OfflineRehearsalBeginMessage
        | OfflineRehearsalPhaseMessage
        | OfflineRehearsalFinishMessage
    ),
    send_lock: asyncio.Lock,
    rehearsal_active: asyncio.Event | None = None,
) -> None:
    if isinstance(message, OfflineRehearsalBeginMessage):
        await _send_json(websocket,dict(v=1,type='offline_rehearsal_begin_result',
            request_id=message.request_id,accepted=False,reason='feature_removed'),send_lock)
        return

    # Historical report finishing/reading remains available; no new automated
    # rehearsal can acquire a control owner through the retired entry.
    store: RehearsalReportStore = websocket.app.state.offline_rehearsal_store
    state, diagnostics = await _rehearsal_snapshot(websocket, control)

    if isinstance(message, OfflineRehearsalPhaseMessage):
        phase_result = message.model_dump(
            mode="json",
            exclude={"v", "type", "request_id", "run_id"},
        )
        try:
            await store.record_phase(
                owner_token,
                message.run_id,
                phase_result,
                state,
                diagnostics,
            )
        except Exception as error:
            payload = {
                "v": 1,
                "type": "offline_rehearsal_phase_ack",
                "request_id": message.request_id,
                "accepted": False,
                "run_id": message.run_id,
                "phase": message.phase,
                "reason": _rehearsal_reject_reason("phase", error),
            }
        else:
            payload = {
                "v": 1,
                "type": "offline_rehearsal_phase_ack",
                "request_id": message.request_id,
                "accepted": True,
                "run_id": message.run_id,
                "phase": message.phase,
            }
        await _send_json(websocket, payload, send_lock)
        return

    try:
        result = await store.finish(
            owner_token,
            message.run_id,
            message.outcome,
            message.failure,
            state,
            diagnostics,
        )
    except Exception as error:
        payload = {
            "v": 1,
            "type": "offline_rehearsal_finish_result",
            "request_id": message.request_id,
            "accepted": False,
            "run_id": message.run_id,
            "reason": _rehearsal_reject_reason("finish", error),
        }
    else:
        if rehearsal_active is not None:
            rehearsal_active.clear()
        payload = {
            "v": 1,
            "type": "offline_rehearsal_finish_result",
            "request_id": message.request_id,
            "accepted": True,
            "run_id": message.run_id,
            "outcome": message.outcome,
            "json_path": str(result.json_path),
            "markdown_path": str(result.markdown_path),
            "hardware_verified": False,
            "hardware_pending": list(HARDWARE_PENDING),
        }
    await _send_json(websocket, payload, send_lock)


async def _rehearsal_worker(
    websocket: WebSocket,
    control: RobotControl,
    owner_token: object,
    messages: asyncio.Queue[
        OfflineRehearsalBeginMessage
        | OfflineRehearsalPhaseMessage
        | OfflineRehearsalFinishMessage
    ],
    send_lock: asyncio.Lock,
    rehearsal_active: asyncio.Event | None,
) -> None:
    while True:
        message = await messages.get()
        try:
            await _handle_rehearsal_message(
                websocket,
                control,
                owner_token,
                message,
                send_lock,
                rehearsal_active,
            )
        finally:
            messages.task_done()


async def _receive_messages(
    websocket: WebSocket,
    control: RobotControl,
    start_sender: asyncio.Event,
    send_lock: asyncio.Lock,
    owner_token: object | None = None,
    rehearsal_active: asyncio.Event | None = None,
    rehearsal_messages=None,
    *,
    close_requested: asyncio.Event | None = None,
) -> None:
    async def send(payload):
        await _send_json(websocket,payload,send_lock)
    jobs=OperatorJobs(control,send,allowed=lambda:
        (close_requested is None or not close_requested.is_set())
        and (rehearsal_active is None or not rehearsal_active.is_set()))
    try:
        await _receive_operator_messages(websocket,control,start_sender,send_lock,
            owner_token,rehearsal_active,rehearsal_messages,jobs,close_requested=close_requested)
    finally:
        with anyio.CancelScope(shield=True):
            await jobs.close()


async def _receive_operator_messages(
    websocket: WebSocket,
    control: RobotControl,
    start_sender: asyncio.Event,
    send_lock: asyncio.Lock,
    owner_token: object | None = None,
    rehearsal_active: asyncio.Event | None = None,
    rehearsal_messages: asyncio.Queue[
        OfflineRehearsalBeginMessage
        | OfflineRehearsalPhaseMessage
        | OfflineRehearsalFinishMessage
    ] | None = None,
    jobs: OperatorJobs | None = None,
    *,
    close_requested: asyncio.Event | None = None,
) -> None:
    app = websocket.app
    assert jobs is not None
    while True:
        if close_requested is not None and close_requested.is_set():
            return
        jobs.check_error()
        try:
            payload = await websocket.receive_json()
            if close_requested is not None and close_requested.is_set():
                return
            message = _parse_message(payload)
        except WebSocketDisconnect:
            return
        except (ValidationError, ValueError, TypeError):
            await _protocol_error(websocket, send_lock)
            return

        if isinstance(message, VRFrame):
            if (message.session_id.startswith('quest-')
                    and getattr(control.backend,'position_candidate',False)
                    and not getattr(control.backend,'position_vr_enabled',False)):
                await _protocol_error(websocket,send_lock)
                return
            app.state.latest.publish(message, control.clock.now_ns())
            start_sender.set()
            continue

        if isinstance(message,FinitePositionInputMessage):
            received_ns=control.clock.now_ns()
            try:control.accept_position_input(message,received_ns)
            except (RuntimeError,ValueError):
                await _send_json(websocket,dict(v=1,type='position_input_rejected',reason='not_authorized'),send_lock)
            else:
                await control.recorder.write_event(dict(kind='finite_position_input',
                    input_seq=message.seq,generation=message.generation,event=message.kind,
                    vector=list(message.vector),pitch=message.pitch,received_ns=received_ns),received_ns)
            continue

        if isinstance(message,GripperRequestMessage):
            await jobs.gripper(message)
            continue

        if isinstance(message,GripperCancelMessage):
            jobs.cancel_gripper(message.request_id)
            continue

        if isinstance(message, SetSimulationScaleMessage):
            result = await control.set_simulation_scale(
                message.translation_scale,
                automation_active=(
                    rehearsal_active is not None
                    and rehearsal_active.is_set()
                ),
            )
            response = SimulationScaleResultMessage(
                request_id=message.request_id,
                accepted=result.accepted,
                translation_scale=result.translation_scale,
                reason=result.reason,
            )
            await _send_json(
                websocket,
                response.model_dump(mode="json", exclude_none=True),
                send_lock,
            )
            continue

        if isinstance(
            message,
            (
                OfflineRehearsalBeginMessage,
                OfflineRehearsalPhaseMessage,
                OfflineRehearsalFinishMessage,
            ),
        ):
            if owner_token is None:
                await _send_json(
                    websocket,
                    {
                        "v": 1,
                        "type": "offline_rehearsal_begin_result",
                        "request_id": message.request_id,
                        "accepted": False,
                        "reason": "owner_mismatch",
                    },
                    send_lock,
                )
            else:
                if rehearsal_messages is None:
                    raise RuntimeError("rehearsal message worker is unavailable")
                rehearsal_messages.put_nowait(message)
            continue

        if message.type == "hello":
            await _send_json(
                websocket,
                {"v": 1, "type": "hello_ack", "request_id": message.request_id},
                send_lock,
            )
            start_sender.set()
        elif message.type == "prepare_request":
            await jobs.prepare(message)
        elif message.type == "arm_request":
            if getattr(control,'_prepare_on_arm',False):
                await jobs.arm(message)
                continue
            try:
                await control.arm()
            except RuntimeError:
                await _send_json(
                    websocket,
                    {
                        "v": 1,
                        "type": "arm_rejected",
                        "request_id": message.request_id,
                        "message": "请先松开手柄抓握键，再请求使能。",
                    },
                    send_lock,
                )
            else:
                await _send_json(
                    websocket,
                    {"v": 1, "type": "arm_ack", "request_id": message.request_id},
                    send_lock,
                )
        elif message.type == "disarm":
            jobs.cancel_gripper()
            await control.disarm()
            await _send_json(
                websocket,
                {"v": 1, "type": "disarm_ack", "request_id": message.request_id},
                send_lock,
            )
        elif message.type == "reset_fault":
            try:
                result = await control.reset_fault()
            except Exception:
                await _send_json(
                    websocket,
                    {
                        "v": 1,
                        "type": "fault_reset_result",
                        "request_id": message.request_id,
                        "accepted": False,
                        "reason": "unrecoverable_fault",
                        "message": "该故障无法在线复位，请重启后端并重新检查。",
                    },
                    send_lock,
                )
            else:
                if result.accepted:
                    async with send_lock:
                        state = await control.state_message()
                        await websocket.send_json(state.model_dump(mode="json"))
                        await websocket.send_json(
                            {
                                "v": 1,
                                "type": "fault_reset_result",
                                "request_id": message.request_id,
                                "accepted": True,
                                "mode": "DISARMED",
                            }
                        )
                else:
                    await _send_json(
                        websocket,
                        {
                            "v": 1,
                            "type": "fault_reset_result",
                            "request_id": message.request_id,
                            "accepted": False,
                            "reason": result.reason,
                            "message": result.message,
                        },
                        send_lock,
                    )
        elif message.type == "home_request":
            try:
                result = await control.home()
            except Exception:
                await _send_json(
                    websocket,
                    {
                        "v": 1,
                        "type": "home_result",
                        "request_id": message.request_id,
                        "accepted": False,
                        "reason": "home_failed",
                        "message": "仿真无法返回初始姿态，请稍后重试。",
                    },
                    send_lock,
                )
            else:
                if result.accepted:
                    payload = {
                        "v": 1,
                        "type": "home_result",
                        "request_id": message.request_id,
                        "accepted": True,
                        "mode": "DISARMED",
                    }
                else:
                    payload = {
                        "v": 1,
                        "type": "home_result",
                        "request_id": message.request_id,
                        "accepted": False,
                        "reason": result.reason,
                        "message": result.message,
                    }
                await _send_json(websocket, payload, send_lock)
        elif message.type == "ping":
            await _send_json(
                websocket,
                {"v": 1, "type": "pong", "request_id": message.request_id},
                send_lock,
            )


async def _drain_idle_web_close(control,tasks,sender,close_requested,read_pending,*,revoke_generation=True):
    """Let only existing zero-actuation reads/static stops reach original bounds.

    This does not extend an RPC deadline or start a new read cycle. The
    single server's outer cleanup bound remains in force if a task won't exit.
    """
    if revoke_generation:control._advance_control_generation()
    close_requested.set()
    loop=asyncio.get_running_loop()
    deadline=loop.time()+getattr(control.backend,'_stop_cleanup_grace_s',1.)
    remaining=set(tasks)
    while remaining:
        remaining={task for task in remaining if not task.done()}
        protected={task for task in remaining if (task is sender and read_pending.is_set())
            or task in control._backend_stop_owners}
        for task in remaining-protected:
            if not task.cancelling():task.cancel()
        if not protected or loop.time()>=deadline:break
        await asyncio.wait(protected,timeout=min(.005,deadline-loop.time()))
    for task in remaining:
        if not task.done() and not task.cancelling():task.cancel()


async def _run_coupled_session(
    websocket: WebSocket,
    control: RobotControl,
    sender_tasks: set[asyncio.Task[None]],
    owner_token: object | None = None,
    rehearsal_active: asyncio.Event | None = None,
    session_trace: SessionTrace | None = None,
) -> None:
    session_trace = session_trace or SessionTrace(control)
    start_sender = asyncio.Event()
    close_guard=getattr(getattr(control,'backend',None),'web_idle_close_safe',None)
    idle_web=callable(close_guard)
    close_requested=asyncio.Event() if idle_web else None
    read_pending=asyncio.Event() if idle_web else None
    send_lock = asyncio.Lock()
    rehearsal_messages: asyncio.Queue[
        OfflineRehearsalBeginMessage
        | OfflineRehearsalPhaseMessage
        | OfflineRehearsalFinishMessage
    ] = asyncio.Queue()
    receiver = asyncio.create_task(
        _receive_messages(
            websocket,
            control,
            start_sender,
            send_lock,
            owner_token,
            rehearsal_active,
            rehearsal_messages,
            **({'close_requested':close_requested} if idle_web else {}),
        ),
        name="teleop-receiver",
    )
    sender = asyncio.create_task(
        _delayed_state_sender(websocket, control, start_sender, send_lock,
            **({'close_requested':close_requested,'read_pending':read_pending} if idle_web else {})),
        name="teleop-state-50hz",
    )
    tasks: list[asyncio.Task[None]] = [receiver, sender]
    if owner_token is not None:
        rehearsal_worker = asyncio.create_task(
            _rehearsal_worker(
                websocket,
                control,
                owner_token,
                rehearsal_messages,
                send_lock,
                rehearsal_active,
            ),
            name="teleop-rehearsal-worker",
        )
        tasks.append(rehearsal_worker)
    sender_tasks.add(sender)
    app_state = getattr(getattr(websocket, "app", None), "state", None)
    diagnostics = getattr(app_state, "diagnostics", None)
    runtime = getattr(app_state, "runtime_backend", None)
    if isinstance(diagnostics, DiagnosticsStore) and runtime in {
        "SIMULATOR",
        "LEBAI",
        "LEBAI_FAKE",
        "LEBAI_DOCKER",
    }:
        diagnostics_task = asyncio.create_task(
            _delayed_diagnostics_sender(
                websocket,
                control,
                diagnostics,
                runtime,
                getattr(app_state, "log_session_dir", None),
                start_sender,
                send_lock,
            ),
            name="teleop-diagnostics-5hz",
        )
        tasks.append(diagnostics_task)
        sender_tasks.add(diagnostics_task)
    for task in tasks:
        task.add_done_callback(session_trace.task_done)
    session_trace.mark("tasks_started")
    wait_error: BaseException | None = None
    try:
        with anyio.CancelScope(shield=True):
            await asyncio.wait(
                tasks, return_when=asyncio.FIRST_COMPLETED
            )
    except BaseException as error:
        wait_error = error
    finally:
        try:
            session_trace.mark(
                "cancel_requested",
                cancel_reason=(
                    "external_cancel" if isinstance(wait_error, asyncio.CancelledError)
                    else "wait_error" if wait_error is not None else "peer_completed"
                ),
            )
            drain_idle=(idle_web and wait_error is None and close_guard()
                and control.mode in {TeleopMode.READY,TeleopMode.DISARMED}
                and getattr(control,'_fault',None) is None)
            quiet_guard=getattr(control.backend,'docker_quiet_close_safe',None)
            position_session=control.finite_position_session
            drain_docker=(idle_web and wait_error is None and callable(quiet_guard) and quiet_guard()
                and control.mode in {TeleopMode.READY,TeleopMode.DISARMED,TeleopMode.ARMED,TeleopMode.HOLD}
                and not control._fault and not control._pending_stop_completion
                and not control._recovery_lock.locked() and not control._recovery_phase
                and control._preparation_unlock is None
                and (position_session is None or (position_session.worker is None
                    and position_session.phase in ('ready','hold') and position_session.gate.released)))
            try:
                if drain_docker:
                    # Quiescent completed actions: block all control ticks and
                    # receive/new sends while an existing read reaches its own
                    # deadline. on_disconnect still owns the original stop.
                    control._disconnecting+=1
                    try:
                        with anyio.CancelScope(shield=True):
                            await _drain_idle_web_close(control,tasks,sender,close_requested,read_pending,
                                revoke_generation=False)
                    finally:control._disconnecting-=1
                elif drain_idle:
                    with anyio.CancelScope(shield=True):
                        await _drain_idle_web_close(control,tasks,sender,close_requested,read_pending)
            except BaseException as error:
                if wait_error is None:wait_error=error
            finally:
                # Raw Task.cancel() can interrupt an AnyIO shield. Always join
                # children before disconnect cleanup and owner release.
                for task in tasks:
                    if not task.done():task.cancel()
                with anyio.CancelScope(shield=True):
                    joined=asyncio.gather(*tasks,return_exceptions=True)
                    while not joined.done():
                        try:await asyncio.shield(joined)
                        except asyncio.CancelledError as error:
                            if wait_error is None:wait_error=error
                    results=joined.result()
                    session_trace.mark("tasks_joined")
        finally:
            for task in tasks:
                sender_tasks.discard(task)
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
    backend=control.backend
    if (getattr(backend,'position_policy',None)=='finite_position_docker'
            and (control._fault or backend._latched_fault
                or backend._snapshot_calls.unknown_methods or backend._motion_calls.unknown_methods)):
        message='后端故障已锁存，已停止自动重连；请查看诊断，恢复后刷新页面。'
        await websocket.send_json(dict(v=1,type='connection_rejected',reason='backend_fault',message=message))
        await websocket.close(code=4410,reason='backend_fault')
        return
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
    session_trace = SessionTrace(control)
    session_trace.mark("accepted")
    rehearsal_active = asyncio.Event()
    try:
        if control.mode == TeleopMode.DISCONNECTED:
            await control.connect()
        await _run_coupled_session(
            websocket,
            control,
            app.state.teleop_sender_tasks,
            owner_token,
            rehearsal_active,
            session_trace,
        )
    except BaseException as error:
        session_error = error
    finally:
        with anyio.CancelScope(shield=True):
            async with app.state.teleop_owner_lock:
                owns_connection = app.state.teleop_owner is owner_token
        if owns_connection:
            try:
                try:
                    store = getattr(
                        app.state,
                        "offline_rehearsal_store",
                        None,
                    )
                    if (
                        isinstance(store, RehearsalReportStore)
                        and rehearsal_active.is_set()
                    ):
                        with anyio.CancelScope(shield=True):
                            state, diagnostics = await _rehearsal_snapshot(
                                websocket,
                                control,
                            )
                            await store.abort_owner(
                                owner_token,
                                "connection_closed",
                                state,
                                diagnostics,
                            )
                except BaseException as error:
                    cleanup_error = error
                finally:
                    try:
                        # A closed WebSocket cancels its ASGI scope. The bounded
                        # backend stop must finish independently of that scope;
                        # cancellation is not evidence that the robot stopped.
                        with anyio.CancelScope(shield=True):
                            session_trace.mark("disconnect_started")
                            await control.on_disconnect()
                    except BaseException as error:
                        session_trace.mark(
                            "disconnect_completed",
                            disconnect_outcome=(
                                "cancelled" if isinstance(error, asyncio.CancelledError) else "error"
                            ),
                            disconnect_error_type=type(error).__name__[:128],
                        )
                        if cleanup_error is None:
                            cleanup_error = error
                    else:
                        session_trace.mark("disconnect_completed", disconnect_outcome="returned")
                    finally:
                        latest = LatestVRFrame()
                        control.latest = latest
                        app.state.latest = latest
            finally:
                with anyio.CancelScope(shield=True):
                    async with app.state.teleop_owner_lock:
                        if app.state.teleop_owner is owner_token:
                            app.state.teleop_owner = None
                            session_trace.mark("owner_released")
    if session_error is not None:
        raise session_error
    if cleanup_error is not None:
        raise cleanup_error
