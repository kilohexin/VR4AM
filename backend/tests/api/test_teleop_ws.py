import asyncio
import gc
import json
import threading
import time
import warnings
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
import anyio
try:
    from starlette.exceptions import StarletteDeprecationWarning
except ImportError:
    class StarletteDeprecationWarning(DeprecationWarning):
        """Compatibility category for Starlette versions that removed it."""


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
from app.digital_twin.runtime import create_digital_twin_app
from app.control.robot_control import FaultResetResult, HomeResult, LatestVRFrame
from app.diagnostics.store import DiagnosticsStore
from app.rehearsal.report import (
    HARDWARE_PENDING,
    REHEARSAL_PHASES,
    RehearsalReportStore,
)
from app.schemas.messages import BackendState, Pose, RobotStateMessage, TeleopMode

ROOT = Path(__file__).resolve().parents[3]


async def test_websocket_cancellation_finishes_disconnect_cleanup(monkeypatch):
    from app.api import teleop_ws

    stopping = anyio.Event()
    completed = []

    async def disconnect():
        stopping.set()
        await anyio.sleep(0.01)
        completed.append("stopped")

    control = SimpleNamespace(mode=TeleopMode.DISARMED, on_disconnect=disconnect)
    state = SimpleNamespace(control=control, teleop_owner_lock=asyncio.Lock(),
                            teleop_owner=None, teleop_sender_tasks=set())
    websocket = SimpleNamespace(accept=AsyncMock(), app=SimpleNamespace(state=state))
    monkeypatch.setattr(teleop_ws, "_run_coupled_session", AsyncMock())
    async with anyio.create_task_group() as group:
        group.start_soon(teleop_ws.teleop_websocket, websocket)
        await stopping.wait()
        group.cancel_scope.cancel()
    assert completed == ["stopped"]
    assert state.teleop_owner is None


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


def receive_until_types(ws, required: set[str]) -> dict[str, dict[str, object]]:
    received: dict[str, dict[str, object]] = {}
    for _ in range(20):
        payload = ws.receive_json()
        message_type = payload.get("type")
        if isinstance(message_type, str) and message_type in required:
            received[message_type] = payload
        if required <= received.keys():
            return received
    raise AssertionError(f"missing message types: {required - received.keys()}")


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


def _rehearsal_phase(run_id: str, phase: str) -> dict[str, object]:
    return {
        "v": 1,
        "type": "offline_rehearsal_phase",
        "request_id": f"phase-{phase}",
        "run_id": run_id,
        "phase": phase,
        "status": "passed",
        "started_client_ms": 10.0,
        "completed_client_ms": 20.0,
        "target": {},
        "measurements": {},
        "failure": None,
    }


def _install_rehearsal_store(app, root: Path, runtime: str) -> RehearsalReportStore:
    store = RehearsalReportStore(
        root=root,
        runtime=runtime,
        provenance=lambda: {
            "git": {"commit": "a" * 40, "dirty": False, "dirty_paths": []},
            "model": {},
        },
    )
    app.state.offline_rehearsal_store = store
    return store


def test_rehearsal_begin_phase_finish_are_correlated_and_never_call_motion(
    tmp_path: Path,
) -> None:
    app = create_app(backend_label="LEBAI_FAKE")
    with TestClient(app) as client:
        _install_rehearsal_store(app, tmp_path, "LEBAI_FAKE")
        app.state.control.arm = AsyncMock(wraps=app.state.control.arm)
        app.state.control.home = AsyncMock(wraps=app.state.control.home)
        app.state.control.disarm = AsyncMock(wraps=app.state.control.disarm)
        app.state.backend.command_tcp = AsyncMock(wraps=app.state.backend.command_tcp)
        app.state.backend.set_gripper = AsyncMock(
            wraps=app.state.backend.set_gripper
        )

        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json(
                {
                    "v": 1,
                    "type": "offline_rehearsal_begin",
                    "request_id": "begin-1",
                    "plan_version": 1,
                }
            )
            begun = ws.receive_json()
            assert begun["type"] == "offline_rehearsal_begin_result"
            assert begun["request_id"] == "begin-1"
            assert begun["accepted"] is True
            run_id = begun["run_id"]

            for phase in REHEARSAL_PHASES:
                ws.send_json(_rehearsal_phase(run_id, phase))
                ack = ws.receive_json()
                assert ack == {
                    "v": 1,
                    "type": "offline_rehearsal_phase_ack",
                    "request_id": f"phase-{phase}",
                    "accepted": True,
                    "run_id": run_id,
                    "phase": phase,
                }

            ws.send_json(
                {
                    "v": 1,
                    "type": "offline_rehearsal_finish",
                    "request_id": "finish-1",
                    "run_id": run_id,
                    "outcome": "passed",
                    "failure": None,
                }
            )
            finished = ws.receive_json()

        assert finished == {
            "v": 1,
            "type": "offline_rehearsal_finish_result",
            "request_id": "finish-1",
            "accepted": True,
            "run_id": run_id,
            "outcome": "passed",
            "json_path": str(tmp_path / f"{run_id}.json"),
            "markdown_path": str(tmp_path / f"{run_id}.md"),
            "hardware_verified": False,
            "hardware_pending": list(HARDWARE_PENDING),
        }
        app.state.control.arm.assert_not_awaited()
        app.state.control.home.assert_not_awaited()
        app.state.control.disarm.assert_not_awaited()
        app.state.backend.command_tcp.assert_not_awaited()
        app.state.backend.set_gripper.assert_not_awaited()


@pytest.mark.asyncio
@pytest.mark.parametrize("worker_start_delay", [0.0, 0.10])
async def test_slow_rehearsal_provenance_does_not_block_frames_or_control_ticks(
    tmp_path: Path,
    worker_start_delay: float,
) -> None:
    from app.api import teleop_ws

    loop = asyncio.get_running_loop()
    slow_started = asyncio.Event()
    release_provenance = threading.Event()
    slow_finished = threading.Event()

    def slow_provenance() -> dict[str, object]:
        time.sleep(worker_start_delay)
        loop.call_soon_threadsafe(slow_started.set)
        try:
            if not release_provenance.wait(2.0):
                raise TimeoutError("test did not release provenance worker")
            return {
                "git": {"commit": "a" * 40, "dirty": False, "dirty_paths": []},
                "model": {},
            }
        finally:
            slow_finished.set()

    store = RehearsalReportStore(
        root=tmp_path,
        runtime="LEBAI_FAKE",
        provenance=slow_provenance,
    )
    latest = LatestVRFrame()
    diagnostics = DiagnosticsStore()
    incoming: asyncio.Queue[dict[str, object]] = asyncio.Queue()
    sent: list[dict[str, object]] = []
    ticks: list[float] = []

    class Control:
        mode = TeleopMode.READY
        control_generation = 0
        clock = SimpleNamespace(now_ns=lambda: 456)

        async def state_message(self) -> RobotStateMessage:
            ticks.append(asyncio.get_running_loop().time())
            return _disarmed_robot_state()

    class Socket:
        app = SimpleNamespace(
            state=SimpleNamespace(
                latest=latest,
                offline_rehearsal_store=store,
                diagnostics=diagnostics,
                runtime_backend="LEBAI_FAKE",
                log_session_dir=None,
                settings=SimpleNamespace(state_hz=50, backend="simulator"),
            )
        )

        async def receive_json(self) -> dict[str, object]:
            return await incoming.get()

        async def send_json(self, payload: dict[str, object]) -> None:
            sent.append(payload)

    await incoming.put({"v": 1, "type": "hello", "request_id": "hello"})
    await incoming.put(_valid_frame(seq=1))
    await incoming.put({
        "v": 1,
        "type": "offline_rehearsal_begin",
        "request_id": "slow-begin",
        "plan_version": 1,
    })
    session = asyncio.create_task(
        teleop_ws._run_coupled_session(Socket(), Control(), set(), object(), asyncio.Event())
    )
    try:
        # Establish the blocked-work interval before sending the frames under
        # test. Thread startup order is not a production latency guarantee.
        await asyncio.wait_for(slow_started.wait(), timeout=1.0)
        assert not slow_finished.is_set()
        ticks_before = len(ticks)
        await incoming.put(_valid_frame(seq=2))
        await incoming.put(_valid_frame(seq=3))

        async def wait_for_progress() -> None:
            while (
                latest.snapshot() is None or latest.snapshot().frame.seq != 3
                or len(ticks) < ticks_before + 2
            ):
                await asyncio.sleep(0.002)

        await asyncio.wait_for(wait_for_progress(), timeout=1.0)
        snapshot = latest.snapshot()
        assert snapshot is not None and snapshot.frame.seq == 3
        assert len(ticks) >= ticks_before + 2
        assert not slow_finished.is_set()
        assert not any(message.get("type") == "offline_rehearsal_begin_result" for message in sent)
        release_provenance.set()

        async def wait_for_message(message_type: str) -> None:
            while not any(message.get("type") == message_type for message in sent):
                await asyncio.sleep(0.002)

        await asyncio.wait_for(wait_for_message("offline_rehearsal_begin_result"), timeout=1.0)
        begun = next(
            message for message in sent
            if message.get("type") == "offline_rehearsal_begin_result"
        )
        assert begun["request_id"] == "slow-begin"
        assert begun["accepted"] is True
        run_id = str(begun["run_id"])
        await incoming.put(_rehearsal_phase(run_id, REHEARSAL_PHASES[0]))
        await asyncio.wait_for(wait_for_message("offline_rehearsal_phase_ack"), timeout=1.0)
        ack = next(
            message for message in sent
            if message.get("type") == "offline_rehearsal_phase_ack"
        )
        assert ack["request_id"] == f"phase-{REHEARSAL_PHASES[0]}"
        assert ack["run_id"] == run_id
        assert ack["phase"] == REHEARSAL_PHASES[0]
    finally:
        release_provenance.set()
        session.cancel()
        with pytest.raises(asyncio.CancelledError):
            await session


def test_rehearsal_begin_rejects_non_fake_without_closing_socket(tmp_path: Path) -> None:
    app = create_app()
    with TestClient(app) as client:
        _install_rehearsal_store(app, tmp_path, "SIMULATOR")
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json(
                {
                    "v": 1,
                    "type": "offline_rehearsal_begin",
                    "request_id": "begin-sim",
                    "plan_version": 1,
                }
            )
            assert ws.receive_json() == {
                "v": 1,
                "type": "offline_rehearsal_begin_result",
                "request_id": "begin-sim",
                "accepted": False,
                "reason": "not_fake_runtime",
            }
            ws.send_json({"v": 1, "type": "ping", "request_id": "still-open"})
            assert ws.receive_json() == {
                "v": 1,
                "type": "pong",
                "request_id": "still-open",
            }


def test_rehearsal_wrong_run_and_owner_are_rejected_without_socket_close(
    tmp_path: Path,
) -> None:
    app = create_app(backend_label="LEBAI_FAKE")
    with TestClient(app) as client:
        store = _install_rehearsal_store(app, tmp_path, "LEBAI_FAKE")
        state = client.portal.call(app.state.control.state_message)
        diagnostics = app.state.diagnostics.message(
            runtime="LEBAI_FAKE",
            hardware_verified=False,
            server_mono_ns=app.state.control.clock.now_ns(),
            control_generation=app.state.control.control_generation,
            log_session_dir=app.state.log_session_dir,
        )
        other_run = client.portal.call(
            store.begin, object(), 1, state, diagnostics
        ).run_id

        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json(_rehearsal_phase("missing-run", REHEARSAL_PHASES[0]))
            missing = ws.receive_json()
            assert missing["accepted"] is False
            assert missing["reason"] == "run_not_active"

            ws.send_json(_rehearsal_phase(other_run, REHEARSAL_PHASES[0]))
            owner = ws.receive_json()
            assert owner["accepted"] is False
            assert owner["reason"] == "owner_mismatch"

            ws.send_json({"v": 1, "type": "ping", "request_id": "still-open"})
            assert ws.receive_json()["type"] == "pong"


def test_rehearsal_disconnect_persists_aborted_report(tmp_path: Path) -> None:
    app = create_app(backend_label="LEBAI_FAKE")
    with TestClient(app) as client:
        _install_rehearsal_store(app, tmp_path, "LEBAI_FAKE")
        with client.websocket_connect("/ws/v1/teleop") as ws:
            ws.send_json(
                {
                    "v": 1,
                    "type": "offline_rehearsal_begin",
                    "request_id": "begin-abort",
                    "plan_version": 1,
                }
            )
            run_id = ws.receive_json()["run_id"]

        payload = json.loads(
            (tmp_path / f"{run_id}.json").read_text(encoding="utf-8")
        )
        assert payload["outcome"] == "aborted"
        assert payload["failure"] == {"reason": "connection_closed"}
        assert payload["hardware_verified"] is False


def _receive_reset_sequence(ws, limit: int = 8) -> list[dict[str, object]]:
    messages: list[dict[str, object]] = []
    for _ in range(limit):
        message = ws.receive_json()
        if message.get("type") in {"robot_state", "fault_reset_result"}:
            messages.append(message)
        if message.get("type") == "fault_reset_result":
            return messages
    raise AssertionError("未收到 fault_reset_result")


def test_diagnostics_are_sent_only_to_owner_and_sender_tasks_are_cleaned_up() -> None:
    app = create_app(backend_label="LEBAI_FAKE")
    with TestClient(app) as client:
        with client.websocket_connect("/ws/v1/teleop") as owner:
            owner.send_json({"v": 1, "type": "hello", "request_id": "owner"})
            assert owner.receive_json()["type"] == "hello_ack"
            messages = receive_until_types(owner, {"robot_state", "diagnostics"})

            assert messages["diagnostics"]["runtime"] == "LEBAI_FAKE"
            assert messages["diagnostics"]["hardware_verified"] is False
            assert app.state.teleop_owner is not None

            with client.websocket_connect("/ws/v1/teleop") as intruder:
                rejection = intruder.receive_json()
                assert rejection["type"] == "connection_rejected"
                close = intruder.receive()
                assert close["type"] == "websocket.close"
                assert close["code"] == 4409

        assert app.state.teleop_owner is None
        assert app.state.teleop_sender_tasks == set()


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


def test_simulation_scale_request_returns_authoritative_correlated_result() -> None:
    app = create_digital_twin_app()
    with TestClient(app) as client, client.websocket_connect("/ws/v1/teleop") as ws:
        ws.send_json(_valid_frame(seq=1))
        _receive_until(ws, "robot_state")
        ws.send_json(
            {
                "v": 1,
                "type": "set_simulation_scale",
                "request_id": "scale-1",
                "translation_scale": 1.7,
            }
        )
        result = _receive_until(ws, "simulation_scale_result")

    assert result == {
        "v": 1,
        "type": "simulation_scale_result",
        "request_id": "scale-1",
        "accepted": True,
        "translation_scale": 1.7,
    }


def test_real_runtime_rejects_simulation_scale_without_closing_socket() -> None:
    app = create_app()
    with TestClient(app) as client, client.websocket_connect("/ws/v1/teleop") as ws:
        ws.send_json(
            {
                "v": 1,
                "type": "set_simulation_scale",
                "request_id": "scale-real",
                "translation_scale": 1.7,
            }
        )
        result = _receive_until(ws, "simulation_scale_result")
        ws.send_json({"v": 1, "type": "ping", "request_id": "still-open"})
        pong = _receive_until(ws, "pong")

    assert result["accepted"] is False
    assert result["reason"] == "not_simulation"
    assert pong["request_id"] == "still-open"


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
@pytest.mark.parametrize(
    ("backend", "runtime", "expected_period_s"),
    [
        ("simulator", "SIMULATOR", 0.02),
        ("lebai", "LEBAI", 0.04),
        ("lebai", "LEBAI_FAKE", 0.04),
    ],
)
async def test_state_sender_selects_runtime_configured_state_rate(
    monkeypatch: pytest.MonkeyPatch,
    backend: str,
    runtime: str,
    expected_period_s: float,
) -> None:
    from app.api import teleop_ws

    sleeps: list[float] = []
    settings = SimpleNamespace(
        backend=backend,
        state_hz=50,
        lebai=SimpleNamespace(control=SimpleNamespace(state_hz=25)),
    )

    class FakeWebSocket:
        app = SimpleNamespace(
            state=SimpleNamespace(settings=settings, runtime_backend=runtime)
        )

        async def send_json(self, _payload: dict[str, object]) -> None:
            return None

    class FakeMessage:
        def model_dump(self, *, mode: str) -> dict[str, object]:
            return {"type": "robot_state", "mode": mode}

    class FakeControl:
        async def state_message(self) -> FakeMessage:
            return FakeMessage()

    async def fake_sleep(delay: float) -> None:
        sleeps.append(delay)
        raise asyncio.CancelledError

    monkeypatch.setattr(teleop_ws.asyncio, "sleep", fake_sleep)

    with pytest.raises(asyncio.CancelledError):
        await teleop_ws.state_sender(FakeWebSocket(), FakeControl())

    assert sleeps == [expected_period_s]


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
