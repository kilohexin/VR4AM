"""Offline policy checks, not a model of physical braking."""
import asyncio
from unittest.mock import AsyncMock

import pytest

from app.commissioning.pause_comparison import PauseComparisonAdapter
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from tests.robots.fake_lebai import FakeLebaiClient
from tests.robots.real_settings import control_settings
from tests.robots.test_lebai_adapter_control import FakeClock


class PauseClient(FakeLebaiClient):
    async def pause_move(self):
        self.write_calls.append(("pause_move",))
        self.robot_state = "PAUSED"


async def make_adapter(client, *, normal=False):
    clock = FakeClock()
    events = []

    async def record(event, _ns):
        events.append(event)

    adapter = (RealLebaiAdapter if normal else PauseComparisonAdapter)(
        control_settings(), client_factory=AsyncMock(return_value=client),
        clock=clock.now_ns, sleep=clock.sleep, event_callback=record,
    )
    await adapter.connect()
    return adapter, events


async def test_pause_reuses_one_request_at_shutdown_and_reports_actual_method():
    client = PauseClient()
    adapter, events = await make_adapter(client)
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        await adapter.stop(StopReason.SHUTDOWN)
        assert client.write_calls == [("pause_move",)]
        diagnostics = [e for e in events if e["kind"] == "stop_diagnostics"]
        assert len(diagnostics) == 2
        assert all(e["stop_policy"] == "pause_move_only" for e in diagnostics)
        assert all("pause_move_lock" in e and "stop_move_lock" not in e for e in diagnostics)
        assert all(e["rpc_calls"][0]["method"] == "pause_move" for e in diagnostics)
        lifecycle = [e for e in events if e["kind"] == "stop_rpc_lifecycle"]
        assert len(lifecycle) == 1 and lifecycle[0]["method"] == "pause_move"
        assert (await adapter.preflight()).ready is False
    finally:
        await adapter.disconnect()


async def test_production_keeps_stop_move_even_when_pause_is_available():
    client = PauseClient()
    adapter, events = await make_adapter(client, normal=True)
    try:
        await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == [("stop_move",)]
        assert next(e for e in events if e["kind"] == "stop_diagnostics")["stop_policy"] == "stop_move_only"
    finally:
        await adapter.disconnect()


async def test_missing_pause_capability_cannot_admit_motion_or_send_fallback():
    client = FakeLebaiClient()
    adapter, _ = await make_adapter(client)
    try:
        preflight = await adapter.preflight()
        assert not preflight.ready and preflight.reason == "pause_move_unavailable"
        with pytest.raises(BackendCommandError, match="pause_move_unavailable"):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert client.write_calls == []
    finally:
        await adapter.disconnect()


async def test_verified_pause_finishes_smoke_without_relabelling_hold_as_idle(tmp_path, monkeypatch):
    from app.commissioning import smoke
    from app.commissioning.actions import SmokeOptions, StopAction
    from app.commissioning.pause_comparison import build_pause_backend, pause_is_settled
    from app.config import REAL_ROBOT_CONFIRMATION
    from app.schemas.messages import BackendState
    from tests.commissioning.test_actions import _control_config
    monkeypatch.setattr(smoke, "COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    client = PauseClient()
    result = await smoke.run_smoke(
        SmokeOptions(_control_config(tmp_path), StopAction(), REAL_ROBOT_CONFIRMATION),
        AsyncMock(return_value=client), backend_builder=build_pause_backend,
        settled_state_check=pause_is_settled,
    )
    assert result.after.robot_state is BackendState.HOLD
    assert client.write_calls == [("pause_move",)]


async def test_post_pause_settling_rejects_drift_unverified_and_other_hold():
    from app.commissioning.pause_comparison import pause_is_settled
    client = PauseClient()
    adapter, _ = await make_adapter(client)
    try:
        state = await adapter.get_state()
        assert not pause_is_settled(adapter, state)
        await adapter.stop(StopReason.GRIP_RELEASED)
        state = await adapter.get_state()
        assert pause_is_settled(adapter, state)
        from app.commissioning.smoke import _wait_for_stable_state
        settled = await _wait_for_stable_state(adapter, 1., settled_state_check=pause_is_settled)
        assert settled.robot_state == state.robot_state
        client.kin_data["actual_joint_pose"][0] += .002
        assert not pause_is_settled(adapter, await adapter.get_state())
        client.robot_state = "STOP"
        assert not pause_is_settled(adapter, await adapter.get_state())
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize("mode", ["late", "error", "moving", "drift"])
async def test_pause_failure_latches_and_never_resumes_or_escalates(mode):
    client = PauseClient()
    release = asyncio.Event()

    async def pause():
        client.write_calls.append(("pause_move",))
        if mode == "late":
            await release.wait()
        if mode == "error":
            raise RuntimeError("injected_pause_error")
        client.robot_state = "MOVING" if mode == "moving" else "PAUSED"

    client.pause_move = pause
    if mode == "drift":
        original = client.get_kin_data

        async def kin():
            if client.write_calls:
                client.kin_data["actual_joint_pose"][0] += .002
            return await original()

        client.get_kin_data = kin
    adapter, events = await make_adapter(client)
    try:
        with pytest.raises(BackendCommandError):
            await adapter.stop(StopReason.GRIP_RELEASED)
        assert adapter._latched_fault in {"stop_unverified", "stop_incomplete"}
        fault = adapter._latched_fault
        release.set()
        await asyncio.sleep(0)
        with pytest.raises(BackendCommandError):
            await adapter.stop(StopReason.SHUTDOWN)
        assert adapter._latched_fault == fault
        assert client.write_calls == [("pause_move",)]
        assert not any(e["kind"] == "stop_confirmed" for e in events)
        if mode == "late":
            call = next(e for e in events if e["kind"] == "stop_diagnostics")["rpc_calls"][0]
            assert call["timeout_ms"] == 200
            assert call["method"] == "pause_move" and call["outcome"] == "timeout"
    finally:
        release.set()
        await adapter.disconnect()
