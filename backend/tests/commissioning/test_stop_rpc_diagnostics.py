import json
import asyncio
from unittest.mock import AsyncMock

import pytest

from app.commissioning.actions import PrepareAction, SmokeOptions, StopAction, TranslationAction
from app.commissioning.smoke import parse_smoke_args, run_smoke
from app.config import REAL_ROBOT_CONFIRMATION
from tests.commissioning.test_actions import _control_config
from tests.robots.fake_lebai import FakeLebaiClient
from tests.commissioning.test_stop_timeout_lifecycle import PendingStopClient


def test_stop_rpc_diagnostics_requires_explicit_cli_opt_in():
    args = ["stop", "--config", "unused.yaml", "--confirm", REAL_ROBOT_CONFIRMATION]
    assert parse_smoke_args(args).stop_rpc_diagnostics is False
    assert parse_smoke_args(args + ["--stop-rpc-diagnostics"]).stop_rpc_diagnostics is True


@pytest.mark.parametrize("enabled", [False, True])
async def test_smoke_records_effective_diagnostic_policy_and_emits_only_if_enabled(tmp_path, monkeypatch, enabled):
    client = FakeLebaiClient.idle()
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    result = await run_smoke(SmokeOptions(_control_config(tmp_path), StopAction(),
        REAL_ROBOT_CONFIRMATION, stop_rpc_diagnostics=enabled), AsyncMock(return_value=client))
    assert result.stable
    folder, = (tmp_path / "logs").iterdir()
    events = [json.loads(line) for line in (folder / "session.jsonl").read_text().splitlines()]
    metadata = next(e for e in events if e["kind"] == "session_started")["metadata"]
    assert metadata["stop_rpc_diagnostics"] == {
        "enabled": enabled, "schema_version": 1, "read_timeout_ms": 100,
        "duration_ms": 2000, "interval_ms": 100, "cleanup_timeout_ms": 50,
        "assessment": "diagnostic_only",
    }
    reads = [e for e in events if e["kind"] == "stop_diagnostic_read" and e["outcome"] == "sample"]
    assert bool(reads) is enabled
    assert [c[0] for c in client.write_calls] == ["stop_move"]


async def test_prepare_rejects_diagnostic_opt_in_before_configuration_or_connection(tmp_path):
    factory = AsyncMock()
    with pytest.raises(ValueError, match="smoke_stop_rpc_diagnostics_action_not_supported"):
        await run_smoke(SmokeOptions(tmp_path / "missing.yaml", PrepareAction(),
            REAL_ROBOT_CONFIRMATION, stop_rpc_diagnostics=True), factory)
    factory.assert_not_awaited()


def test_translation_diagnostics_and_observation_are_independent_cli_options():
    options = parse_smoke_args(["translate", "--config", "unused.yaml", "--axis", "x",
        "--distance-m", "0.002", "--confirm", REAL_ROBOT_CONFIRMATION,
        "--stop-rpc-diagnostics", "--observe-stop-seconds", "60"])
    assert options.stop_rpc_diagnostics
    assert options.observe_stop_seconds == 60
    assert options.action == TranslationAction("x", .002)


@pytest.mark.parametrize("value", ["true", 1, None])
async def test_non_boolean_programmatic_option_rejected_before_connection(tmp_path, value):
    factory = AsyncMock()
    with pytest.raises(ValueError, match="smoke_stop_rpc_diagnostics_invalid"):
        await run_smoke(SmokeOptions(tmp_path / "missing.yaml", StopAction(),
            REAL_ROBOT_CONFIRMATION, stop_rpc_diagnostics=value), factory)
    factory.assert_not_awaited()


async def test_cli_enabled_translation_records_early_reads_but_preserves_stop_failure(tmp_path, monkeypatch):
    client = PendingStopClient("never")
    monkeypatch.setattr("app.commissioning.smoke.COMMISSIONING_LOG_ROOT", tmp_path / "logs")
    options = parse_smoke_args(["translate", "--config", str(_control_config(tmp_path)),
        "--axis", "x", "--distance-m", "0.002", "--confirm", REAL_ROBOT_CONFIRMATION,
        "--stop-rpc-diagnostics", "--observe-stop-seconds", "0.05"])
    try:
        with pytest.raises(RuntimeError, match="smoke_stop_failed:stop_unverified"):
            await asyncio.wait_for(run_smoke(options, AsyncMock(return_value=client)), 12)
        folder, = (tmp_path / "logs").iterdir()
        events = [json.loads(line) for line in (folder / "session.jsonl").read_text().splitlines()]
        first_stop = next(e for e in events if e["kind"] == "stop_diagnostics")
        first_read = next(e for e in events if e["kind"] == "stop_diagnostic_read" and e["outcome"] == "sample")
        call = first_stop["rpc_calls"][0]
        assert call["started_ns"] <= first_read["started_ns"] < first_read["completed_ns"] < call["completed_ns"]
        assert first_stop["latched_fault"] == "stop_unverified"
        assert client.stops == 2
        assert not any(e["kind"] == "stop_confirmed" for e in events)
        tail = next(e for e in events if e["kind"] == "stop_observation_summary")
        assert tail["complete"]
    finally:
        await client.cleanup()
