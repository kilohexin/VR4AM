from datetime import UTC, datetime, timedelta
import importlib.util
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE

SOURCE = Path(__file__).resolve().parents[3] / "scripts/real_robot_pause_comparison.py"


@pytest.fixture
def context(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("pause_entry", SOURCE)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    monkeypatch.setattr(entry, "ROOT", tmp_path)
    monkeypatch.setattr(entry.idle, "ROOT", tmp_path)
    monkeypatch.setattr(entry.idle, "runtime_hashes", lambda: {})
    monkeypatch.setattr(entry, "runtime_hashes", lambda: {"synthetic": "a" * 64})
    monkeypatch.setattr(entry, "existing_rpc_connections", lambda: 0)
    data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode="control"))
    data["real_robot"]["ip"] = "192.0.2.1"
    data["real_robot"]["control"].update(entry.CAPS)
    config = tmp_path / "control.yaml"
    config.write_text(yaml.safe_dump(data), encoding="utf-8")
    expires = (datetime.now(UTC) + timedelta(minutes=5)).isoformat()
    plan = entry.prepare(config, tmp_path / ".tmp/once", expires=expires)
    approved = tmp_path / "preview.json"
    approved.write_text(json.dumps(entry.preview(plan)), encoding="utf-8")
    kwargs = dict(approved=approved, expected_hash=entry.sha(approved),
                  expires=expires, confirm=entry.CONFIRM)
    return entry, plan, kwargs


def test_preview_is_nonexecuting_and_fixed_to_pause_z3mm(context, monkeypatch, capsys):
    entry, plan, _ = context
    monkeypatch.setattr(entry, "execute", lambda **_: pytest.fail("execution forbidden"))
    assert entry.main(["--config", str(plan["config"]), "--output", str(plan["output"])]) == 0
    preview = json.loads(capsys.readouterr().out)
    assert preview["allowed_writes"] == ["move_pvat", "pause_move"]
    assert preview["action"]["distance_m"] == .003
    assert preview["stop_rpc_timeout_ms"] == 200
    assert preview["automatic_resume"] is False and preview["packet_capture"] is False
    assert not plan["output"].exists()


@pytest.mark.parametrize("change", [
    {"confirm": "yes"}, {"expires": "2000-01-01T00:00:00Z"},
    {"expires": "2099-01-01T00:00:00Z"}, {"expires": "invalid"},
    {"expected_hash": "b" * 64}, {"approved": None},
])
async def test_invalid_approval_cannot_connect_or_consume_attempt(context, change):
    entry, plan, kwargs = context
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **{**kwargs, **change}, client_factory=factory)
    factory.assert_not_called()
    assert not plan["output"].exists()


@pytest.mark.parametrize("change", ["config", "runtime", "scope", "rpc"])
async def test_changed_inputs_or_connection_refuse_before_sdk(context, monkeypatch, change):
    entry, plan, kwargs = context
    if change == "config":
        plan["config"].write_text(plan["config"].read_text() + "\n#changed\n")
    elif change == "runtime":
        monkeypatch.setattr(entry, "runtime_hashes", lambda: {"synthetic": "b" * 64})
    elif change == "scope":
        value = json.loads(kwargs["approved"].read_text())
        value["allowed_writes"] = ["stop_move"]
        kwargs["approved"].write_text(json.dumps(value))
        kwargs["expected_hash"] = entry.sha(kwargs["approved"])
    else:
        monkeypatch.setattr(entry, "existing_rpc_connections", lambda: 1)
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_not_called()
    assert not plan["output"].exists()


def test_caps_cannot_be_increased(context):
    entry, plan, _ = context
    data = yaml.safe_load(plan["config"].read_text())
    data["real_robot"]["control"]["max_joint_speed_radps"] = .06
    plan["config"].write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="diagnostic_cap_invalid"):
        entry.prepare(plan["config"], plan["output"])


async def test_fake_full_motion_pause_tail_cannot_run_twice(context):
    from tests.robots.fake_lebai import FakeLebaiClient
    entry, plan, kwargs = context
    client = FakeLebaiClient()
    original = client.move_pvat

    async def moving(*args):
        result = await original(*args)
        client.robot_state = "MOVING"
        client.kin_data["actual_joint_speed"] = [.03, 0., 0., 0., 0., 0.]
        return result

    async def pause():
        client.write_calls.append(("pause_move",))
        client.robot_state = "PAUSED"
        client.kin_data["actual_joint_speed"] = [0.] * 6

    client.move_pvat = moving
    client.pause_move = pause
    factory = AsyncMock(return_value=client)
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    assert result["early_stop_triggered"] is True
    assert result["complete"] is True and result["hardware_stop_verified"] is False
    assert result["runtime"] == "SIMULATION"
    methods = [c[0] for c in client.write_calls]
    assert methods.count("pause_move") == 1
    assert set(methods) == {"move_pvat", "pause_move"}
    assert "move_pvat" not in methods[methods.index("pause_move") + 1:]
    events = [json.loads(line) for p in (plan["output"] / "sessions").glob("*/session.jsonl")
              for line in p.read_text(encoding="utf-8").splitlines()]
    assert any(e["kind"] == "stop_observation_summary" for e in events)
    assert all(e["stop_policy"] == "pause_move_only" for e in events if e["kind"] == "stop_diagnostics")
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_awaited_once()


async def test_failed_connection_consumes_attempt(context):
    entry, plan, kwargs = context
    factory = AsyncMock(side_effect=RuntimeError("injected_connection_failure"))
    result = await entry.execute(plan, **kwargs, client_factory=factory)
    assert not result["complete"]
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kwargs, client_factory=factory)
    factory.assert_awaited_once()
