from datetime import UTC, datetime, timedelta
import importlib.util
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.commissioning.test_paused_stop_cleanup import paused_client

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/real_robot_paused_stop_cleanup.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('paused_stop_entry', SOURCE)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    monkeypatch.setattr(entry, 'ROOT', tmp_path)
    monkeypatch.setattr(entry.idle, 'ROOT', tmp_path)
    monkeypatch.setattr(entry.idle, 'runtime_hashes', lambda: {})
    monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'a' * 64})
    monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 0)
    data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    data['real_robot']['control']['max_joint_speed_radps'] = .05
    data['real_robot']['control']['max_joint_acceleration_radps2'] = .2
    path = tmp_path / 'control.yaml'
    path.write_text(yaml.safe_dump(data))
    expires = (datetime.now(UTC) + timedelta(minutes=5)).isoformat()
    plan = entry.prepare(path, tmp_path / '.tmp/once', expires=expires)
    approved = tmp_path / 'preview.json'
    approved.write_text(json.dumps(entry.preview(plan)))
    return entry, plan, dict(approved=approved, expected_hash=entry.sha(approved), expires=expires, confirm=entry.CONFIRM)


def test_preview_is_offline_and_limits_writes_to_one_stop(context, monkeypatch, capsys):
    entry, plan, _ = context
    monkeypatch.setattr(entry, 'execute', AsyncMock(side_effect=AssertionError('no execute')))
    assert entry.main(['--config', str(plan['config']), '--output', str(plan['output'])]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['allowed_writes'] == ['stop_move'] and value['max_sdk_stop_entries'] == 1
    assert value['requires_raw_paused'] and not value['automatic_resume']
    assert value['stop_rpc_timeout_ms'] == 200 and value['stable_window_ms'] == 300 and value['verification_timeout_ms'] == 500
    assert not plan['output'].exists()


@pytest.mark.parametrize('change', [dict(confirm='yes'), dict(approved=None), dict(expected_hash='b'*64),
    dict(expires=None), dict(expires='2000-01-01T00:00:00Z'), dict(expires='2099-01-01T00:00:00Z')])
async def test_invalid_approval_cannot_connect_or_reserve(context, change):
    entry, plan, kw = context
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **{**kw, **change}, client_factory=factory)
    factory.assert_not_called()
    assert not plan['output'].exists()


@pytest.mark.parametrize('change', ['config', 'runtime', 'scope', 'rpc'])
async def test_changed_inputs_are_rejected_before_sdk(context, monkeypatch, change):
    entry, plan, kw = context
    if change == 'config':
        plan['config'].write_text(plan['config'].read_text() + '\n# changed')
    elif change == 'runtime':
        monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic': 'b'*64})
    elif change == 'rpc':
        monkeypatch.setattr(entry, 'existing_rpc_connections', lambda: 1)
    else:
        value = json.loads(kw['approved'].read_text())
        value['allowed_writes'] = ['resume_move']
        kw['approved'].write_text(json.dumps(value))
        kw['expected_hash'] = entry.sha(kw['approved'])
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(plan, **kw, client_factory=factory)
    factory.assert_not_called()


async def test_one_stop_with_saved_report_and_no_second_connection(context):
    entry, plan, kw = context
    client = paused_client()
    original = client.stop_move
    async def stop():
        await original()
        client.robot_state = 'IDLE'
    client.stop_move = stop
    factory = AsyncMock(return_value=client)
    result = await entry.execute(plan, **kw, client_factory=factory)
    assert result['complete'] and result['runtime'] == 'SIMULATION'
    assert not result['hardware_stop_verified'] and not result['motion_authorized']
    assert client.write_calls == [('stop_move',)]
    assert json.loads((plan['output'] / 'result.json').read_text()) == result
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kw, client_factory=factory)
    factory.assert_awaited_once()


async def test_connection_failure_consumes_scope_without_robot_cleanup(context):
    entry, plan, kw = context
    factory = AsyncMock(side_effect=RuntimeError('secret_fake_failure'))
    result = await entry.execute(plan, **kw, client_factory=factory)
    assert not result['complete'] and result['sdk_stop_entries'] == 0
    assert 'secret_fake_failure' not in json.dumps(result)
    with pytest.raises(FileExistsError):
        await entry.execute(plan, **kw, client_factory=factory)
    factory.assert_awaited_once()


async def test_cancelled_connection_persists_unknown_without_stop(context):
    import asyncio
    entry, plan, kw = context
    started = asyncio.Event()
    async def pending(ip):
        started.set()
        await asyncio.Event().wait()
    task = asyncio.create_task(entry.execute(plan, **kw, client_factory=pending))
    await started.wait()
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    report = json.loads((plan['output'] / 'result.json').read_text())
    assert not report['complete'] and report['sdk_stop_entries'] == 0
    assert report['connection_diagnostics']['pending_remote_result_unknown']


async def test_utc_expiry_during_hashes_refuses_before_output(context, monkeypatch):
    from types import SimpleNamespace
    entry, plan, kw = context
    current = [datetime.now(UTC)]
    monkeypatch.setattr(entry, 'datetime', SimpleNamespace(now=lambda zone: current[0], fromisoformat=datetime.fromisoformat))
    def hashes():
        current[0] = datetime.fromisoformat(kw['expires'])
        return {'synthetic':'a'*64}
    monkeypatch.setattr(entry, 'runtime_hashes', hashes)
    factory = AsyncMock()
    with pytest.raises(ValueError, match='expired'):
        await entry.execute(plan, **kw, client_factory=factory)
    assert not plan['output'].exists()
    factory.assert_not_called()


async def test_sdk_init_exceeding_original_connection_budget_cannot_connect(context, monkeypatch):
    from types import SimpleNamespace
    from app.commissioning import readonly_kinematics
    entry, plan, kw = context
    clock = [1_000_000_000]
    monkeypatch.setattr(entry, 'monotonic_ns', lambda: clock[0])
    monkeypatch.setattr(readonly_kinematics, 'monotonic_ns', lambda: clock[0])
    def init():
        clock[0] += 3_000_000_000
    sdk = SimpleNamespace(init=init, connect=AsyncMock())
    monkeypatch.setattr(entry.readonly, 'verify_sdk_source', lambda: None)
    original = entry.importlib.import_module
    monkeypatch.setattr(entry.importlib, 'import_module', lambda name: sdk if name=='lebai_sdk' else original(name))
    result = await entry.execute(plan, **kw)
    sdk.connect.assert_not_called()
    assert not result['complete'] and not result['connection_diagnostics']['native_sdk_connect_entered']


async def test_expiry_during_copied_config_hash_ends_before_sdk(context, monkeypatch):
    from types import SimpleNamespace
    entry, plan, kw = context
    current = [datetime.now(UTC)]
    monkeypatch.setattr(entry, 'datetime', SimpleNamespace(now=lambda zone: current[0], fromisoformat=datetime.fromisoformat))
    original = entry.sha
    def hashfile(path):
        value = original(path)
        if Path(path).name == 'config-control.yaml':
            current[0] = datetime.fromisoformat(kw['expires'])
        return value
    monkeypatch.setattr(entry, 'sha', hashfile)
    factory = AsyncMock(return_value=paused_client())
    result = await entry.execute(plan, **kw, client_factory=factory)
    factory.assert_not_called()
    assert not result['complete'] and result['sdk_stop_entries'] == 0
