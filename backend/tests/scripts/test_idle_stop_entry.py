from datetime import UTC, datetime, timedelta
import hashlib
import importlib.util
import json
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from app.config import Settings
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE

SOURCE = Path(__file__).resolve().parents[3] / 'scripts/real_robot_idle_stop.py'


@pytest.fixture
def context(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('idle_stop_entry', SOURCE)
    entry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(entry)
    monkeypatch.setattr(entry, 'ROOT', tmp_path)
    monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic-source': 'a'*64})
    data = yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    data['real_robot']['ip'] = '192.0.2.1'
    config = tmp_path / 'control.yaml'
    config.write_text(yaml.safe_dump(data), encoding='utf-8')
    expires = (datetime.now(UTC)+timedelta(minutes=5)).isoformat()
    inputs = entry.prepare(config, tmp_path / '.tmp/once', expires=expires)
    approved = tmp_path / 'preview.json'
    approved.write_text(json.dumps(entry.preview(inputs)), encoding='utf-8')
    kwargs = {'approved': approved, 'expected_hash': entry.sha(approved),
              'expires': expires, 'confirm': entry.CONFIRM}
    return entry, inputs, kwargs


def test_default_cli_only_previews_without_execution(context, monkeypatch, capsys):
    entry, inputs, _kwargs = context
    monkeypatch.setattr(entry, 'execute', lambda *_args, **_kwargs: pytest.fail('must not execute'))
    assert entry.main(['--config', str(inputs['config']), '--output', str(inputs['output'])]) == 0
    value = json.loads(capsys.readouterr().out)
    assert value['allowed_writes'] == ['stop_move'] and value['max_sdk_stop_entries'] == 1
    assert not inputs['output'].exists() and not value['hardware_stop_verified']


@pytest.mark.parametrize('changes', [
    {'confirm': 'yes'}, {'expires': '2000-01-01T00:00:00Z'}, {'expires': '2099-01-01T00:00:00Z'},
    {'expires': 'invalid'}, {'expected_hash': 'b'*64}, {'approved': None},
])
async def test_bad_approval_cannot_create_output_or_connect(context, changes):
    entry, inputs, kwargs = context
    factory = AsyncMock()
    with pytest.raises(ValueError):
        await entry.execute(inputs, **{**kwargs, **changes}, client_factory=factory)
    assert not inputs['output'].exists()
    factory.assert_not_called()


async def test_changed_config_is_rejected_before_output_or_connection(context):
    entry, inputs, kwargs = context
    inputs['config'].write_text(inputs['config'].read_text()+'\n#changed\n')
    factory = AsyncMock()
    with pytest.raises(ValueError, match='inputs_changed'):
        await entry.execute(inputs, **kwargs, client_factory=factory)
    factory.assert_not_called()


async def test_changed_runtime_is_rejected_before_connection(context, monkeypatch):
    entry, inputs, kwargs = context
    monkeypatch.setattr(entry, 'runtime_hashes', lambda: {'synthetic-source': 'b'*64})
    factory = AsyncMock()
    with pytest.raises(ValueError, match='inputs_changed'):
        await entry.execute(inputs, **kwargs, client_factory=factory)
    factory.assert_not_called()


def test_config_change_during_settings_parse_is_not_bound_to_stale_target(context, monkeypatch):
    entry, inputs, _kwargs = context
    original = Settings.load
    def change(path):
        settings = original(path)
        data = yaml.safe_load(path.read_text())
        data['real_robot']['ip'] = '192.0.2.2'
        path.write_text(yaml.safe_dump(data))
        return settings
    monkeypatch.setattr(Settings, 'load', change)
    with pytest.raises(ValueError, match='inputs_changed'):
        entry.prepare(inputs['config'], inputs['output'])


@pytest.mark.parametrize('option', ['--home', '--distance-m', '--gripper', '--ip'])
def test_cli_refuses_motion_and_arbitrary_target_options(context, option):
    entry, inputs, _kwargs = context
    with pytest.raises(SystemExit) as error:
        entry.main(['--config', str(inputs['config']), '--output', str(inputs['output']), option, 'value'])
    assert error.value.code == 2


def test_confirmation_environment_is_restored(context, monkeypatch):
    entry, inputs, _kwargs = context
    monkeypatch.setenv('VR4ARM_REAL_ROBOT_CONFIRM', 'original')
    entry.prepare(inputs['config'], inputs['output'])
    assert entry.os.environ['VR4ARM_REAL_ROBOT_CONFIRM'] == 'original'


def test_approved_deadline_cannot_be_extended_by_changing_only_cli_value(context):
    entry, inputs, kwargs = context
    renewed = (datetime.fromisoformat(kwargs['expires']) + timedelta(minutes=1)).isoformat()
    with pytest.raises(ValueError):
        entry.check(inputs, **{**kwargs, 'expires': renewed})


async def test_complete_fake_entry_records_scope_and_cannot_run_twice(context):
    from tests.robots.fake_lebai import FakeLebaiClient
    entry, inputs, kwargs = context
    factory = AsyncMock(return_value=FakeLebaiClient.idle())
    result = await entry.execute(inputs, **kwargs, client_factory=factory)
    assert result['complete'] and result['sdk_stop_entries'] == 1
    assert (inputs['output']/'result.json').exists()
    events = [json.loads(line) for path in (inputs['output']/'sessions').glob('*/session.jsonl')
              for line in path.read_text().splitlines() if line.strip()]
    header = next(event for event in events if event['kind'] == 'session_started')
    assert header['metadata']['runtime'] == 'SIMULATION'
    assert header['metadata']['hardware_verified'] is False
    with pytest.raises(FileExistsError):
        await entry.execute(inputs, **kwargs, client_factory=factory)
    factory.assert_awaited_once()


async def test_failed_connection_consumes_attempt_and_never_retries(context):
    entry, inputs, kwargs = context
    factory = AsyncMock(side_effect=RuntimeError('synthetic connection error'))
    result = await entry.execute(inputs, **kwargs, client_factory=factory)
    assert not result['complete'] and result['sdk_stop_entries'] == 0
    with pytest.raises(FileExistsError):
        await entry.execute(inputs, **kwargs, client_factory=factory)
    factory.assert_awaited_once()
