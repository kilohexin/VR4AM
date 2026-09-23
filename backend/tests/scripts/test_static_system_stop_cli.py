import json
from dataclasses import asdict

import pytest
import yaml

from app.commissioning.static_system_stop import CONFIRM
from app.commissioning.static_system_stop_cli import execute
from app.config import REAL_ROBOT_CONFIRMATION
from tests.robots.real_settings import control_settings


@pytest.fixture
def config(tmp_path, monkeypatch):
    monkeypatch.setenv('VR4ARM_REAL_ROBOT_CONFIRM', REAL_ROBOT_CONFIRMATION)
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump({'backend': 'lebai', 'real_robot': asdict(control_settings())}))
    return path


@pytest.mark.parametrize('failure', ['confirmation', 'readonly', 'existing'])
async def test_cli_invalid_request_does_not_connect(config, tmp_path, failure):
    output = tmp_path / 'run'
    if failure == 'existing': output.mkdir()
    if failure == 'readonly':
        config.write_text(config.read_text().replace('mode: control', 'mode: readonly'))
    async def forbidden(ip):
        pytest.fail('must not connect')
    with pytest.raises((ValueError, FileExistsError)):
        await execute(config, output, 'bad' if failure == 'confirmation' else CONFIRM,
                      client_factory=forbidden)


async def test_connection_failure_records_failure_without_config_write(config, tmp_path):
    original = config.read_bytes()
    async def fail(ip):
        raise RuntimeError('connection failed')
    output = tmp_path / 'run'
    assert await execute(config, output, CONFIRM, client_factory=fail) == 2
    report = json.loads((output / 'result.json').read_text())
    assert not report['complete']
    assert report['config_unchanged']
    assert config.read_bytes() == original


async def test_cli_journals_probe_and_checks_configuration(config, tmp_path, monkeypatch):
    from app.commissioning import static_system_stop_cli as module
    async def connect(ip): return object()
    async def probe(client, settings, *, confirm, emit, before_write):
        before_write()
        emit({'kind': 'test_event'})
        return {'complete': True, 'write_started': True}
    monkeypatch.setattr(module, 'run_probe', probe)
    output = tmp_path / 'run'
    assert await execute(config, output, CONFIRM, client_factory=connect) == 0
    assert json.loads((output / 'result.json').read_text())['config_unchanged']
    assert any(json.loads(line)['kind'] == 'test_event'
               for line in (output / 'events.jsonl').read_text().splitlines())
