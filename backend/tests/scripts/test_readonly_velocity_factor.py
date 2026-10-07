import json

import pytest

from app.commissioning import readonly_state as module
from tests.scripts.test_readonly_state import Reader, setup


class FactorReader(Reader):
    def __init__(self, now, value=100):
        super().__init__(now)
        self.value = value

    async def get_velocity_factor(self):
        self.calls.append('get_velocity_factor')
        self.now[0] += 1_000_000
        return self.value


async def collect(setup, reader):
    async def connect(_ip):
        return reader
    code = await module.collect_state(setup[0], setup[1], connect, include_velocity_factor=True)
    return code, json.loads(setup[1].read_text())


async def test_explicit_factor_query_is_once_without_writes_or_expanding_poll_list(setup):
    reader = FactorReader(setup[2])
    before = setup[0].read_bytes()
    code, report = await collect(setup, reader)
    assert code == 0
    assert reader.calls.count('get_velocity_factor') == 1
    assert report['query_allowlist'] == [
        'get_robot_state', 'get_estop_reason', 'get_kin_data', 'get_tcp', 'get_claw',
        'get_payload', 'get_gravity', 'get_velocity_factor',
    ]
    assert next(r for r in report['reads'] if r['method'] == 'get_velocity_factor')['raw'] == 100
    assert setup[0].read_bytes() == before
    assert report['hardware_stop_verified'] is False


@pytest.mark.parametrize('value', [True, '100', 100., None])
async def test_invalid_factor_stops_collection_without_retry_or_generic_fallback(setup, value):
    reader = FactorReader(setup[2], value)
    code, report = await collect(setup, reader)
    assert code == 2
    assert report['complete'] is False
    assert reader.calls.count('get_velocity_factor') == 1
    assert report['reads'][-1]['method'] == 'get_velocity_factor'
    assert report['reads'][-1]['outcome'] == 'failed'
    assert len(reader.calls) == 8


async def test_default_collector_does_not_read_factor(setup):
    reader = FactorReader(setup[2])
    async def connect(_ip):
        return reader
    assert await module.collect_state(setup[0], setup[1], connect) == 0
    assert 'get_velocity_factor' not in reader.calls


def test_factor_option_preview_has_no_connection(setup, monkeypatch, capsys):
    async def forbidden(*args, **kwargs):
        pytest.fail('offline preview must never collect or connect')
    monkeypatch.setattr(module, 'collect_state', forbidden)
    assert module.main(['--config', str(setup[0]), '--include-velocity-factor']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['action'] == 'offline_config_preview'
    assert report['query_allowlist'][-1] == 'get_velocity_factor'
    assert '192.0.2.1' not in str(report)
