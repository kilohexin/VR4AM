import asyncio
import json

import pytest

from app.commissioning.readonly_dynamics import collect_dynamics


class Reader:
    def __init__(self, fail=False):
        self.calls = []
        self.fail = fail

    async def get_payload(self):
        self.calls.append('get_payload')
        if self.fail:
            raise RuntimeError('do not copy arbitrary SDK error strings')
        return {'mass': .46, 'cog': {'x': .01, 'y': 0, 'z': .02}, 'extra': 'preserved'}

    async def get_gravity(self):
        self.calls.append('get_gravity')
        return {'x': 0., 'y': 0., 'z': -9.81}


@pytest.fixture
def config(tmp_path):
    path = tmp_path / 'config.yaml'
    path.write_bytes(b'real_robot:\r\n  mode: readonly\r\n  ip: 192.0.2.1\r\n')
    return path


async def test_collects_only_two_reads_preserves_values_and_config(config, tmp_path):
    from app.commissioning import readonly_dynamics as module
    client = Reader()
    connections = []

    async def connect(ip):
        connections.append(ip)
        return client

    original = config.read_bytes()
    output = tmp_path / 'report.json'
    assert await module.collect_dynamics(config, output, connect) == 0
    result = json.loads(output.read_text())
    assert connections == ['192.0.2.1']
    assert client.calls == ['get_payload', 'get_gravity']
    assert result['reads'][0]['raw']['extra'] == 'preserved'
    assert result['reads'][1]['raw']['z'] == -9.81
    assert result['complete'] is True
    assert config.read_bytes() == original
    assert result['config_sha256_before'] == result['config_sha256_after']


@pytest.mark.parametrize('mode', ['control', None])
async def test_rejects_non_readonly_before_connection(config, tmp_path, mode):
    config.write_text(f'real_robot:\n  mode: {mode}\n  ip: 192.0.2.1\n')

    async def forbidden(ip):
        pytest.fail('must not connect')

    with pytest.raises(ValueError, match='readonly'):
        await collect_dynamics(config, tmp_path / 'out.json', forbidden)


async def test_existing_report_never_overwritten_or_connected(config, tmp_path):
    output = tmp_path / 'out.json'
    output.write_text('original')

    async def forbidden(ip):
        pytest.fail('must not connect')

    with pytest.raises(FileExistsError):
        await collect_dynamics(config, output, forbidden)
    assert output.read_text() == 'original'


async def test_first_read_failure_stops_without_retry(config, tmp_path):
    client = Reader(fail=True)

    async def connect(ip):
        return client

    output = tmp_path / 'out.json'
    assert await collect_dynamics(config, output, connect) == 2
    result = json.loads(output.read_text())
    assert client.calls == ['get_payload']
    assert result['complete'] is False
    assert result['reads'][0]['error_type'] == 'RuntimeError'
    assert 'arbitrary SDK' not in output.read_text()


async def test_connection_timeout_records_failure(config, tmp_path):
    async def connect(ip):
        raise asyncio.TimeoutError

    output = tmp_path / 'out.json'
    assert await collect_dynamics(config, output, connect) == 2
    result = json.loads(output.read_text())
    assert result['error_type'] == 'TimeoutError'
    assert result['reads'] == []


async def test_read_timeout_does_not_continue_or_retry(config, tmp_path):
    class SlowReader(Reader):
        async def get_payload(self):
            self.calls.append('get_payload')
            await asyncio.Event().wait()

    client = SlowReader()

    async def connect(ip):
        return client

    output = tmp_path / 'out.json'
    assert await asyncio.wait_for(collect_dynamics(config, output, connect), 4) == 2
    assert client.calls == ['get_payload']
    assert json.loads(output.read_text())['reads'][0]['error_type'] == 'TimeoutError'


async def test_changed_config_is_reported_not_restored(config, tmp_path):
    async def connect(ip):
        config.write_text('changed externally')
        return Reader()

    output = tmp_path / 'out.json'
    assert await collect_dynamics(config, output, connect) == 2
    result = json.loads(output.read_text())
    assert result['config_unchanged'] is False
    assert config.read_text() == 'changed externally'


async def test_non_json_read_is_reported_without_losing_failure_artifact(config, tmp_path):
    class InvalidReader(Reader):
        async def get_payload(self):
            self.calls.append('get_payload')
            return {'mass': float('nan')}

    client = InvalidReader()

    async def connect(ip):
        return client

    output = tmp_path / 'out.json'
    assert await collect_dynamics(config, output, connect) == 2
    result = json.loads(output.read_text())
    assert result['reads'][0]['error_type'] == 'ValueError'
    assert client.calls == ['get_payload']


async def test_late_return_cannot_hide_expired_read_deadline(config, tmp_path, monkeypatch):
    from app.commissioning import readonly_dynamics as module
    now = [0]
    monkeypatch.setattr(module, 'monotonic_ns', lambda: now[0])

    class LateReader(Reader):
        async def get_payload(self):
            value = await super().get_payload()
            now[0] += 3_000_000_000
            return value

    client = LateReader()

    async def connect(ip):
        return client

    output = tmp_path / 'out.json'
    assert await collect_dynamics(config, output, connect) == 2
    result = json.loads(output.read_text())
    assert client.calls == ['get_payload']
    assert result['reads'][0]['outcome'] == 'failed'
    assert result['reads'][0]['error_type'] == 'TimeoutError'
