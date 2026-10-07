import asyncio
import json

import pytest

from app.commissioning import readonly_state as module


EXPECTED_READS = ('get_robot_state', 'get_estop_reason', 'get_kin_data',
                  'get_tcp', 'get_claw', 'get_payload', 'get_gravity')


@pytest.fixture
def setup(tmp_path, monkeypatch):
    config = tmp_path / 'readonly.yaml'
    config.write_text('real_robot:\n  mode: readonly\n  ip: 192.0.2.1\n')
    now = [0]
    monkeypatch.setattr(module, 'monotonic_ns', lambda: now[0])

    async def pause(seconds):
        now[0] += int(seconds * 1e9)

    monkeypatch.setattr(module, 'sleep', pause)
    return config, tmp_path / 'report.json', now


class Reader:
    def __init__(self, now, fail=None, state='IDLE'):
        self.now, self.fail, self.state = now, fail, state
        self.calls = []

    def __getattr__(self, name):
        assert name in EXPECTED_READS, f'forbidden SDK operation: {name}'

        async def read():
            self.calls.append(name)
            self.now[0] += 1_000_000
            if name == self.fail:
                raise RuntimeError('sensitive SDK error text')
            if name == 'get_robot_state':
                return self.state
            if name == 'get_estop_reason':
                return 0
            if name == 'get_kin_data':
                return {'actual_joint_pose': [0.] * 6,
                        'actual_joint_speed': [0.] * 6,
                        'actual_tcp_pose': dict(x=.3, y=0., z=.4, rx=0., ry=0., rz=0.),
                        'extra': 'retain raw fields'}
            return {'value': .04}

        return read


async def run(setup, client):
    config, output, _ = setup
    connections = []

    async def connect(ip):
        connections.append(ip)
        return client

    code = await module.collect_state(config, output, connect)
    return code, json.loads(output.read_text()), connections


async def test_fixed_read_allowlist_one_connection_bounded_window_and_no_config_change(setup):
    config, output, now = setup
    before = config.read_bytes()
    client = Reader(now)
    code, report, connections = await run(setup, client)
    assert code == 0 and report['complete']
    assert connections == ['192.0.2.1']
    assert module.READ_METHODS == EXPECTED_READS
    assert client.calls[:7] == list(EXPECTED_READS)
    assert client.calls.count('get_payload') == client.calls.count('get_gravity') == 1
    assert client.calls.count('get_robot_state') > 1
    assert all(r['started_ns'] < report['window_deadline_ns'] for r in report['reads'])
    assert report['reads'][2]['raw']['extra'] == 'retain raw fields'
    assert report['hardware_stop_verified'] is False
    assert report['connection_release'] == 'process_exit_required_not_verified'
    assert report['config_unchanged'] and config.read_bytes() == before
    assert '192.0.2.1' not in output.read_text()


@pytest.mark.parametrize('mode', ['control', 'null', 'wrong'])
async def test_reject_mode_before_connection(setup, mode):
    config, output, _ = setup
    config.write_text(f'real_robot:\n  mode: {mode}\n  ip: 192.0.2.1\n')

    async def forbidden(ip):
        pytest.fail('must not connect')

    with pytest.raises(ValueError, match='readonly'):
        await module.collect_state(config, output, forbidden)
    assert not output.exists()


async def test_no_overwrite_and_no_connection(setup):
    config, output, _ = setup
    output.write_text('original')

    async def forbidden(ip):
        pytest.fail('must not connect')

    with pytest.raises(FileExistsError):
        await module.collect_state(config, output, forbidden)
    assert output.read_text() == 'original'


@pytest.mark.parametrize('state', ['MOVING', 'STOP', 'unknown', True])
async def test_non_idle_ends_after_first_read_without_stop_or_retry(setup, state):
    client = Reader(setup[2], state=state)
    code, report, _ = await run(setup, client)
    assert code == 2 and not report['complete']
    assert client.calls == ['get_robot_state']
    assert report['reads'][0]['raw'] == state


async def test_read_failure_saved_no_retry_no_sensitive_exception(setup):
    client = Reader(setup[2], fail='get_kin_data')
    code, report, _ = await run(setup, client)
    assert code == 2
    assert client.calls == list(module.READ_METHODS[:3])
    assert report['reads'][-1]['outcome'] == 'failed'
    assert 'sensitive' not in setup[1].read_text()


async def test_late_sdk_return_ends_window_and_keeps_raw_evidence(setup):
    class LateReader(Reader):
        async def get_robot_state(self):
            self.calls.append('get_robot_state')
            self.now[0] += 6_000_000_000
            return 'IDLE'

    client = LateReader(setup[2])
    code, report, _ = await run(setup, client)
    assert code == 2
    assert report['reads'][0]['raw'] == 'IDLE'
    assert report['reads'][0]['error_type'] == 'TimeoutError'
    assert client.calls == ['get_robot_state']


async def test_changed_config_during_connect_ends_before_reads(setup):
    config, output, now = setup
    client = Reader(now)

    async def connect(ip):
        config.write_text('externally changed')
        return client

    assert await module.collect_state(config, output, connect) == 2
    report = json.loads(output.read_text())
    assert not report['config_unchanged']
    assert client.calls == []
    assert config.read_text() == 'externally changed'


async def test_cancellation_writes_partial_report_then_propagates(setup):
    class CancelledReader(Reader):
        async def get_robot_state(self):
            self.calls.append('get_robot_state')
            raise asyncio.CancelledError

    client = CancelledReader(setup[2])
    with pytest.raises(asyncio.CancelledError):
        await run(setup, client)
    report = json.loads(setup[1].read_text())
    assert not report['complete'] and report['cancelled']
    assert report['reads'][0]['outcome'] == 'cancelled'


def test_cli_defaults_to_redacted_offline_preview(setup, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail('preview must not collect or connect')

    monkeypatch.setattr(module, 'collect_state', forbidden)
    assert module.main(['--config', str(setup[0])]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['action'] == 'offline_config_preview'
    assert '192.0.2.1' not in json.dumps(report)
    assert not setup[1].exists()


@pytest.mark.parametrize('raw', [None, {}, {'actual_joint_pose': [0.] * 5}])
async def test_missing_or_invalid_kinematics_end_before_tool_and_claw_queries(setup, raw):
    class InvalidReader(Reader):
        async def get_kin_data(self):
            self.calls.append('get_kin_data')
            return raw

    client = InvalidReader(setup[2])
    code, report, _ = await run(setup, client)
    assert code == 2 and not report['complete']
    assert client.calls == list(module.READ_METHODS[:3])


async def test_non_finite_json_ends_without_reporting_collection_success(setup):
    class InvalidReader(Reader):
        async def get_claw(self):
            self.calls.append('get_claw')
            return {'amplitude': float('nan')}

    client = InvalidReader(setup[2])
    code, report, _ = await run(setup, client)
    assert code == 2
    assert report['reads'][-1]['error_type'] == 'ValueError'
    assert 'NaN' not in setup[1].read_text()


async def test_estop_ends_after_read_preserves_raw_reason(setup):
    class EstoppedReader(Reader):
        async def get_estop_reason(self):
            self.calls.append('get_estop_reason')
            return 'hard_estop'

    client = EstoppedReader(setup[2])
    code, report, _ = await run(setup, client)
    assert code == 2
    assert client.calls == list(module.READ_METHODS[:2])
    assert report['reads'][-1]['raw'] == 'hard_estop'


async def test_state_changes_to_moving_during_poll_ends_without_control_calls(setup):
    class MovingReader(Reader):
        async def get_robot_state(self):
            self.calls.append('get_robot_state')
            return 'IDLE' if self.calls.count('get_robot_state') == 1 else 'MOVING'

    client = MovingReader(setup[2])
    code, report, _ = await run(setup, client)
    assert code == 2
    assert client.calls == list(module.READ_METHODS) + ['get_robot_state']
    assert report['reads'][-1]['raw'] == 'MOVING'


async def test_real_await_timeout_writes_failure_and_never_continues(setup):
    class SlowReader(Reader):
        async def get_robot_state(self):
            self.calls.append('get_robot_state')
            await asyncio.Event().wait()

    client = SlowReader(setup[2])
    code, report, _ = await asyncio.wait_for(run(setup, client), 3)
    assert code == 2
    assert client.calls == ['get_robot_state']
    assert report['reads'][-1]['error_type'] == 'TimeoutError'


@pytest.mark.parametrize('late', [False, True])
async def test_failed_or_late_connection_has_no_reads_or_retry(setup, late):
    config, output, now = setup
    calls = []
    client = Reader(now)

    async def connect(ip):
        calls.append(ip)
        if late:
            now[0] += 6_000_000_000
            return client
        raise TimeoutError('sensitive connection details')

    assert await module.collect_state(config, output, connect) == 2
    report = json.loads(output.read_text())
    assert len(calls) == 1 and client.calls == report['reads'] == []
    assert report['error_type'] == 'TimeoutError'
    assert 'sensitive' not in output.read_text()


def test_execution_requires_output_before_collection(setup, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('must not collect')

    monkeypatch.setattr(module, 'collect_state', forbidden)
    with pytest.raises(SystemExit) as error:
        module.main(['--config', str(setup[0]), '--execute-readonly'])
    assert error.value.code == 2
