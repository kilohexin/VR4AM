"""Local protocol/analysis guards; no native SDK imported by these tests."""
import asyncio
import importlib
import json

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve


def probe():
    return importlib.import_module('scripts.sdk_stop_contention_probe')


@pytest.mark.parametrize('stop_delay,read_delay', [(0, .03), (.03, 0)])
async def test_24_reads_are_pending_before_stop_and_replies_are_correlated(stop_delay, read_delay):
    module = probe()
    peer = module.ContentionPeer(read_count=24, delay_s=stop_delay, read_delay_s=read_delay)
    async with serve(peer.handler, '127.0.0.1', 0) as server:
        port = module.owned_port([sock.getsockname() for sock in server.sockets])
        try:
            async with connect(f'ws://127.0.0.1:{port}') as wire:
                for index in range(24):
                    await wire.send(json.dumps({'jsonrpc': '2.0', 'id': index,
                        'method': module.READ_METHODS[index % 6], 'params': [{}]}))
                await asyncio.wait_for(peer.reads_pending.wait(), 1)
                assert len(peer.records) == 24
                assert all('send_started_ns' not in row for row in peer.records)
                await wire.send(json.dumps({'jsonrpc': '2.0', 'id': 24, 'method': 'stop_move', 'params': [{}]}))
                replies = [json.loads(await asyncio.wait_for(wire.recv(), 1)) for _ in range(25)]
                assert {row['id'] for row in replies} == set(range(25))
                assert all('result' in row and 'error' not in row for row in replies)
                assert replies[0]['id'] == (24 if stop_delay == 0 else 0)
            assert not peer.errors
        finally:
            await peer.close()
    assert not peer._tasks and not peer._connections


@pytest.mark.parametrize('count', [-1, 1, True, 25])
def test_only_fixed_bounded_read_counts_are_allowed(count):
    with pytest.raises(ValueError):
        probe().ContentionPeer(read_count=count)


@pytest.mark.parametrize('argument,value', [
    ('delay_s', -.1), ('delay_s', float('nan')), ('delay_s', float('inf')),
    ('delay_s', True), ('delay_s', 10), ('read_delay_s', -.1),
    ('read_delay_s', float('nan')), ('read_delay_s', float('inf')),
    ('read_delay_s', True), ('read_delay_s', 10),
])
def test_simulated_delays_must_be_finite_and_bounded(argument, value):
    with pytest.raises(ValueError):
        probe().ContentionPeer(**{argument: value})


def test_existing_output_is_preserved_before_sdk_initialization(tmp_path, monkeypatch):
    module = probe()
    output = tmp_path / 'result.json'
    output.write_bytes(b'old evidence')
    def forbidden():
        pytest.fail('SDK must not be initialized')
    monkeypatch.setattr(module, '_sdk_identity', forbidden)
    with pytest.raises(SystemExit) as error:
        module.main(['--output', str(output)])
    assert error.value.code == 2 and output.read_bytes() == b'old evidence'


def test_cli_does_not_accept_robot_target(tmp_path):
    with pytest.raises(SystemExit) as error:
        probe().main(['--output', str(tmp_path / 'result.json'), '--ip', '192.0.2.1'])
    assert error.value.code == 2


async def test_mismatched_native_binary_is_refused_before_init(monkeypatch):
    module = probe()
    class SDK:
        def init(self):
            pytest.fail('mismatched native SDK must not initialize')
    monkeypatch.setattr(module, '_sdk_identity', lambda: (SDK(), {'matches_field_binary': False}))
    with pytest.raises(RuntimeError, match='binary'):
        await module.run_matrix(1)


@pytest.mark.parametrize('repeats', [0, True, 6])
async def test_invalid_repeats_are_refused_before_sdk_import(repeats, monkeypatch):
    module = probe()
    def forbidden():
        pytest.fail('SDK must not import')
    monkeypatch.setattr(module, '_sdk_identity', forbidden)
    with pytest.raises(ValueError):
        await module.run_matrix(repeats)


@pytest.mark.parametrize('method', ['get_robot_state', 'get_estop_reason', 'get_kin_data',
                                  'get_tcp', 'get_claw', 'get_running_motion'])
def test_read_validation_rejects_wrong_fixture_results(method):
    with pytest.raises(RuntimeError, match='read'):
        probe().validate_read_results([{'method': method, 'outcome': 'returned', 'value': 'wrong'}])


@pytest.mark.parametrize('row', [
    {'method': 'get_robot_state', 'outcome': 'error', 'value': 'IDLE'},
    {'method': 'stop_move', 'outcome': 'returned', 'value': {}},
    {'method': 'get_robot_state', 'outcome': 'returned'},
])
def test_read_validation_rejects_failed_unknown_or_missing_results(row):
    with pytest.raises(RuntimeError, match='read'):
        probe().validate_read_results([row])
