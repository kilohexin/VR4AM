"""Protocol/CLI checks; never import the installed native SDK."""
import copy
import importlib
import json

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve


def probe():
    try:
        return importlib.import_module('scripts.native_jog_stream_loopback')
    except ModuleNotFoundError:
        pytest.fail('stream loopback script missing')


WIRE = {'speed': {'position': {'x': 0., 'y': 0., 'z': .003},
    'rotation': {'euler_zyx': {'x': 0., 'y': 0., 'z': 0.}}},
    'param': {'acc': .01, 'time': .04, 'constrained': True},
    'frame': {'position_kind': 'CUSTOM', 'rotation_kind': 'CUSTOM',
        'position': {'x': 0., 'y': 0., 'z': 0.},
        'rotation': {'euler_zyx': {'x': 0., 'y': 0., 'z': 0.}}}}


def request(method='speed_linear', data=None, id=1):
    return json.dumps(dict(jsonrpc='2.0', id=id, method=method, params=[WIRE if data is None else data]))


def test_fixed_speed_wire_and_owned_motion_state_query():
    p = probe()
    assert p.parse_request(request())['params'] == [WIRE]
    assert p.parse_request(request('get_motion_state', {'id': 7}))['params'] == [{'id': 7}]


@pytest.mark.parametrize('method', ['move_pvat', 'movej', 'stop_sys', 'pause_move', 'resume_move', 'set_claw'])
def test_other_writes_cannot_enter_peer(method):
    with pytest.raises(ValueError):
        probe().parse_request(request(method))


@pytest.mark.parametrize('change', ['zero_time', 'large_time', 'bool_acc', 'rot', 'frame', 'nonfinite'])
def test_speed_parameters_cannot_be_relaxed(change):
    value = copy.deepcopy(WIRE)
    if change == 'zero_time':
        value['param']['time'] = 0
    elif change == 'large_time':
        value['param']['time'] = .6
    elif change == 'bool_acc':
        value['param']['acc'] = True
    elif change == 'rot':
        value['speed']['rotation']['euler_zyx']['z'] = .01
    elif change == 'frame':
        value['frame']['position_kind'] = 'TCP'
    else:
        value['speed']['position']['z'] = float('nan')
    with pytest.raises(ValueError):
        probe().parse_request(request(data=value))


@pytest.mark.parametrize('extra', [['--ip', '192.168.10.20'], ['--port', '3031'], ['--execute-once'], ['--config', 'x']])
def test_cli_rejects_target_or_execute_parameters_before_output(tmp_path, extra):
    output = tmp_path/'never.json'
    with pytest.raises(SystemExit) as error:
        probe().main(['--output', str(output), *extra])
    assert error.value.code == 2 and not output.exists()


def test_existing_output_precedes_sdk_identity_check(tmp_path, monkeypatch):
    p = probe()
    path = tmp_path/'existing.json'
    path.write_bytes(b'original')
    def forbidden():
        pytest.fail('SDK initialization attempted')
    monkeypatch.setattr(p, '_sdk_identity', forbidden)
    with pytest.raises(SystemExit):
        p.main(['--output', str(path)])
    assert path.read_bytes() == b'original'


def test_sdk_identity_mismatch_precedes_import(tmp_path, monkeypatch):
    p = probe()
    binary = tmp_path/'wrong.pyd'
    binary.write_bytes(b'wrong')
    monkeypatch.setattr(p, 'SDK_BINARY', binary)
    with pytest.raises(ValueError, match='SDK'):
        p._sdk_identity()


@pytest.mark.parametrize('addresses', [[], [('localhost', 42)], [('172.16.2.162', 3031)], [('127.0.0.1', True)]])
def test_only_owned_ipv4_listener_is_accepted(addresses):
    with pytest.raises(ValueError):
        probe().owned_port(addresses)


async def test_peer_limits_write_count_and_rpc_ids():
    p = probe()
    peer = p.StreamPeer('normal')
    async with serve(peer.handler, '127.0.0.1', 0) as server:
        port = p.owned_port([sock.getsockname() for sock in server.sockets])
        async with connect(f'ws://127.0.0.1:{port}') as client:
            for seq in range(1, 11):
                await client.send(request(id=seq))
                assert json.loads(await client.recv())['result']['id'] == seq+6
            await client.send(request(id=11))
            await client.wait_closed()
            assert client.close_code == 1008
        assert len(peer.records) == 10 and peer.errors
        await peer.close()


@pytest.mark.parametrize('damage', ['new_write', 'fault_cleared', 'wrong_confirmation'])
def test_late_effect_cannot_be_reported_as_success(damage):
    p = probe()
    result = dict(case='late_effect', accepted_packets=1, native_entries=1, speed_requests=1,
        stop_requests=1, confirmed=False, latched_fault='stop_incomplete',
        unknown_motion_methods=[], local_connection_closed=True, peer_errors=[])
    p.assert_case_outcome(result)
    if damage == 'new_write':
        result['speed_requests'] = 2
    elif damage == 'fault_cleared':
        result['latched_fault'] = None
    else:
        result['confirmed'] = True
    with pytest.raises(RuntimeError):
        p.assert_case_outcome(result)
