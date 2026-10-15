"""Strict loopback guards; these tests never import the installed native SDK."""
import asyncio
import copy
import json
from pathlib import Path
import subprocess
import sys

import pytest
from websockets.asyncio.client import connect
from websockets.asyncio.server import serve

from scripts import native_jog_loopback_probe as probe

# Derived from SDK v0.3.13 rpc/motion.rs, serde/posture.rs and proto 3.4.3.
WIRE = {'speed': {'position': {'x':0.,'y':0.,'z':.005},
    'rotation': {'euler_zyx': {'x':0.,'y':0.,'z':0.}}},
    'param': {'acc':.02,'time':.08,'constrained':True},
    'frame': {'position_kind':'CUSTOM','position':{'x':0.,'y':0.,'z':0.},
        'rotation_kind':'CUSTOM','rotation':{'euler_zyx':{'x':0.,'y':0.,'z':0.}}}}


def request(method='speed_linear', payload=None, id=1):
    return json.dumps(dict(jsonrpc='2.0',id=id,method=method,params=[WIRE if payload is None else payload]))


def test_wire_fixture_matches_independent_contract():
    assert probe.parse_request(request())['params'] == [WIRE]


@pytest.mark.parametrize('method',['move_pvat','movej','set_claw','stop_sys','pause_move','speed_line'])
def test_nonwhitelisted_writes_refused(method):
    with pytest.raises(ValueError): probe.parse_request(request(method))


@pytest.mark.parametrize('field,value',[('time',0),('time',None),('time',True),('time',.081),
    ('acc',-.02),('acc',float('nan')),('constrained',False)])
def test_bad_speed_params_refused(field,value):
    wire=copy.deepcopy(WIRE);wire['param'][field]=value
    with pytest.raises(ValueError): probe.parse_request(request(payload=wire))


@pytest.mark.parametrize('change',['rotation','frame','speed','extra','bool'])
def test_bad_cartesian_wire_refused(change):
    wire=copy.deepcopy(WIRE)
    if change=='rotation': wire['speed']['rotation']['euler_zyx']['x']=.1
    elif change=='frame': wire['frame']['position_kind']='TCP'
    elif change=='speed': wire['speed']['position']['z']=.006
    elif change=='bool': wire['speed']['position']['x']=False
    else:wire['junk']=1
    with pytest.raises(ValueError): probe.parse_request(request(payload=wire))


@pytest.mark.parametrize('addresses',[[],[('127.0.0.1',10),('127.0.0.1',11)],
    [('::1',10,0,0)],[('localhost',10)],[('172.16.2.162',3031)],[('127.0.0.1',True)]])
def test_only_single_owned_ipv4_listener_accepted(addresses):
    with pytest.raises(ValueError):probe.owned_port(addresses)


@pytest.mark.parametrize('kind',['file','directory'])
def test_output_conflict_precedes_sdk_initialization(tmp_path,monkeypatch,kind):
    path=tmp_path/'existing'
    if kind=='file':path.write_bytes(b'evidence')
    else:path.mkdir()
    def forbidden(): pytest.fail('SDK import must not happen')
    monkeypatch.setattr(probe,'_sdk_identity',forbidden)
    with pytest.raises(SystemExit) as error:probe.main(['--output',str(path)])
    assert error.value.code==2
    if kind=='file':assert path.read_bytes()==b'evidence'


def test_cli_has_no_target_option(tmp_path):
    path=tmp_path/'never.json'
    r=subprocess.run([sys.executable,'scripts/native_jog_loopback_probe.py','--output',str(path),
        '--ip','172.16.2.162'],capture_output=True,text=True,timeout=10)
    assert r.returncode==2
    assert not path.exists()


def test_wrong_binary_rejected_before_import(tmp_path,monkeypatch):
    binary=tmp_path/'lebai_sdk.pyd';binary.write_bytes(b'wrong')
    monkeypatch.setattr(probe,'SDK_BINARY',binary)
    def forbidden(*args):pytest.fail('must hash before import')
    monkeypatch.setattr(probe.importlib,'import_module',forbidden)
    with pytest.raises(ValueError,match='SDK'):probe._sdk_identity()


async def test_peer_rejects_duplicate_rpc_ids():
    peer=probe.NativeJogPeer('immediate')
    async with serve(peer.handler,'127.0.0.1',0) as server:
        port=probe.owned_port([s.getsockname() for s in server.sockets])
        async with connect(f'ws://127.0.0.1:{port}') as c:
            await c.send(request('get_robot_state',{}))
            assert json.loads(await c.recv())['result']=={'state':'IDLE'}
            await c.send(request('get_robot_state',{}))
            await c.wait_closed()
            assert c.close_code==1008
        assert len(peer.records)==1
        assert peer.errors==['duplicate RPC id']
        await peer.close()
