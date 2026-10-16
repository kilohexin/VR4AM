"""No SDK is imported by these synthetic loopback guard tests."""
import asyncio
import json
import pytest
from scripts import independent_stop_loopback as probe

@pytest.mark.parametrize('host',['localhost','172.16.2.162','::1'])
async def test_no_external_target_or_unowned_port(host):
    class Forbidden:
        def connect(self,*args):pytest.fail('SDK connect must not happen')
    with pytest.raises(ValueError):await probe.connect_owned(Forbidden(),host,10,set())

async def test_loopback_address_alone_does_not_approve_arbitrary_port():
    with pytest.raises(ValueError):await probe.connect_owned(None,'127.0.0.1',3030,set())

@pytest.mark.parametrize('method',['stop_sys','pause_move','movej','get_robot_state'])
def test_peer_accepts_only_the_two_fixed_probe_commands(method):
    with pytest.raises(ValueError):probe.parse_request(json.dumps(dict(jsonrpc='2.0',id=1,method=method,params=[{}])))

def test_output_conflict_precedes_any_sdk_import(tmp_path,monkeypatch):
    output=tmp_path/'existing.json';output.write_text('keep')
    def forbidden():pytest.fail('SDK identity must not run')
    monkeypatch.setattr(probe,'_sdk_identity',forbidden)
    with pytest.raises(SystemExit):probe.main(['--output',str(output)])
    assert output.read_text()=='keep'

def test_cli_exposes_no_robot_target(tmp_path):
    with pytest.raises(SystemExit):probe.main(['--output',str(tmp_path/'never.json'),'--ip','172.16.2.162'])
    assert not (tmp_path/'never.json').exists()
