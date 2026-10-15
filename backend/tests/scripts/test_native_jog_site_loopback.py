import copy
import json
import pytest
from scripts import native_jog_site_loopback as probe


@pytest.mark.parametrize('method',['move_pvat','move_joint','stop_sys','pause_move','set_claw'])
def test_owned_peer_denies_other_commands(method):
    with pytest.raises(ValueError):probe.parse_request(json.dumps(dict(jsonrpc='2.0',id=1,method=method,params=[{}])))


def test_fixed_wire_and_fk_inputs():
    wire=probe.speed_request()
    assert probe.parse_request(json.dumps(dict(jsonrpc='2.0',id=1,method='speed_linear',params=[wire])))
    bad=copy.deepcopy(wire);bad['param']['time']=.09
    with pytest.raises(ValueError):probe.parse_request(json.dumps(dict(jsonrpc='2.0',id=1,method='speed_linear',params=[bad])))


def test_output_exclusive_before_sdk_init(tmp_path,monkeypatch):
    output=tmp_path/'existing.json';output.write_text('evidence')
    monkeypatch.setattr(probe,'_sdk_identity',lambda:pytest.fail('must not import SDK'))
    with pytest.raises(SystemExit):probe.main(['--output',str(output)])
    assert output.read_text()=='evidence'


def test_no_target_parameter(tmp_path):
    with pytest.raises(SystemExit):probe.main(['--output',str(tmp_path/'x.json'),'--ip','192.168.1.1'])
