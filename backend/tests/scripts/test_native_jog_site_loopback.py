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


def test_delayed_preflight_case_resolves_only_its_fixed_retained_motion_id():
    assert 'preflight_read_delay_20ms' in probe.CASES
    request=dict(jsonrpc='2.0',id=1,method='get_motion_state',params=[dict(id=503)])
    assert probe.parse_request(json.dumps(request))['params']==[dict(id=503)]
    request['params'][0]['id']=504
    with pytest.raises(ValueError):probe.parse_request(json.dumps(request))


def test_mismatch_report_is_persisted_and_cli_fails(tmp_path,monkeypatch):
    result=dict(offline_only=True,real_robot_connections=0,complete=False,
        cases=[dict(case='normal',complete=False,latched_fault='stop_unverified')])
    async def failed_matrix():return result
    monkeypatch.setattr(probe,'matrix',failed_matrix)
    output=tmp_path/'failure.json'
    assert probe.main(['--output',str(output)])==2
    assert json.loads(output.read_text())==result


def test_delayed_read_case_accepts_only_explicit_preflight_only_revocation():
    r=dict(case='preflight_read_delay_20ms',complete=False,local_connection_closed=True,protocol_errors=[],
        speedl_entries=0,stop_entries=0,fk_entries=0,latched_fault=None,rpc_records=[],
        primary_rejection='native_jog_loopback_preflight_only',preflight=dict(ready=True,snapshot_age_ms=60))
    assert probe.matches_expected_case(r)
    bad=copy.deepcopy(r);bad['primary_rejection']='robot_state_stale'
    assert not probe.matches_expected_case(bad)
    bad=copy.deepcopy(r);bad['speedl_entries']=1
    assert not probe.matches_expected_case(bad)
    bad=copy.deepcopy(r);bad['preflight']['snapshot_age_ms']=100
    assert not probe.matches_expected_case(bad)
