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


def phase_result(case):
    return dict(case=case,complete=True,local_connection_closed=True,protocol_errors=[],
        speedl_entries=1,stop_entries=1,stop_confirmed=True,latched_fault=None,
        speedl_remote_result_unknown=False,speedl_started_ns=80_000_000,
        speedl_call=dict(outcome='returned',native_completed_ns=115_000_000),
        native_reads=[dict(method='kinematics_forward',outcome='returned',completed_ns=0)],
        rpc_records=[dict(method='speed_linear',send_completed_ns=115_000_000),
            dict(method='stop_move',send_completed_ns=170_000_000)])


def test_fixed_phase_cases_are_in_native_matrix():
    assert {'feedback_ages_reply_35ms','revoked_during_speedl','late_effect_after_stop'}<=set(probe.CASES)


def test_aged_feedback_case_requires_actual_crossing_and_known_return():
    r=phase_result('feedback_ages_reply_35ms');assert probe.matches_expected_case(r)
    for change in ('not_crossed','stale_at_entry','unknown','failed'):
        bad=copy.deepcopy(r)
        if change=='not_crossed':bad['speedl_call']['native_completed_ns']=99_999_999
        elif change=='stale_at_entry':bad['speedl_started_ns']=100_000_000
        elif change=='unknown':bad['speedl_remote_result_unknown']=True
        else:bad['complete']=False
        assert not probe.matches_expected_case(bad)


def test_pending_revocation_case_requires_specific_revocation_and_unknown():
    r=phase_result('revoked_during_speedl');r.update(complete=False,stop_confirmed=False,
        latched_fault='stop_unverified',speedl_remote_result_unknown=True,
        authority_rejection='native_jog_loopback_revoked');r['speedl_call']['outcome']='unknown'
    r['rpc_records'][0]['send_completed_ns']=220_000_000
    assert probe.matches_expected_case(r)
    for key,value in (('authority_rejection','native_jog_feedback_fk_expired_before_entry'),
            ('speedl_remote_result_unknown',False),('complete',True)):
        bad=copy.deepcopy(r);bad[key]=value;assert not probe.matches_expected_case(bad)
    bad=copy.deepcopy(r);bad['rpc_records'][0].pop('send_completed_ns')
    assert not probe.matches_expected_case(bad)


def test_late_effect_case_requires_stop_before_late_moving_and_failure():
    r=phase_result('late_effect_after_stop');r.update(complete=False,stop_confirmed=False,
        latched_fault='stop_unverified',speedl_remote_result_unknown=True,
        primary_rejection='stop_incomplete',late_effect_started_ns=200_000_000,
        original_stop_confirmed=False)
    r['speedl_call']['outcome']='unknown'
    r['rpc_records'][0]['send_completed_ns']=220_000_000
    r['rpc_records'].append(dict(method='get_robot_state',received_ns=210_000_000,synthetic_state='MOVING'))
    assert probe.matches_expected_case(r)
    bad=copy.deepcopy(r);bad['late_effect_started_ns']=160_000_000
    assert not probe.matches_expected_case(bad)
    bad=copy.deepcopy(r);bad['rpc_records'][-1]['synthetic_state']='IDLE'
    assert not probe.matches_expected_case(bad)
    bad=copy.deepcopy(r);bad['original_stop_confirmed']=True
    assert not probe.matches_expected_case(bad)
    bad=copy.deepcopy(r);bad['rpc_records'][0]['send_completed_ns']=160_000_000
    assert not probe.matches_expected_case(bad)


def test_delayed_stop_cannot_pass_when_speed_also_failed():
    r=phase_result('stop_late_250ms');r.update(complete=False,stop_confirmed=False,
        latched_fault='stop_unverified',primary_rejection='sdk_timeout:stop_move')
    assert probe.matches_expected_case(r)
    r['speedl_call']['outcome']='unknown';r['speedl_remote_result_unknown']=True
    assert not probe.matches_expected_case(r)
