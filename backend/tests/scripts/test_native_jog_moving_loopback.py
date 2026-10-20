"""Owned-loopback CLI and changing theoretical traces; no native SDK import."""
import copy
import importlib
import json

import pytest

from tests.scripts.test_native_jog_stream_loopback import WIRE, request


def probe():
    try:
        return importlib.import_module('scripts.native_jog_moving_loopback')
    except ModuleNotFoundError:
        pytest.fail('moving-feedback owned loopback missing')


@pytest.mark.parametrize('sign', [1,-1])
def test_fixed_positive_and_negative_base_velocity_wire(sign):
    wire = copy.deepcopy(WIRE); wire['speed']['position']['z'] *= sign
    assert probe().parse_request(request(data=wire))['params'] == [wire]


@pytest.mark.parametrize('damage', ['zero_time','rotation','frame','arbitrary_speed','bool_acc','other_write'])
def test_moving_peer_never_accepts_generic_motion(damage):
    wire = copy.deepcopy(WIRE)
    if damage == 'zero_time': wire['param']['time'] = 0
    elif damage == 'rotation': wire['speed']['rotation']['euler_zyx']['x'] = .1
    elif damage == 'frame': wire['frame']['position_kind'] = 'TCP'
    elif damage == 'arbitrary_speed': wire['speed']['position']['z'] = .004
    elif damage == 'bool_acc': wire['param']['acc'] = True
    with pytest.raises(ValueError):
        probe().parse_request(request('movej' if damage == 'other_write' else 'speed_linear', wire))


def test_trace_changes_pose_and_rates_and_reverses_through_zero():
    p = probe(); trace = p.MovingTrace()
    before = trace.history[0]
    trace.step(.003)
    first = trace.history[-1]
    assert first['speed_mps'] == pytest.approx(.00016)
    assert first['q'] != before['q'] and first['tcp_p'] != before['tcp_p']
    assert any(v != 0 for v in first['qd']) and any(v != 0 for v in first['qdd'])
    trace.step(-.003)
    assert trace.history[-1]['speed_mps'] == pytest.approx(0.)
    trace.step(-.003)
    assert trace.history[-1]['speed_mps'] == pytest.approx(-.00016)
    trace.hold()
    assert all(v == 0 for v in trace.history[-1]['qd'])
    assert trace.history[-1]['q'] == trace.history[-2]['q']


@pytest.mark.parametrize('extra', [['--ip','172.16.2.162'], ['--port','3031'], ['--execute'], ['--config','x']])
def test_cli_cannot_select_a_target_or_execute_mode(tmp_path,extra):
    output = tmp_path/'absent.json'
    with pytest.raises(SystemExit): probe().main(['--output',str(output),*extra])
    assert not output.exists()


def test_existing_output_prevents_sdk_initialization(tmp_path,monkeypatch):
    p = probe(); output = tmp_path/'keep.json'; output.write_text('original')
    monkeypatch.setattr(p,'_sdk_identity',lambda:pytest.fail('SDK must not initialize'))
    with pytest.raises(SystemExit): p.main(['--output',str(output)])
    assert output.read_text() == 'original'


@pytest.mark.parametrize('damage', ['extra_motion','cleared_fault','confirmed'])
def test_late_effect_result_cannot_masquerade_as_a_pass(damage):
    p = probe()
    result = dict(case='late_effect',native_entries=1,speed_requests=1,accepted_packets=1,
        stop_requests=1,confirmed=False,latched_fault='stop_incomplete',unknown_motion_methods=[],
        local_connection_closed=True,watchdog_finished=True,peer_errors=[],continuation_rejected=False,
        trace=[{'qd':[.001]*6}])
    p.assert_case_outcome(result)
    if damage == 'extra_motion': result['speed_requests'] = 2
    elif damage == 'cleared_fault': result['latched_fault'] = None
    else: result['confirmed'] = True
    with pytest.raises(RuntimeError): p.assert_case_outcome(result)


@pytest.mark.parametrize('case', ['region_rejected','angular_rejected'])
async def test_record_contains_exactly_the_damaged_feedback_sent_to_sdk(case):
    p = probe(); peer = p.MovingPeer(case)
    peer.active = peer.damage = True; peer.trace.step(.003)
    class Capture:
        async def send(self, raw): self.payload = json.loads(raw)
    connection = Capture(); record = dict(method='get_kin_data',id=3)
    await peer._reply(connection,record)
    assert record['model_feedback'] == connection.payload['result']


@pytest.mark.parametrize('case,reason', [('region_rejected','native_jog_moving_reference_leaves_region'),
    ('angular_rejected','native_jog_moving_not_pure_translation')])
@pytest.mark.parametrize('damage', ['wrong_reason','missing_feedback','wrong_feedback','missing_call'])
def test_rejection_needs_exact_guard_reason_and_a_transmitted_bad_snapshot(case,reason,damage):
    p = probe()
    bad = dict(actual_joint_pose=[0.,-1.,1.0029,0.,1.57,0.],target_joint_pose=[0.,-1.,1.,0.,1.57,0.],
        target_joint_speed=[0.,0.,0.,0.,0.,.00001])
    result = dict(case=case,native_entries=1,speed_requests=1,accepted_packets=1,stop_requests=1,
        confirmed=True,latched_fault=None,unknown_motion_methods=[],local_connection_closed=True,
        watchdog_finished=True,peer_errors=[],continuation_rejected=True,trace=[{'qd':[.001]*6}],
        calls=[dict(outcome='returned',value=7),dict(outcome='error',error=reason,started_ns=100)],
        rpc_records=[dict(method='get_kin_data',received_ns=101,send_completed_ns=102,model_feedback=bad)])
    p.assert_case_outcome(result)
    if damage == 'wrong_reason': result['calls'][1]['error'] = 'native_jog_stream_input_rejected'
    elif damage == 'missing_feedback': result['rpc_records'] = []
    elif damage == 'missing_call': result['calls'] = result['calls'][:1]
    elif case == 'region_rejected': bad['actual_joint_pose'] = bad['target_joint_pose'][:]
    else: bad['target_joint_speed'] = [0.]*6
    with pytest.raises(RuntimeError): p.assert_case_outcome(result)
