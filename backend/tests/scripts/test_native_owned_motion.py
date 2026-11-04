import re

import pytest
from scripts import check_native_owned_motion as probe


@pytest.fixture(scope='module')
def fixture():return probe.build_packets((0.,-.8,1.4,-.6,1.57,0.))


def test_native_owned_trial_cannot_bypass_finite_manifest_or_stop_budget(fixture):
    _,base,packets=fixture;source=probe.program(packets[0].lead.start.q,base,packets)
    assert len(packets)==46 and len(packets)+packets[-1].braking_ticks<=64
    assert set(re.findall(r'lebai:(\w+)\(',source))=={'get_tcp','get_velocity_factor','get_robot_state',
        'get_kin_data','get_signal','get_signals','set_signal','set_signals','move_pvat','stop_move'}
    assert 'Finite TEST whitelist' in source and 'owned_tcp_path' in source and 'owned_rotation_region' in source
    assert source.count('lebai:stop_move()')==1
    assert 'sdk_result.call_void' in source and 'accepted==true' not in source


def test_failed_or_unknown_prepare_never_launches_owner_motion(monkeypatch):
    calls=[]
    def fail(*args):
        calls.append(args);return dict(raw_output='FINITE_PREP_ENTER\t\n',transport_unknown=True,stop_unverified=True)
    monkeypatch.setattr(probe,'transport',fail)
    result=probe.execute({'container_id':'isolated'})
    assert not result['passed'] and result['stop_unverified'] and len(calls)==1


def test_unverified_native_emitter_is_quarantined_before_any_preparation(fixture,monkeypatch):
    _,base,packets=fixture
    with pytest.raises(ValueError,match='native_emitter_return_unverified'):
        probe.program(packets[0].lead.start.q,base,packets,native_emitter=True)
    monkeypatch.setattr(probe,'transport',lambda *a:pytest.fail('no preparation'))
    assert not probe.execute({'container_id':'isolated'},native_emitter=True)['passed']


def test_incomplete_owned_evidence_keeps_unknown_write_even_if_stop_returned(fixture):
    packets=fixture[2]
    raw='OWNED_ENTER\t1\t200\t1\tfollowing\nFINITE_ABORT\ttrue\ttrue\nOWNED_DONE\tfalse\tunknown\t0'
    result=probe.analyze(raw,packets)
    assert not result['passed'] and result['unknown_motion_write'] and result['stop_unverified']


def complete_evidence(packets):
    expected=len(packets)+packets[-1].braking_ticks;rows=[]
    for i in range(1,len(packets)+1):rows.append(f'OWNED_ACK\t{i}\ttrue\tnil\t100')
    for i in range(1,expected+1):
        time=219 if i==1 else 259 if i==2 else 200+(i-2)*80+10
        mode='following' if i<=len(packets) else 'braking'
        rows.extend([f'OWNED_ENTER\t{i}\t{time}\t{i}\t{mode}',f'OWNED_POINT\t{i}\t{time+20}\t{i}\t{mode}'])
    rows.extend(f'OWNED_SAMPLE\t{i*40}\t.00001\t.17\t0' for i in range(140))
    rows.extend(['OWNED_QUIET\t5600\t.00001\t140',f'OWNED_DONE\ttrue\tnil\t{expected}'])
    return '\n'.join(rows)


def test_scheduled_reference_lead_uses_packet_epoch_not_delayed_first_invocation(fixture):
    packets=fixture[2];raw=complete_evidence(packets)
    result=probe.analyze(raw,packets)
    assert result['passed'] and result['planned_epoch_ms']==200
    assert result['max_scheduled_reference_lead_ms']==150
    assert result['min_scheduled_reference_lead_ms']==61
    # An actually early third point exceeds the bound and still rejects.
    assert not probe.analyze(raw.replace('OWNED_ENTER\t3\t290','OWNED_ENTER\t3\t250'),packets)['passed']


def test_default_owner_preview_never_actuates(monkeypatch,tmp_path,capsys):
    monkeypatch.setattr(probe.sys,'argv',['probe','--container','isolated','--output',str(tmp_path/'unused.json')])
    monkeypatch.setattr(probe,'native_admission',lambda _:dict(container_id='isolated'))
    monkeypatch.setattr(probe,'execute',lambda _:pytest.fail('preview cannot actuate'))
    assert probe.main()==0 and '"execute": false' in capsys.readouterr().out
