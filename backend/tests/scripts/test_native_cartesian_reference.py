import re
import subprocess

import pytest
from scripts import check_native_cartesian_reference as probe


def trace():
    return '\n'.join([f'REFERENCE_FK_POINT\t{i}\t2\t0.0000001\t0.000001' for i in range(1,4)]
        +['REFERENCE_FK_DONE\ttrue\tnil\t3'])


def test_native_source_can_only_read_tool_joint_state_and_calculate_fk():
    cases,brakes=probe.build_cases();source=probe.program(cases)
    assert 0<len(cases)<=256 and brakes==5
    assert set(re.findall(r'lebai:(\w+)\(',source))=={'get_tcp','get_kin_data','kinematics_forward'}
    assert 'changed_joint_state' in source and 'nonzero_tool' in source


@pytest.mark.parametrize('change',[
    lambda s:s.replace('DONE\ttrue','DONE\tfalse'),
    lambda s:s.replace('POINT\t2','POINT\t1'),
    lambda s:s.replace('\t2\t0.0000001','\t80\t0.0000001'),
    lambda s:s.replace('0.0000001','nan'),
    lambda s:s.replace('0.0000001','0.001'),
    lambda s:s.replace('REFERENCE_FK_POINT\t3','MISSING\t3'),
])
def test_bad_or_incomplete_native_evidence_cannot_pass(change):
    raw=change(trace())
    assert not probe.analyze(raw,3)['passed']


def test_success_reports_only_fk_agreement_not_motion_interpolation():
    result=probe.analyze(trace(),3)
    assert result['passed'] and result['samples']==3
    assert result['motion_calls']==result['signal_writes']==0
    assert not result['native_interpolation_verified']


def test_execution_rechecks_container_identity_before_any_calculation_transport(monkeypatch):
    admitted=dict(image_id=probe.IMAGE_ID,container_id='isolated')
    monkeypatch.setattr(probe,'native_admission',lambda _:dict(admitted,container_id='changed'))
    monkeypatch.setattr(probe.subprocess,'run',lambda *a,**kw:pytest.fail('no transport'))
    with pytest.raises(ValueError,match='isolation_changed'):
        probe.execute(admitted,[dict(q=(0.,)*6,p=(0.,)*3,r=(0.,)*9)])


@pytest.mark.parametrize('damage',[None,'extra_network','port','mount','image','device','command'])
def test_pinned_digest_creation_is_allowed_but_isolation_changes_are_rejected(damage):
    row=dict(Id='isolated',Image=probe.IMAGE_ID,Config=dict(Image=probe.IMAGE_ID,Cmd=['/data/startup.sh']),
        State=dict(Running=True),Mounts=[],HostConfig=dict(NetworkMode='none',Privileged=False,Devices=[],PortBindings={}),
        NetworkSettings=dict(Ports={},Networks={'none':{}}))
    if damage=='extra_network':row['NetworkSettings']['Networks']['bridge']={}
    if damage=='port':row['NetworkSettings']['Ports']['3010/tcp']=[{}]
    if damage=='mount':row['Mounts']=[{}]
    if damage=='image':row['Image']='other-image'
    if damage=='device':row['HostConfig']['Devices']=[{}]
    if damage=='command':row['Config']['Cmd']=['other']
    if damage:
        with pytest.raises(ValueError,match='isolation_changed'):probe.validate_native_isolated(row)
    else:assert probe.validate_native_isolated(row)['image_id']==probe.IMAGE_ID


def test_host_override_is_rejected_before_any_docker_call(monkeypatch):
    monkeypatch.setenv('DOCKER_HOST','tcp://remote:2375')
    monkeypatch.setattr(probe.subprocess,'run',lambda *a,**kw:pytest.fail('no remote inspection'))
    with pytest.raises(ValueError,match='nonlocal'):probe.native_admission('isolated')


def test_timeout_preserves_partial_result_and_never_retries(monkeypatch):
    admitted=dict(image_id=probe.IMAGE_ID,container_id='isolated');calls=[]
    monkeypatch.setattr(probe,'native_admission',lambda _:admitted)
    def fail(*a,**kw):
        calls.append(a);raise subprocess.TimeoutExpired('docker',25,output=b'REFERENCE_FK_POINT\t1\t2\t0\t0\n')
    monkeypatch.setattr(probe.subprocess,'run',fail)
    result=probe.execute(admitted,[dict(q=(0.,)*6,p=(0.,)*3,r=(0.,)*9)])
    assert len(calls)==1 and result['read_unknown'] and not result['passed']
    assert 'REFERENCE_FK_POINT' in result['raw_output']
