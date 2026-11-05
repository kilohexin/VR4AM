import re
from dataclasses import replace

import pytest

from app.commissioning.native_jog_mdh import MdhTool
from app.commissioning.real_cartesian_pvat import build_contract
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel


@pytest.fixture(scope='module')
def contract():
    model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    start=JointReference((0.,-.8,1.4,-.6,1.57,0.));p=model.pose(start.q).p
    return build_contract(model,start,workspace_lower=tuple(x-.01 for x in p),
        workspace_upper=tuple(x+.01 for x in p),config_sha256='a'*64,source_sha256='b'*64,
        tool_report_sha256='c'*64)


def test_native_program_is_finite_and_has_no_start_settings_claw_or_dynamic_code(contract):
    from scripts.real_pvat_native import build_program
    source=build_program(contract)
    assert set(re.findall(r'lebai:(\w+)\(',source))=={
        'get_robot_state','get_kin_data','get_tcp','get_velocity_factor','kinematics_forward','move_pvat','stop_move'}
    assert source.count('lebai:move_pvat(')==source.count('lebai:stop_move(')==1
    assert 'RP_DONE' in source and contract.sha256 in source
    assert 'pvat_cadence' in source and 'pvat_reply_deadline' in source
    assert 'feedback_gap' in source and 'tail_drift' in source and 'arrival_deadline' in source
    assert '/proc/' not in source and 'docker' not in source
    assert 'loadstring' not in source and '__' not in source


@pytest.mark.parametrize('rows',[
    [],[['RP_DONE','true','nil','20']],
    [['RP_ENTER','1','0'],['RP_DONE','false','failed','0']],
    [['RP_RETURN','1','1'],['RP_DONE','true','nil','20']],
])
def test_incomplete_receipts_never_become_acceptance(contract,rows):
    from scripts.real_pvat_native import analyze
    report=analyze(contract,rows)
    assert not report['sampled_passed'] and not report['hardware_verified']
    assert report['motion_unknown'] and report['stop_unverified']


def test_unknown_entry_stays_unknown_even_with_done_and_abort_return(contract):
    from scripts.real_pvat_native import analyze
    report=analyze(contract,[['RP_ENTER','1','0'],['RP_STOP_ENTER','1'],
        ['RP_STOP_RETURN','2'],['RP_DONE','false','failed','0']])
    assert report['motion_unknown'] and report['stop_unverified']


def test_late_return_in_failed_trace_remains_unknown(contract):
    from scripts.real_pvat_native import analyze
    report=analyze(contract,[['RP_ENTER','1','100'],['RP_RETURN','1','160'],
        ['RP_STOP_ENTER','170'],['RP_STOP_RETURN','801'],['RP_DONE','false','pvat_reply_deadline','1']])
    assert report['motion_unknown'] and report['stop_unverified'] and not report['sampled_passed']


def trace(contract):
    def sample(state,t):
        model=MdhReferenceModel(contract.model_tool,contract.joint_lower,contract.joint_upper)
        from app.robots.lebai_codec import pose_to_lebai
        raw=pose_to_lebai(model.pose(state.q))
        return ['RP_SAMPLE',str(t),str(t),'IDLE']+list(map(str,state.q+state.v+state.a+state.q+state.v+state.a)) + [str(raw[k]) for k in ('x','y','z','rx','ry','rz')]
    rows=[sample(contract.origin,0),['RP_ADMIT','1','0']]
    for i,point in enumerate(contract.points,1):
        t=1+(i-1)*79
        observed=sample(point.end,t+2);observed[3]='MOVING'
        rows += [['RP_ENTER',str(i),str(t)],['RP_RETURN',str(i),str(t+1)],observed]
    for t in range(1600,3801,20):
        rows.append(sample(contract.points[-1].end,t))
        if t==1800:rows.append(['RP_ARRIVAL',str(t)])
        if t==3800:rows.append(['RP_TAIL',str(t)])
    return rows+[['RP_DONE','true','nil','20']]


def test_full_sampled_trace_passes_independent_replay_but_never_hardware(contract):
    from scripts.real_pvat_native import analyze
    report=analyze(contract,trace(contract))
    assert report['sampled_passed'] and not report['motion_unknown'] and not report['stop_unverified']
    assert not report['hardware_verified']


@pytest.mark.parametrize('change',[
    lambda rows:[r for r in rows if r[0]!='RP_TAIL'],
    lambda rows:[r for r in rows if not (r[0]=='RP_SAMPLE' and 2000<=int(r[1])<=2120)],
    lambda rows:rows+[['RP_DONE','true','nil','20']],
    lambda rows:[(['RP_RETURN','1',r[2]] if r[0]=='RP_RETURN' else r) for r in rows],
    lambda rows:[(['RP_RETURN',r[1],str(int(r[2])+60)] if r[0]=='RP_RETURN' else r) for r in rows],
    lambda rows:[(['RP_ARRIVAL','1700'] if r[0]=='RP_ARRIVAL' else r) for r in rows],
])
def test_tampered_trace_cannot_pass(contract,change):
    from scripts.real_pvat_native import analyze
    assert not analyze(contract,change(trace(contract)))['sampled_passed']


@pytest.mark.parametrize('field,value',[(40,'100'),(2,'1000000'),(40,'nan')])
def test_initial_feedback_is_validated_before_admission(contract,field,value):
    from scripts.real_pvat_native import analyze
    rows=trace(contract);rows[0][field]=value
    assert not analyze(contract,rows)['sampled_passed']


def test_preloaded_first_reply_candidate_is_explicit_and_independently_replayed(contract):
    from scripts.real_pvat_native import build_program,analyze
    rows=trace(contract)
    rows[3][2]='61';rows[4][1:3]=['63','63']
    candidate=replace(contract,first_reply_deadline_ns=80_000_000)
    assert 'local first_reply_ms=80' in build_program(candidate)
    assert 'local first_reply_ms=60' in build_program(contract)
    assert analyze(candidate,rows)['sampled_passed']
    assert not analyze(contract,rows)['sampled_passed']


def test_direct_api_is_explicit_named_joints_and_true_return_without_fallback(contract):
    from scripts.real_pvat_native import build_program
    direct=replace(contract,pvat_api='lua')
    source=build_program(direct)
    assert direct.sha256!=contract.sha256
    assert 'lebai:move_pvat' not in source
    assert 'move_pvat(joint_pose(item.p),joint_pose(item.v),joint_pose(item.a),.08)==true' in source
    assert 'q["j"..i]=values[i]' in source
