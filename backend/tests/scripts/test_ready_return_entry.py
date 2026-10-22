"""New scope, injected Fake only, no SDK initialization or robot connection."""
import importlib.util
import json
import subprocess
import sys
from dataclasses import asdict
from datetime import UTC,datetime,timedelta
from pathlib import Path
from unittest.mock import AsyncMock

import pytest
import yaml

from tests.commissioning.test_ready_return import fixture as runner_fixture
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE

SOURCE=Path(__file__).resolve().parents[3]/'scripts/real_robot_ready_return.py'


def test_default_preview_entry_exists():
    assert SOURCE.exists(),'default-preview ready-return CLI missing'


@pytest.fixture
def context(tmp_path,monkeypatch):
    assert SOURCE.exists(),'default-preview ready-return CLI missing'
    spec=importlib.util.spec_from_file_location('ready_return_entry_test',SOURCE)
    e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
    monkeypatch.setattr(e,'ROOT',tmp_path)
    code=tmp_path/'code.py';code.write_text('synthetic')
    monkeypatch.setattr(e,'runtime_hashes',lambda:{'code.py':e.sha(code)})
    monkeypatch.setattr(e,'existing_rpc_connections',lambda:0)
    monkeypatch.setattr(e,'load_sdk',lambda:pytest.fail('SDK forbidden'))
    s,start,c,clock,recorder=runner_fixture()
    config=tmp_path/'config.yaml';data=yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'))
    data['real_robot']['teleop_ready_q']=list(s.teleop_ready_q)
    data['real_robot']['control'].update(max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2)
    config.write_text(yaml.safe_dump(data))
    selected=tmp_path/'selected.json';selected.write_text(json.dumps(dict(
        selection='operator_selected_current_working_ready_pose',q_rad=list(s.teleop_ready_q),
        raw_state='IDLE',tool_tcp_setting=asdict(s.expected_tcp),source=dict(target_ip=s.ip))))
    state=dict(raw_robot_state='IDLE',motion_state='FINISHED',actual_q=list(start),target_q=list(start),
        actual_qd=[0.]*6,target_qd=[0.]*6,actual_qdd=[0.]*6,actual_tcp=dict(p=[.3,0,.4],q=[0,0,0,1]))
    session=tmp_path/'session.jsonl';session.write_text('\n'.join(map(json.dumps,[
        dict(kind='stop_observation_sample',state=state,sequence=1),
        dict(kind='paused_stop_cleanup_result',runtime='LIVE_STATIC_DIAGNOSTIC',complete=True,
             sdk_stop_entries=1,read_remote_result_unknown=False,latched_fault=None)])))
    expires=(datetime.now(UTC)+timedelta(minutes=5)).isoformat()
    plan=e.prepare(config,session,selected,tmp_path/'.tmp/result',expires=expires)
    approved=tmp_path/'preview.json';approved.write_text(json.dumps(e.preview(plan)))
    kw=dict(approved=approved,expected_hash=e.sha(approved),expires=expires,confirm=e.CONFIRM)
    return e,plan,kw,c,clock


def test_preview_has_fixed_scope_and_no_execution(context,monkeypatch,capsys):
    e,p,kw,c,clock=context
    monkeypatch.setattr(e,'execute',AsyncMock(side_effect=AssertionError('preview executed')))
    assert e.main(['--config',str(p['config']),'--start-session',str(p['start_session']),
        '--ready-pose',str(p['ready_pose']),'--output',str(p['output'])])==0
    v=json.loads(capsys.readouterr().out)
    assert v['allowed_writes']==['movej','stop_move'] and v['max_movej_entries']==1
    assert v['target_q_rad']==list(p['settings'].teleop_ready_q)
    assert v['max_fk_calls']==v['max_ik_calls']==0
    assert v['options']['position_tolerance_rad']==pytest.approx(__import__('math').radians(.1))
    assert v['start_tolerance_rad']==v['options']['position_tolerance_rad']
    assert v['snapshot_read_policy']=='owned_six_fields_parallel_except_stop'
    assert v['stop_rpc_timeout_ms']==200 and v['stop_stable_ms']==300 and v['stop_verification_ms']==500
    assert not v['motion_authorized'] and not v['automatic_retry'] and not p['output'].exists()


@pytest.mark.parametrize('kind',['confirm','expiry','hash','preview','config','start_session','ready_pose','runtime','rpc'])
async def test_invalid_scope_never_initializes_or_connects(context,monkeypatch,kind):
    e,p,kw,c,clock=context;factory=AsyncMock(return_value=c)
    if kind=='confirm':kw['confirm']='I_APPROVE_ONE_IDLE_STOP_MOVE_BASELINE'
    elif kind=='expiry':kw['expires']='2000-01-01T00:00:00Z'
    elif kind=='hash':kw['expected_hash']='b'*64
    elif kind=='preview':kw['approved'].write_text('{}')
    elif kind=='runtime':(e.ROOT/'code.py').write_text('changed')
    elif kind=='rpc':monkeypatch.setattr(e,'existing_rpc_connections',lambda:1)
    else:p[kind].write_bytes(p[kind].read_bytes()+b'\n')
    with pytest.raises(ValueError):await e.execute(p,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    factory.assert_not_called();assert not p['output'].exists()


async def test_injected_execution_seals_evidence_and_is_consumed(context):
    e,p,kw,c,clock=context;factory=AsyncMock(return_value=c)
    r=await e.execute(p,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    assert r['complete'],r
    assert r['runtime']=='SIMULATION' and r['movej_entries']==1 and r['stop_entries']==0
    assert json.loads((p['output']/'result.json').read_text())==r
    with pytest.raises(FileExistsError):await e.execute(p,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    assert factory.await_count==1


@pytest.mark.parametrize('kind',['target','tool','address','not_idle','pending','unknown'])
def test_reference_damage_is_rejected_before_admission(context,kind):
    e,p,kw,c,clock=context
    if kind in ('target','tool','address'):
        v=json.loads(p['ready_pose'].read_text())
        if kind=='target':v['q_rad'][0]+=.01
        elif kind=='tool':v['tool_tcp_setting']['z']+=.001
        else:v['source']['target_ip']='192.168.42.1'
        p['ready_pose'].write_text(json.dumps(v))
    else:
        rows=[json.loads(x) for x in p['start_session'].read_text().splitlines()]
        if kind=='not_idle':rows[0]['state']['raw_robot_state']='PAUSED'
        elif kind=='pending':rows[0]['state']['motion_state']='RUNNING'
        else:rows[1]['read_remote_result_unknown']=True
        p['start_session'].write_text('\n'.join(map(json.dumps,rows)))
    with pytest.raises(ValueError):e.prepare(p['config'],p['start_session'],p['ready_pose'],p['output'])


def readonly_reference(context):
    """A completed, sealed read-only survey; it is not a stop confirmation."""
    e,p,kw,c,clock=context
    folder=e.ROOT/'survey';output=folder/'execution';output.mkdir(parents=True)
    q=list(p['start_q']);tcp=dict(p=[.3,0,.4],q=[0,0,0,1])
    tool=dict(p=[p['settings'].expected_tcp.x,p['settings'].expected_tcp.y,
                 p['settings'].expected_tcp.z],q=[0,0,0,1])
    observation=dict(state='IDLE',q=q,qd=[0.]*6,qdd=[0.]*6,tcp=tcp,tcp_setting=tool)
    sdk_tcp=dict(x=.3,y=0.,z=.4,rx=0.,ry=0.,rz=0.)
    kin=dict(actual_joint_pose=q,target_joint_pose=q,actual_joint_speed=[0.]*6,
        actual_joint_acc=[0.]*6,target_joint_speed=[0.]*6,target_joint_acc=[0.]*6,
        actual_tcp_pose=sdk_tcp)
    rows=[]
    def row(method,raw=None):
        rows.append(dict(method=method,entered=True,outcome='returned',
            remote_result_unknown=False,raw=raw))
    row('connect')
    def group():
        for method,raw in [('is_connected',True),('get_robot_state','IDLE'),
            ('get_estop_reason',0),('get_running_motion',0),('get_kin_data',kin),
            ('get_tcp',asdict(p['settings'].expected_tcp)),('get_robot_state','IDLE')]:row(method,raw)
    group();row('kinematics_forward',sdk_tcp);row('kinematics_inverse',q)
    for _ in range(3):row('kinematics_forward',sdk_tcp)
    group()
    report=dict(schema=1,runtime='LIVE_READONLY',complete=True,kinematics_consistent=True,
        motion_authorized=False,hardware_stop_verified=False,pending_remote_result_unknown=False,
        initial=observation,final=observation,calls=rows)
    path=output/'result.json';path.write_text(json.dumps(report))
    scope=dict(schema=1,scope='one_readonly_fk_ik_survey_no_robot_writes',mode='readonly',
        target_ip=p['settings'].ip,output=str(output),allowed_writes=[],max_connections=1,
        max_total_calls_including_connect=24,max_ik_calls=1,max_fk_calls=4)
    preview=folder/'preview.json';preview.write_text(json.dumps(scope))
    admission=output/'admission.json';admission.write_text(json.dumps(dict(scope=scope,
        approved_preview_sha256=e.sha(preview))))
    return path,report,admission,preview


def test_readonly_reference_preserves_saved_target_and_stop_uncertainty(context):
    e,p,kw,c,clock=context;path,_,admission,preview=readonly_reference(context)
    fresh=e.prepare(p['config'],path,p['ready_pose'],p['output'])
    v=e.preview(fresh)
    assert fresh['start_q']==p['start_q']
    assert v['target_q_rad']==list(p['settings'].teleop_ready_q)
    assert v['reference_kind']=='completed_readonly_survey'
    assert not v['motion_time_stop_verified'] and not v['motion_authorized']
    assert v['prior_stop_uncertainty_cleared'] is False
    assert str(admission) in fresh['input_hashes'] and str(preview) in fresh['input_hashes']


async def test_readonly_start_executes_existing_return_once_without_routine_stop(context):
    e,p,kw,c,clock=context;path,r,_,_=readonly_reference(context)
    # A small settled servo error is permitted by the unchanged live start check.
    for call in r['calls']:
        if call['method']=='get_kin_data':
            call['raw']['target_joint_pose']=list(call['raw']['actual_joint_pose'])
            call['raw']['target_joint_pose'][4]+=.0001
    path.write_text(json.dumps(r))
    fresh=e.prepare(p['config'],path,p['ready_pose'],p['output'],expires=kw['expires'])
    kw['approved'].write_text(json.dumps(e.preview(fresh)))
    kw['expected_hash']=e.sha(kw['approved'])
    result=await e.execute(fresh,**kw,client_factory=AsyncMock(return_value=c),
        clock=clock.now_ns,sleep=clock.sleep)
    assert result['complete'],result
    assert result['movej_entries']==1 and result['stop_entries']==0


@pytest.mark.parametrize('file_index',[0,1])
async def test_changed_readonly_seal_never_connects(context,file_index):
    e,p,kw,c,clock=context;path,_,admission,preview=readonly_reference(context)
    fresh=e.prepare(p['config'],path,p['ready_pose'],p['output'],expires=kw['expires'])
    kw['approved'].write_text(json.dumps(e.preview(fresh)));kw['expected_hash']=e.sha(kw['approved'])
    (admission,preview)[file_index].write_text('{}')
    factory=AsyncMock(return_value=c)
    with pytest.raises(ValueError):
        await e.execute(fresh,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    factory.assert_not_called()


@pytest.mark.parametrize('damage',['incomplete','simulation','unknown','paused','moving_q',
    'target_q','target_speed','tool','write','address','seal','missing_call','failed_call',
    'tcp_mismatch','raw_tcp_missing','fk_payload','ik_payload'])
def test_readonly_reference_rejects_damage_before_sdk(context,damage):
    e,p,kw,c,clock=context;path,r,admission,preview=readonly_reference(context)
    if damage=='incomplete':r['complete']=False
    elif damage=='simulation':r['runtime']='SIMULATION'
    elif damage=='unknown':r['pending_remote_result_unknown']=True
    elif damage=='paused':r['final']['state']='PAUSED'
    elif damage=='moving_q':r['final']['qd'][0]=.001
    elif damage in ('target_q','target_speed'):
        raw=next(x['raw'] for x in r['calls'] if x['method']=='get_kin_data')
        key='target_joint_pose' if damage=='target_q' else 'target_joint_speed'
        raw[key]=list(raw[key]);raw[key][0]+=.01
    elif damage=='tool':r['final']['tcp_setting']['p'][2]+=.01
    elif damage in ('write','address'):
        a=json.loads(admission.read_text())
        if damage=='write':a['scope']['allowed_writes']=['stop_move']
        else:a['scope']['target_ip']='192.168.42.2'
        preview.write_text(json.dumps(a['scope']));a['approved_preview_sha256']=e.sha(preview)
        admission.write_text(json.dumps(a))
    elif damage=='seal':preview.write_text('{}')
    elif damage=='missing_call':r['calls'].pop()
    elif damage=='tcp_mismatch':r['final']['tcp']['p'][2]+=.01
    elif damage=='raw_tcp_missing':
        del next(x['raw'] for x in r['calls'] if x['method']=='get_kin_data')['actual_tcp_pose']
    elif damage=='fk_payload':next(x for x in r['calls'] if x['method']=='kinematics_forward')['raw']={}
    elif damage=='ik_payload':next(x for x in r['calls'] if x['method']=='kinematics_inverse')['raw']=[0.]*5
    else:r['calls'][-1]['remote_result_unknown']=True
    path.write_text(json.dumps(r))
    with pytest.raises(ValueError,match='ready_return_reference_rejected'):
        e.prepare(p['config'],path,p['ready_pose'],p['output'])


def test_cli_process_exits_with_a_cancellation_resistant_abandoned_task(tmp_path):
    # Only loop-shutdown behavior: no native loader, connection or controller.
    code='''
import asyncio,json,sys
from pathlib import Path
from scripts import real_robot_ready_return as e
output=Path(sys.argv[1])
e.prepare=lambda *a,**kw:dict(output=output)
e.load_sdk=lambda:(_ for _ in ()).throw(AssertionError('SDK forbidden'))
async def stubborn():
 while True:
  try:await asyncio.Event().wait()
  except asyncio.CancelledError:pass
async def execute(*a,**kw):
 asyncio.create_task(stubborn());await asyncio.sleep(0)
 output.mkdir()
 r=dict(complete=False,movej_entries=1,stop_entries=1,latched_fault='stop_unverified')
 (output/'result.json').write_text(json.dumps(r))
 return r
e.execute=execute
raise SystemExit(e.main(['--config','unused','--start-session','unused','--ready-pose','unused','--output',str(output),'--execute-once']))
'''
    try:
        r=subprocess.run([sys.executable,'-c',code,str(tmp_path/'result')],cwd=SOURCE.parents[1],
            capture_output=True,text=True,timeout=8)
    except subprocess.TimeoutExpired:
        pytest.fail('CLI did not exit after the bounded result; cancellation-resistant task still awaited')
    assert r.returncode==2,r.stderr
    result=json.loads((tmp_path/'result/result.json').read_text())
    assert not result['complete'] and result['pending_tasks_at_process_exit']>=1


def test_cli_keyboard_interrupt_leaves_time_for_original_cleanup(tmp_path):
    code='''
import asyncio,json,sys
from pathlib import Path
from scripts import real_robot_ready_return as e
output=Path(sys.argv[1]);e.prepare=lambda *a,**kw:dict(output=output)
e.load_sdk=lambda:(_ for _ in ()).throw(AssertionError('SDK forbidden'))
def interrupt():raise KeyboardInterrupt
async def execute(*a,**kw):
 output.mkdir();asyncio.get_running_loop().call_later(.02,interrupt)
 try:await asyncio.Event().wait()
 except asyncio.CancelledError:
  await asyncio.sleep(.2)
  (output/'result.json').write_text(json.dumps(dict(complete=False,cleanup_finished=True,movej_entries=1,stop_entries=1,latched_fault=None)))
  raise
e.execute=execute
raise SystemExit(e.main(['--config','unused','--start-session','unused','--ready-pose','unused','--output',str(output),'--execute-once']))
'''
    r=subprocess.run([sys.executable,'-c',code,str(tmp_path/'result')],cwd=SOURCE.parents[1],capture_output=True,text=True,timeout=8)
    path=tmp_path/'result/result.json'
    assert path.exists(),'loop closed before original cancellation cleanup completed'
    v=json.loads(path.read_text())
    assert r.returncode==130 and v['cleanup_finished'] and v['interrupted'] and not v['complete']
