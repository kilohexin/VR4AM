"""Default preview and invalid-authority checks with synthetic evidence only."""
import importlib.util
import json
from dataclasses import asdict,replace
from datetime import UTC,datetime,timedelta
from pathlib import Path
from unittest.mock import AsyncMock
import pytest
import yaml
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.commissioning.test_finite_position_site import fixture as position_fixture
from tests.commissioning.test_readonly_jog_jacobian import JacobianClient
from app.commissioning.readonly_jog_jacobian import collect,new_report
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import Pose

SOURCE=Path(__file__).resolve().parents[3]/'scripts/real_robot_finite_position.py'


def test_default_preview_entry_exists():
    assert SOURCE.is_file(),'single-position default-preview CLI missing'


@pytest.fixture
async def context(tmp_path,monkeypatch):
    assert SOURCE.exists(),'single-position default-preview CLI missing'
    spec=importlib.util.spec_from_file_location('single_position_entry_test',SOURCE)
    e=importlib.util.module_from_spec(spec);spec.loader.exec_module(e)
    monkeypatch.setattr(e,'ROOT',tmp_path);code=tmp_path/'code.py';code.write_text('synthetic')
    monkeypatch.setattr(e,'runtime_hashes',lambda:{'code.py':e.sha(code)})
    monkeypatch.setattr(e,'existing_rpc_connections',lambda:0)
    monkeypatch.setattr(e,'load_sdk',lambda:pytest.fail('native SDK forbidden'))
    s,c,clock,_,model=position_fixture();ip='192.168.42.10'
    data=yaml.safe_load(REAL_CONFIG_TEMPLATE.format(mode='control'));real=data['real_robot'];real['ip']=ip
    for key in ('home_q','teleop_ready_q','startup_tcp_min_m','startup_tcp_max_m'):real[key]=list(getattr(s,key))
    real['control'].update(max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2)
    config=tmp_path/'config.yaml';config.write_text(yaml.safe_dump(data))
    from app.config import Settings,REAL_ROBOT_CONFIRMATION
    monkeypatch.setenv('VR4ARM_REAL_ROBOT_CONFIRM',REAL_ROBOT_CONFIRMATION)
    settings=Settings.load(config).lebai;survey_client=JacobianClient()
    async def factory(ip):return survey_client
    report=new_report(simulation=True)
    await collect(replace(settings,mode='readonly'),factory,lambda:None,report)
    assert report['complete'],report
    report['runtime']='LIVE_READONLY'  # synthetic evidence, injected execution only
    reference=tmp_path/'reference/result.json';reference.parent.mkdir();reference.write_text(json.dumps(report))
    reference.with_name('admission.json').write_text(json.dumps(dict(scope=dict(
        scope='one_readonly_jog_jacobian_no_robot_writes',target_ip=ip,allowed_writes=[],max_fk_calls=13,max_ik_calls=0))))
    ready=tmp_path/'ready.json';ready.write_text(json.dumps(dict(selection='operator_selected_current_working_ready_pose',
        raw_state='IDLE',q_rad=list(settings.teleop_ready_q),tool_tcp_setting=asdict(settings.expected_tcp),
        actual_tcp_pose=pose_to_lebai(Pose(p=model.forward(settings.teleop_ready_q)[0],q=model.forward(settings.teleop_ready_q)[1])),
        source=dict(target_ip=ip))))
    expiry=(datetime.now(UTC)+timedelta(minutes=5)).isoformat()
    p=e.prepare(config,reference,ready,tmp_path/'.tmp/once',expires=expiry)
    approved=tmp_path/'preview.json';approved.write_text(json.dumps(e.preview(p)))
    kwargs=dict(approved=approved,expected_hash=e.sha(approved),expires=expiry,confirm=e.CONFIRM)
    return e,p,kwargs,c,clock


async def test_default_preview_is_zero_access(context,monkeypatch,capsys):
    e,p,_,c,_=context;monkeypatch.setattr(e,'execute',AsyncMock(side_effect=AssertionError('preview execution')))
    assert e.main(['--config',str(p['config']),'--reference-survey',str(p['reference_survey']),
        '--ready-pose',str(p['ready_pose']),'--output',str(p['output'])])==0
    row=json.loads(capsys.readouterr().out)
    assert row['distance_m']==.002 and row['max_movej_entries']==row['max_stop_entries']==1
    assert row['min_progress_m']==.001 and row['max_endpoint_error_m']==.0005
    assert row['joint_endpoint_tolerance_rad']==pytest.approx(__import__('math').radians(.1))
    assert row['ready_tolerance_rad']==row['joint_endpoint_tolerance_rad']
    assert row['held_input_deadzone']==.15
    assert row['actual_acceleration_policy']=='bounded_transient_at_point_two_strict_below'
    assert row['actual_acceleration_sustained_limit_radps2']==.2
    assert row['actual_acceleration_instant_limit_radps2']==.4
    assert row['actual_acceleration_consecutive_samples']==3 and row['actual_acceleration_max_sample_gap_ms']==100
    assert e.CONFIRM=='I_APPROVE_ONE_2MM_FINITE_POSITION_Z_NATURAL_ARRIVAL_AND_RESERVED_FAULT_STOP'
    assert row['max_ik_calls']==1 and row['max_fk_calls']==5 and not row['motion_authorized']
    assert row['natural_arrival_does_not_send_stop'] and not p['output'].exists() and not c.write_calls
    assert row['input_ttl_ms'] is None and row['held_input_ttl_ms']==100
    assert row['planning_timeout_ms']==1000 and row['entry_policy']=='fresh_identical_start_after_planning'
    assert row['snapshot_read_policy']=='owned_six_fields_parallel'
    assert row['readonly_authorization_policy']=='full_at_group_boundaries_binding_and_cancellation_per_read'


@pytest.mark.parametrize('kind',['confirm','expiry','hash','preview','config','ready_pose','reference_survey','runtime','rpc'])
async def test_invalid_scope_is_zero_native_access(context,monkeypatch,kind):
    e,p,kw,c,clock=context;factory=AsyncMock(return_value=c)
    if kind=='confirm':kw['confirm']='I_APPROVE_ONE_SPEEDL_Z_DIAGNOSTIC_AND_ONE_ORIGINAL_STOP'
    elif kind=='expiry':kw['expires']='2000-01-01T00:00:00Z'
    elif kind=='hash':kw['expected_hash']='b'*64
    elif kind=='preview':kw['approved'].write_text('{}')
    elif kind=='runtime':(e.ROOT/'code.py').write_text('changed')
    elif kind=='rpc':monkeypatch.setattr(e,'existing_rpc_connections',lambda:1)
    else:p[kind].write_bytes(p[kind].read_bytes()+b'\n')
    with pytest.raises(ValueError):await e.execute(p,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    factory.assert_not_called();assert not p['output'].exists()


async def test_injected_execution_is_consumed_once_and_persists_terminal_result(context):
    e,p,kw,c,clock=context;factory=AsyncMock(return_value=c)
    r=await e.execute(p,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    assert r['complete'],r
    assert r['runtime']=='SIMULATION' and r['movej_entries']==1 and r['stop_entries']==0
    assert json.loads((p['output']/'result.json').read_text())==r
    with pytest.raises(FileExistsError):await e.execute(p,**kw,client_factory=factory,clock=clock.now_ns,sleep=clock.sleep)
    assert factory.await_count==1
