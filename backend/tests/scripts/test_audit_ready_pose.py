"""Offline capture/config comparisons; no native SDK or robot calls."""
import builtins
import copy
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest

from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.real_settings import readonly_settings


@pytest.fixture
def entry():
    spec = importlib.util.spec_from_file_location('audit_ready_entry',
        Path(__file__).resolve().parents[3]/'scripts/audit_ready_pose.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def recording(settings):
    initial = dict(state='IDLE', q_rad=list(settings.teleop_ready_q),
        qd_radps=[0.]*6, qdd_radps2=[0.]*6,
        actual_tcp_pose=dict(x=.3,y=0.,z=.3,rz=0.,ry=0.,rx=0.),
        tool_tcp_setting=asdict(settings.expected_tcp))
    final = copy.deepcopy(initial)
    digest = hashlib.sha256(json.dumps(dict(initial=initial,final=final),
        sort_keys=True,allow_nan=False,separators=(',',':')).encode()).hexdigest()
    methods = ['connect']+['is_connected','get_robot_state','get_estop_reason',
        'get_kin_data','get_tcp','get_robot_state']*2
    return dict(complete=True,runtime='LIVE_READONLY',motion_authorized=False,
        pending_remote_result_unknown=False,calls=[dict(method=m,entered=True,outcome='returned',
            remote_result_unknown=False) for m in methods],initial=initial,final=final,
        operator_selected_ready_pose=dict(q_rad=final['q_rad'],actual_tcp_pose=final['actual_tcp_pose'],
            tool_tcp_setting=final['tool_tcp_setting'],raw_state='IDLE',
            source=dict(target_ip=settings.ip,telemetry_sha256=digest)))


def test_compatible_record_does_not_authorize_hardware(entry):
    settings=readonly_settings()
    result=entry.assess(recording(settings),settings)
    assert result['configuration_compatible']
    assert result['blocking_reasons']==[]
    assert result['historical_capture_only']
    assert result['motion_authorized'] is result['hardware_ready'] is False
    assert result['remaining_live_gates']==['FK_IK_and_timing','motion_release_stop','empty_gripper']


@pytest.mark.parametrize('axis,index', [('x',0),('y',1),('z',2)])
def test_old_workspace_rejects_current_pose_without_changing_config(entry,axis,index):
    settings=readonly_settings();record=recording(settings)
    from dataclasses import replace
    lower=list(settings.startup_tcp_min_m);lower[index]=1.
    upper=list(settings.startup_tcp_max_m);upper[index]=2.
    old=replace(settings,startup_tcp_min_m=tuple(lower),startup_tcp_max_m=tuple(upper))
    result=entry.assess(record,old)
    assert not result['configuration_compatible']
    assert not result['startup_workspace'][axis]['current_inside']
    assert 'startup_workspace' in result['blocking_reasons']
    assert old.startup_tcp_min_m[index]==1.


def test_tool_and_ready_mismatches_are_separate(entry):
    from dataclasses import replace
    settings=readonly_settings();record=recording(settings)
    modified=replace(settings,teleop_ready_q=(.1,-1.,1.,0.,.2,0.),
        expected_tcp=replace(settings.expected_tcp,z=.2))
    result=entry.assess(record,modified)
    assert set(result['blocking_reasons'])=={'ready_pose_matches','tool_tcp_matches'}


@pytest.mark.parametrize('damage', ['incomplete','write','unknown','extra_call','changed','hash','nonfinite','target'])
def test_invalid_capture_fails_closed(entry,damage):
    settings=readonly_settings();record=recording(settings)
    if damage=='incomplete':record['complete']=False
    if damage=='write':record['calls'][3]['method']='stop_move'
    if damage=='unknown':record['calls'][3]['remote_result_unknown']=True
    if damage=='extra_call':record['calls'].append(record['calls'][0])
    if damage=='changed':record['final']['q_rad'][0]=.1
    if damage=='hash':record['operator_selected_ready_pose']['source']['telemetry_sha256']='0'*64
    if damage=='nonfinite':record['initial']['q_rad'][0]=float('nan')
    if damage=='target':record['operator_selected_ready_pose']['source']['target_ip']='192.168.10.21'
    with pytest.raises(ValueError):entry.assess(record,settings)


def test_cli_only_reads_files_and_never_imports_sdk(entry,tmp_path,monkeypatch,capsys):
    config=tmp_path/'readonly.yaml';config.write_text(REAL_CONFIG_TEMPLATE.format(mode='readonly'))
    from app.config import Settings
    settings=Settings.load(config).lebai
    path=tmp_path/'capture.json';path.write_text(json.dumps(recording(settings)))
    original=builtins.__import__
    def reject(name,*args,**kwargs):
        if name.startswith('lebai_sdk'):raise AssertionError('SDK forbidden')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',reject)
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    assert entry.main(['--config',str(config),'--capture',str(path),'--capture-sha256',digest])==0
    assert json.loads(capsys.readouterr().out)['motion_authorized'] is False
    with pytest.raises(ValueError,match='capture_hash_mismatch'):
        entry.main(['--config',str(config),'--capture',str(path),'--capture-sha256','0'*64])


def test_cli_rejects_control_config_before_loading_settings(entry,tmp_path,monkeypatch):
    config=tmp_path/'control.yaml';config.write_text(REAL_CONFIG_TEMPLATE.format(mode='control'))
    path=tmp_path/'capture.json';path.write_text('{}')
    with pytest.raises(ValueError,match='readonly_config_required'):
        entry.main(['--config',str(config),'--capture',str(path),'--capture-sha256',hashlib.sha256(path.read_bytes()).hexdigest()])
