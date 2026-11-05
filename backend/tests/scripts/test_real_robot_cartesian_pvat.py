from hashlib import sha256
import json

import pytest


def manifest(tmp_path):
    q=[0.,-.8,1.4,-.6,1.57,0.]
    report=dict(mode='readonly',complete=True,config_unchanged=True,reads=[
        dict(method='get_tcp',outcome='returned',raw=dict(x=.01,y=-.02,z=.2,rx=0.,ry=0.,rz=0.)),
        dict(method='get_kin_data',outcome='returned',raw=dict(actual_joint_pose=q,target_joint_pose=q,
            actual_joint_speed=[0.]*6,actual_joint_acc=[0.]*6,target_joint_speed=[0.]*6,target_joint_acc=[0.]*6))])
    tool=tmp_path/'readonly.json';tool.write_text(json.dumps(report),encoding='utf-8')
    config=tmp_path/'config.yaml';config.write_text(json.dumps(dict(real_robot=dict(
        soft_joint_min_rad=[-3.]*6,soft_joint_max_rad=[3.]*6,joint_limit_margin_rad=.01))),encoding='utf-8')
    from app.commissioning.native_jog_mdh import MdhTool
    from app.control.cartesian_pvat_reference import MdhReferenceModel
    p=MdhReferenceModel(MdhTool((.01,-.02,.2),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6).pose(q).p
    data=dict(schema=1,tool_report=str(tool),tool_report_sha256=sha256(tool.read_bytes()).hexdigest(),
        config=str(config),config_sha256=sha256(config.read_bytes()).hexdigest(),kin_read_index=1,
        workspace_lower=[x-.01 for x in p],workspace_upper=[x+.01 for x in p])
    path=tmp_path/'manifest.json';path.write_text(json.dumps(data),encoding='utf-8')
    return path,data


def test_preview_is_explicitly_not_an_executable_or_live_authorization(tmp_path):
    from scripts.real_robot_cartesian_pvat import preview_manifest,main
    path,_=manifest(tmp_path);result=preview_manifest(path)
    assert result['points']==20 and not result['execute_available'] and not result['motion_admitted']
    assert result['joint_lower']==[-2.99]*6
    assert result['origin_is_saved_not_fresh'] and result['scope_requires_onsite_approval']
    assert result['sdk_connections']==result['robot_writes']==0
    assert len(result['contract_sha256'])==64
    with pytest.raises(SystemExit):main(['--execute-once'])


@pytest.mark.parametrize('change',['config','tool','index','moving','bounds','margin'])
def test_changed_or_invalid_local_inputs_fail_before_building_a_preview(tmp_path,change):
    from scripts.real_robot_cartesian_pvat import preview_manifest
    path,data=manifest(tmp_path)
    if change in ('tool','config'):
        from pathlib import Path
        Path(data['tool_report' if change=='tool' else 'config']).write_text('{}')
    elif change=='index':data['kin_read_index']=0
    elif change=='bounds':data['workspace_upper']=data['workspace_lower']
    elif change=='margin':
        from pathlib import Path
        p=Path(data['config']);doc=json.loads(p.read_text());doc['real_robot']['joint_limit_margin_rad']=-1
        p.write_text(json.dumps(doc));data['config_sha256']=sha256(p.read_bytes()).hexdigest()
    else:
        from pathlib import Path
        p=Path(data['tool_report']);doc=json.loads(p.read_text());doc['reads'][1]['raw']['actual_joint_acc'][0]=.01
        p.write_text(json.dumps(doc));data['tool_report_sha256']=sha256(p.read_bytes()).hexdigest()
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):preview_manifest(path)
