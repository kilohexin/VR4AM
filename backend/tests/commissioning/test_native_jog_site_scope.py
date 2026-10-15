"""Synthetic site scope; no SDK or network."""
from dataclasses import replace
import pytest
from scipy.spatial.transform import Rotation
from app.commissioning.native_jog_site_scope import SiteJogScope
from app.commissioning.native_jog_packet import NativeJogLimits
from app.commissioning.native_jog_mdh import MdhTool
from tests.commissioning.test_native_jog_mdh import independent_transform
from tests.robots.real_settings import control_settings
from app.schemas.messages import Pose

Q=(0.,-1.2,1.8,-.6,1.,0.)
TOOL=MdhTool((0.,0.,.175),(0.,0.,0.,1.))
SHA='a'*64

def arguments(ip='127.0.0.1',**changes):
    t=independent_transform(Q,TOOL)
    pose=Pose(p=tuple(t[:3,3]),q=tuple(Rotation.from_matrix(t[:3,:3]).as_quat()))
    lo=tuple(v-.1 for v in pose.p);hi=tuple(v+.1 for v in pose.p)
    s=control_settings(ip=ip)
    s=replace(s,teleop_ready_q=Q,startup_tcp_min_m=lo,startup_tcp_max_m=hi,
        control=replace(s.control,max_joint_speed_radps=.05,max_joint_acceleration_radps2=.2))
    args=dict(settings=s,limits=NativeJogLimits(.005,.01,lo,hi,(.001,)*3),config_sha256=SHA,
        reference_tool=TOOL,authorize=lambda:None,hot_check=lambda:None,write_check=lambda:None)
    args.update(changes)
    return args,pose

@pytest.mark.parametrize('ip',['127.0.0.1','192.168.42.10','10.1.2.3','172.16.1.2'])
def test_scope_fixed_model_and_address(ip):
    args,_=arguments(ip);s=SiteJogScope(**args)
    assert s.model_guard().assessment['model_reference_eligible']
    assert not s.model_guard().assessment['continuous_control_admitted']
    s.check();s.hot();s.write()

@pytest.mark.parametrize('ip',['8.8.8.8','localhost','::1','0.0.0.0','127.0.0.2','169.254.1.1'])
def test_invalid_addresses_refused(ip):
    args,_=arguments(ip)
    with pytest.raises(ValueError):SiteJogScope(**args)

@pytest.mark.parametrize('change',['speed','acc','joint','sha','callback','workspace'])
def test_invalid_scope_refused(change):
    args,_=arguments()
    if change=='speed':args['limits']=replace(args['limits'],max_speed_mps=.006)
    elif change=='acc':args['limits']=replace(args['limits'],max_acceleration_mps2=.02)
    elif change=='joint':args['settings']=replace(args['settings'],control=replace(args['settings'].control,max_joint_speed_radps=.15))
    elif change=='sha':args['config_sha256']='bad'
    elif change=='callback':args['authorize']=None
    else:args['limits']=replace(args['limits'],tcp_min_m=(-9.,-9.,-9.))
    with pytest.raises(ValueError):SiteJogScope(**args)

def test_permission_result_must_be_none():
    args,_=arguments(authorize=lambda:True);s=SiteJogScope(**args)
    with pytest.raises(ValueError):s.check()
