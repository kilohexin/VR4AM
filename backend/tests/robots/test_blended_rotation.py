import math
from dataclasses import replace
from types import SimpleNamespace as NS
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from app.robots.blended_rotation_path import audit_rotation_path
from app.robots.blended_rotation_docker import DockerRotationPort
from app.digital_twin.docker_runtime import docker_settings


class Model:
    model_sha256='c'*64
    gain=1.
    def forward(self,q):return tuple(q[:3]),tuple(Rotation.from_rotvec(np.asarray(q[3:])*self.gain).as_quat())
    def jacobian(self,q):return np.diag([1.]*3+[self.gain]*3)
    def jdot(self,q,qd):return np.zeros((6,6))


def fixture(axis=0):
    settings=docker_settings().lebai
    settings=replace(settings,soft_joint_min_rad=(-3.,)*6,soft_joint_max_rad=(3.,)*6,
        startup_tcp_min_m=(-1.,)*3,startup_tcp_max_m=(1.,)*3)
    joints=[];rotations=[];model=Model()
    for fraction in (0.,.25,.5,.75,1.):
        q=[0.]*6;q[axis+3]=math.radians(.5)*fraction
        joints.append(q);rotations.append(model.forward(q)[1])
    return settings,model,joints,rotations


@pytest.mark.parametrize('axis',[0,2])
def test_rotation_rates_and_fixed_position_are_audited(axis):
    s,m,j,r=fixture(axis);m.gain=.1
    j=[[x/.1 for x in row] for row in j]
    s=replace(s,control=replace(s.control,max_joint_step_rad=.2))
    audit=audit_rotation_path((0,0,0),r,j,model=m,settings=s,reference=(0,0,0,1))
    assert audit.acceleration/.1<=s.control.max_joint_acceleration_radps2
    assert audit.speed<=.02


@pytest.mark.parametrize('damage,reason',[
    ('drift','model_mismatch'),('curve','rotation_path'),('singular','singular'),
    ('joint','joint_limit'),('range','rotation_range')])
def test_invalid_rotation_is_rejected(damage,reason):
    s,m,j,r=fixture();reference=(0,0,0,1)
    if damage=='drift':j[2][0]=.001
    if damage=='curve':r[2]=tuple(Rotation.from_rotvec([.001,.001,0]).as_quat())
    if damage=='singular':m.jacobian=lambda q:np.zeros((6,6))
    if damage=='joint':s=replace(s,soft_joint_max_rad=(0.,)*6)
    if damage=='range':reference=tuple(Rotation.from_euler('z',120,degrees=True).as_quat())
    with pytest.raises(ValueError,match=reason):audit_rotation_path((0,0,0),r,j,model=m,settings=s,reference=reference)


@pytest.mark.parametrize('axis',[0,2])
def test_coordinates_follow_tool_axis_and_accept_equivalent_quaternion(axis):
    p=object.__new__(DockerRotationPort);p.axis=axis
    p.orientation=tuple(Rotation.from_euler('z',90,degrees=True).as_quat())
    p.adapter=NS(_position_pitch_reference=p.orientation)
    start=NS(actual_tcp=NS(p=(.2,0,.3),q=p.orientation));p._initialize_reference(start)
    vector=np.zeros(3);vector[axis]=math.radians(.5)
    q=(Rotation.from_quat(p.orientation)*Rotation.from_rotvec(vector)).as_quat()
    s=NS(actual_tcp=NS(p=start.actual_tcp.p,q=tuple(-q)))
    p._validate_pose(s)
    assert p._coordinates(s)[axis]==pytest.approx(.004)
    s.actual_tcp.p=(.201,0,.3)
    with pytest.raises(ValueError,match='tcp_drift'):p._validate_pose(s)
