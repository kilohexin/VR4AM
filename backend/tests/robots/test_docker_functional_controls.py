import math
from dataclasses import replace
from types import SimpleNamespace
import pytest
from scipy.spatial.transform import Rotation
from app.control.finite_position_types import PositionIntent
from app.control.finite_position_gate import PositionGate
from app.robots.finite_position_plan import build_position_target
from tests.robots.test_finite_position_plan import sample


def test_docker_step_profile_does_not_change_physical_default_targets():
    s=sample();i=PositionIntent('web',1,1,0,'held',(1.,0.,0.))
    assert math.dist(build_position_target(s,i)[0],s.tcp_p)==pytest.approx(.002)
    assert math.dist(build_position_target(s,i,step_m=.008)[0],s.tcp_p)==pytest.approx(.008)


def test_spin_uses_tool_z_and_keeps_translation_fixed():
    s=replace(sample(),tcp_q=tuple(map(float,Rotation.from_euler('y',70,degrees=True).as_quat())))
    i=PositionIntent('web',1,1,0,'held',(0.,0.,0.),spin=1.)
    p,q=build_position_target(s,i,angle_step_rad=math.radians(1.5))
    expected=Rotation.from_quat(s.tcp_q)*Rotation.from_euler('z',1.5,degrees=True)
    assert p==s.tcp_p and (Rotation.from_quat(q)*expected.inv()).magnitude()<1e-10


def test_reversed_spin_revokes_unsent_goal():
    s=sample();g=PositionGate(s.binding,clock=lambda:0)
    g.offer(PositionIntent('web',s.binding.generation,1,0,'held',(0.,0.,0.),spin=1.))
    selected=g.take_latest()
    g.offer(PositionIntent('web',s.binding.generation,2,0,'held',(0.,0.,0.),spin=-1.))
    with pytest.raises(ValueError,match='entry_revoked'):g.require_entry(selected)


@pytest.mark.parametrize('value',[True,float('nan'),float('inf'),2.,'1'])
def test_invalid_spin_is_rejected(value):
    with pytest.raises(ValueError):PositionIntent('web',1,1,0,'held',(0.,0.,0.),spin=value)


def test_release_cannot_carry_spin():
    with pytest.raises(ValueError):PositionIntent('web',1,1,0,'release',(0.,0.,0.),spin=1.)


@pytest.mark.parametrize('initial',[10,100])
async def test_virtual_speed_factor_is_explicitly_set_and_confirmed(initial):
    from app.digital_twin.docker_runtime import ensure_docker_speed
    values=iter([initial,100]);writes=[]
    async def get():return next(values)
    async def set_factor(value):writes.append(value)
    assert await ensure_docker_speed(SimpleNamespace(get_velocity_factor=get,set_velocity_factor=set_factor))==100
    assert writes==([100] if initial!=100 else [])


async def test_unconfirmed_virtual_factor_does_not_allow_startup_to_continue():
    from app.digital_twin.docker_runtime import ensure_docker_speed
    async def get():return 10
    async def set_factor(value):pass
    with pytest.raises(ValueError,match='speed_factor'):
        await ensure_docker_speed(SimpleNamespace(get_velocity_factor=get,set_velocity_factor=set_factor))

def test_docker_numeric_roundoff_is_not_sustained_acceleration_excess():
    from app.robots.finite_position_acceleration import PositionAccelerationMonitor
    g=PositionAccelerationMonitor(.2,numeric_epsilon=1e-9)
    for n in range(5):
        g.observe((.2000000000035307,)*6,captured_ns=n,now_ns=n)
    with pytest.raises(ValueError,match='reported_rate_limit'):
        g.observe((.400001,)*6,captured_ns=6,now_ns=6)
