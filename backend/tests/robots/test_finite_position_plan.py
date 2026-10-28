import importlib
import math
from dataclasses import replace
import pytest
from app.control.finite_position_types import PositionBinding,PositionSample,PositionIntent
from app.schemas.messages import BackendState
from tests.robots.finite_position_fixture import LinearModel,fixture


def api():
    assert importlib.util.find_spec('app.robots.finite_position_plan'), 'position planner missing'
    return importlib.import_module('app.robots.finite_position_plan')


def sample():
    z=(0.,)*6;q=(0.,-1.,1.,0.,1.57,0.)
    return PositionSample(PositionBinding(object(),1,'a'*64,'b'*64),1,0,q,q,z,z,z,z,
        (.3,0.,.4),(0.,0.,0.,1.),BackendState.IDLE,None,'FINISHED',None)


def test_diagonal_target_is_two_millimetres():
    s=sample();p,q=api().build_position_target(s,PositionIntent('web',1,1,0,'held',(1.,1.,1.)))
    assert math.isclose(math.dist(p,s.tcp_p),.002,abs_tol=1e-12) and q==s.tcp_q


@pytest.mark.parametrize('direction',[(.16,0.,0.),(.5,0.,0.),(1.,0.,0.),(1.,1.,1.)])
def test_above_deadzone_always_builds_fixed_two_mm_goal(direction):
    s=sample();intent=PositionIntent('web',1,1,0,'held',direction)
    p,q=api().build_position_target(s,intent)
    assert math.isclose(math.dist(p,s.tcp_p),.002,abs_tol=1e-12)
    assert math.isclose(math.hypot(*intent.vector),1.,abs_tol=1e-12)
    assert q==s.tcp_q


@pytest.mark.parametrize('direction',[(0.,0.,0.),(.15,0.,0.),(.1,.1,0.)])
def test_deadzone_is_normal_release_not_a_tiny_goal(direction):
    intent=PositionIntent('web',1,1,0,'held',direction)
    assert intent.kind=='release' and intent.vector==(0.,0.,0.)


def test_deadzone_release_cannot_build_a_motion_target():
    with pytest.raises(ValueError,match='unresolvable'):
        api().build_position_target(sample(),PositionIntent('web',1,1,0,'held',(1e-12,0.,0.)))


def test_path_audit_rejects_workspace_and_bad_model():
    m=api();s=sample();settings,_,_=fixture();goal=list(s.actual_q);goal[0]=.0005
    fk=tuple(LinearModel().forward(tuple(a+(b-a)*f for a,b in zip(s.actual_q,goal))) for f in (0,.25,.5,.75,1))
    audit=m.audit_position_path(s,tuple(goal),fk_samples=fk,settings=settings,model=LinearModel())
    assert audit.model_rates_ok and not audit.factory_calibrated
    bad=replace(settings,startup_tcp_max_m=(.3,.3,.6))
    with pytest.raises(ValueError):m.audit_position_path(s,tuple(goal),fk_samples=fk,settings=bad,model=LinearModel())



def test_explicit_virtual_duration_budget_accepts_same_slow_bounded_path():
    m=api();s=sample();settings,_,_=fixture()
    settings=replace(settings,control=replace(settings.control,max_tcp_speed_mps=.005))
    goal=tuple(x+(.002 if i==0 else 0) for i,x in enumerate(s.actual_q))
    fk=tuple(LinearModel().forward(tuple(a+(b-a)*f for a,b in zip(s.actual_q,goal))) for f in m.FRACTIONS)
    with pytest.raises(ValueError,match='estimated_duration'):
        m.audit_position_path(s,goal,fk_samples=fk,settings=settings,model=LinearModel())
    audit=m.audit_position_path(s,goal,fk_samples=fk,settings=settings,model=LinearModel(),duration_limit_s=4.8)
    assert audit.model_rates_ok and audit.joint_speed<=.05 and audit.joint_acceleration<=.2


@pytest.mark.parametrize('budget',[0,-1,True,float('nan'),float('inf'),5.])
def test_virtual_duration_budget_must_leave_the_arrival_margin(budget):
    with pytest.raises(ValueError,match='duration_budget'):
        api().audit_position_path(sample(),(),fk_samples=(),settings=None,model=None,duration_limit_s=budget)
