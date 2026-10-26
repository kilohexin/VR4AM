"""Pitch is a Cartesian tool rotation, not an unchecked wrist joint command."""
import math
from dataclasses import replace
import pytest
from scipy.spatial.transform import Rotation
from app.control.finite_position_types import PositionIntent,PositionPlan,PathAudit
from app.robots.finite_position_plan import build_position_target,evaluate_position_arrival
from tests.robots.test_finite_position_plan import sample
from tests.robots.finite_position_fixture import LinearModel,fixture


def pitch_intent(pitch=1.,vector=(0.,0.,0.)):
    return PositionIntent('web',1,1,0,'held',vector,pitch=pitch)


def test_pure_pitch_keeps_tcp_and_rotates_about_local_tool_x():
    s=replace(sample(),tcp_q=tuple(map(float,Rotation.from_euler('z',90,degrees=True).as_quat())))
    p,q=build_position_target(s,pitch_intent())
    expected=Rotation.from_quat(s.tcp_q)*Rotation.from_euler('x',.5,degrees=True)
    assert p==s.tcp_p
    assert (Rotation.from_quat(q)*expected.inv()).magnitude()<1e-10


def test_pitch_combines_with_bounded_translation_and_preserves_zero_pitch_compatibility():
    s=sample();p,q=build_position_target(s,pitch_intent(-.8,(1.,1.,0.)))
    assert math.dist(p,s.tcp_p)==pytest.approx(.001)
    assert Rotation.from_quat(q).as_rotvec()[0]==pytest.approx(-math.radians(.25))
    assert PositionIntent('web',1,1,0,'held',(0.,0.,0.)).kind=='release'
    assert pitch_intent().kind=='held'


@pytest.mark.parametrize('bad',[True,float('nan'),float('inf'),2.,'1'])
def test_pitch_rejects_nonfinite_boolean_and_out_of_range(bad):
    with pytest.raises(ValueError):pitch_intent(bad)


def test_release_cannot_carry_pitch():
    with pytest.raises(ValueError):PositionIntent('web',1,1,0,'release',(0.,0.,0.),pitch=1.)


@pytest.mark.parametrize('degrees,expected',[(0.,'position_no_actual_progress'),(-1.,'position_no_actual_progress'),
    (.2,'position_no_actual_progress'),(.5,None),(1.,'position_orientation_error')])
def test_pure_pitch_arrival_requires_actual_angular_progress(degrees,expected):
    s=sample();p,q=build_position_target(s,pitch_intent())
    joints=(*s.actual_q[:3],.01,*s.actual_q[4:])
    audit=PathAudit('c'*64,(0.,.25,.5,.75,1.),True,True,True,False,.02,.1,0.,0.)
    plan=PositionPlan(s.binding,1,s,p,q,joints,.02,.1,0,audit)
    actual=replace(s,tcp_q=tuple(map(float,Rotation.from_euler('x',degrees,degrees=True).as_quat())),
        actual_q=joints,task_state='FINISHED')
    assert evaluate_position_arrival(plan,actual,motion_id=7)==expected


def test_pitch_path_reference_is_fixed_and_rejects_twenty_degree_escape():
    from app.robots.finite_position_plan import audit_position_path,FRACTIONS
    s=sample();settings,_,_=fixture();goal=list(s.actual_q);goal[0]+=.0005
    class PitchModel(LinearModel):
        def forward(self,q):
            p,_=super().forward(q)
            return p,tuple(map(float,Rotation.from_euler('x',q[0]/.0005,degrees=True).as_quat()))
    model=PitchModel();fk=tuple(model.forward(tuple(a+(b-a)*f for a,b in zip(s.actual_q,goal))) for f in FRACTIONS)
    audit_position_path(s,goal,fk_samples=fk,settings=settings,model=model,target_q=fk[-1][1],pitch_reference=s.tcp_q)
    reference=tuple(map(float,Rotation.from_euler('x',-20,degrees=True).as_quat()))
    with pytest.raises(ValueError,match='pitch_limit'):
        audit_position_path(s,goal,fk_samples=fk,settings=settings,model=model,target_q=fk[-1][1],pitch_reference=reference)


async def test_pitch_cannot_be_planned_when_backend_does_not_advertise_capability():
    from tests.robots.test_finite_position_adapter import prepared
    a,c,clock,_=await prepared();a.position_pitch_enabled=False
    try:
        with pytest.raises(ValueError,match='pitch_not_authorized'):
            await a.plan_step(PositionIntent('web',1,2,clock.now_ns(),'held',(0.,0.,0.),pitch=1.),authorize=lambda:None)
        assert not c.write_calls
    finally:await a.disconnect()
