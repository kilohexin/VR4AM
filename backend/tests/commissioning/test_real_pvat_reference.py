import numpy as np
import pytest

from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel


def rig():
    model=MdhReferenceModel(MdhTool((.01,-.02,.20),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
    return model,JointReference((0.,-.8,1.4,-.6,1.57,0.))


@pytest.mark.parametrize('axis',['z','pitch','spin'])
def test_empty_trial_is_finite_contiguous_and_has_exact_rest_terminal(axis):
    from app.commissioning.real_pvat_reference import build_empty_trial
    model,start=rig();trial=build_empty_trial(model,start,axis=axis)
    assert len(trial.plan.segments)==20 and trial.plan.segments[0].start==start
    assert all(a.end==b.start for a,b in zip(trial.plan.segments,trial.plan.segments[1:]))
    assert trial.plan.segments[-1].end.v+trial.plan.segments[-1].end.a==(0.,)*12
    assert len(trial.plan.checks)==20 and not trial.motion_admitted
    base=model.pose(start.q)
    terminal=model.pose(trial.plan.segments[-1].end.q)
    if axis=='z':assert terminal.p==pytest.approx((base.p[0],base.p[1],base.p[2]+.002),abs=1e-10)
    else:assert np.linalg.norm(np.array(base.p)-terminal.p)<1e-10


def test_dynamic_start_and_unapproved_axes_are_not_silently_rewritten():
    from app.commissioning.real_pvat_reference import build_empty_trial
    model,start=rig()
    with pytest.raises(ValueError,match='quiet'):
        build_empty_trial(model,JointReference(start.q,(.001,0.,0.,0.,0.,0.)))
    with pytest.raises(ValueError,match='axis'):build_empty_trial(model,start,axis='home')


def test_terminal_revalidation_failure_cannot_be_reported_as_a_plan(monkeypatch):
    import app.commissioning.real_pvat_reference as module
    model,start=rig();called=[]
    def refuse(segment,*args,**kwargs):
        called.append(segment)
        assert segment.end.v+segment.end.a==(0.,)*12
        assert kwargs==dict(speed=.05,acceleration=.2)
        raise ValueError('cartesian_reference_corridor')
    monkeypatch.setattr(module,'validate_segment',refuse)
    with pytest.raises(ValueError,match='corridor'):module.build_empty_trial(model,start)
    assert len(called)==1
