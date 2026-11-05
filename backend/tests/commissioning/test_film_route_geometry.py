import pytest
from app.commissioning.native_jog_mdh import MdhTool
from app.control.cartesian_pvat_reference import MdhReferenceModel


def rig():
    return MdhReferenceModel(MdhTool((.01,-.02,.2),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6),(0.,-.8,1.4,-.6,1.57,0.)


def test_short_route_solves_lift_then_translation_without_inventing_a_lowering():
    from app.commissioning.film_route_geometry import survey
    model,start=rig();r=survey(model,start,'x+',lift_m=.02,travel_m=.02)
    assert r['geometrically_reachable'] and not r['motion_admitted'] and not r['path_verified']
    assert r['phases']==['lift','transfer'] and r['samples']==4
    base=model.pose(start);end=model.pose(r['last_q'])
    assert end.p==pytest.approx((base.p[0]+.02,base.p[1],base.p[2]+.02),abs=1e-7)


def test_unreachable_route_returns_failure_location_and_never_clamps_target():
    from app.commissioning.film_route_geometry import survey
    model,start=rig();r=survey(model,start,'x+',lift_m=.3,travel_m=2.)
    assert not r['geometrically_reachable'] and not r['motion_admitted']
    assert r['requested_lift_m']==.3 and r['requested_travel_m']==2.
    assert r['failure_phase'] in ('lift','transfer') and r['failure_distance_m']>0


@pytest.mark.parametrize('direction,lift,travel',[('z+',.3,.5),('x+',0,.5),('x+',.3,float('nan'))])
def test_invalid_route_is_rejected(direction,lift,travel):
    from app.commissioning.film_route_geometry import survey
    model,start=rig()
    with pytest.raises(ValueError):survey(model,start,direction,lift_m=lift,travel_m=travel)
