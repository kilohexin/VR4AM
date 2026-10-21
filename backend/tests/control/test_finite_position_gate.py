import importlib
import math
import pytest


def api():
    assert importlib.util.find_spec('app.control.finite_position_gate'), 'finite position input gate missing'
    return importlib.import_module('app.control.finite_position_gate')


def fixture():
    m=api(); now=[0]
    binding=m.PositionBinding(object(),1,'a'*64,'b'*64)
    return m,m.PositionGate(binding,clock=lambda:now[0]),now


def offer(m,g,seq=1,kind='held',vector=(1.,0.,0.),stamp=0):
    value=m.PositionIntent('web',1,seq,stamp,kind,vector)
    g.offer(value);return value


def test_release_leaves_no_next_goal():
    m,g,_=fixture();offer(m,g);offer(m,g,3,'release',(0.,)*3)
    with pytest.raises(ValueError):offer(m,g,2)
    assert g.take_latest() is None


def test_release_preserves_inflight_but_forbids_continuation():
    m,g,_=fixture();offer(m,g);p=g.take_latest();g.mark_entered(p)
    offer(m,g,2,'release',(0.,)*3)
    assert g.entered_steps==1 and g.active is p and g.released
    assert g.take_latest() is None
    g.finish();assert g.take_latest() is None


def test_vector_is_normalized_and_zero_is_release():
    m,g,_=fixture();p=offer(m,g,vector=(1.,1.,1.))
    assert math.isclose(math.hypot(*p.vector),1.)
    offer(m,g,2,vector=(0.,)*3);assert g.released and g.take_latest() is None


@pytest.mark.parametrize('bad',[True,float('nan'),float('inf'),'1'])
def test_bad_input_rejected(bad):
    m,g,_=fixture()
    with pytest.raises(ValueError):offer(m,g,vector=(bad,0.,0.))


@pytest.mark.parametrize('age,valid',[(99_000_000,True),(100_000_000,False),(-1,False)])
def test_freshness_boundary(age,valid):
    m,g,now=fixture();offer(m,g);now[0]=age
    if valid:assert g.take_latest() is not None
    else:
        with pytest.raises(ValueError):g.take_latest()


def test_same_direction_updates_do_not_starve_plan_but_reverse_does():
    m,g,_=fixture();offer(m,g);p=g.take_latest();offer(m,g,2)
    g.require_entry(p)
    offer(m,g,3,vector=(-1.,0.,0.))
    with pytest.raises(ValueError):g.require_entry(p)


def test_fresh_real_updates_can_authorize_a_plan_older_than_input_ttl():
    m,g,now=fixture();offer(m,g);p=g.take_latest()
    for seq in range(2,6):
        now[0]+=40_000_000;offer(m,g,seq,stamp=now[0]);g.require_entry(p)
    assert now[0]==160_000_000
    now[0]+=100_000_000
    with pytest.raises(ValueError):g.require_entry(p)


def test_completion_requires_newer_input_after_finish():
    m,g,now=fixture();offer(m,g);p=g.take_latest();g.mark_entered(p)
    now[0]=20;offer(m,g,2,stamp=20);g.finish()
    assert g.take_latest() is None
    now[0]=21;offer(m,g,3,stamp=21);assert g.take_latest() is not None


def test_ten_steps_end_without_restarting_held_session():
    m,g,now=fixture()
    for seq in range(1,11):
        now[0]=seq;offer(m,g,seq,stamp=seq);p=g.take_latest();g.mark_entered(p);g.finish()
    now[0]=11;offer(m,g,11,stamp=11)
    assert g.take_latest() is None and g.entered_steps==10


def test_wrong_binding_and_session_deadline_rejected():
    m,g,now=fixture()
    with pytest.raises(ValueError):g.offer(m.PositionIntent('web',2,1,0,'held',(1.,0.,0.)))
    offer(m,g);now[0]=10_000_000_000
    with pytest.raises(ValueError):g.take_latest()
