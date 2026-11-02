import pytest
from app.control.blended_position_gate import BlendedPositionGate


def rig():
    now=[1_000_000_000];binding=object()
    gate=BlendedPositionGate(binding,(-.1,)*3,(.1,)*3,clock=lambda:now[0])
    return gate,binding,now


def offer(g,b,n,seq=1,d=(0,0,1)):g.offer(b,seq,n[0],d)
def send(g,b,n,p=(0,0,0),ident=1):
    t=g.reserve(b,p,n[0]);assert t is not None
    g.require_entry(b,t,p,n[0]);g.acknowledge(b,t,ident);return t


def test_finished_id_does_not_erase_geometric_backlog():
    g,b,n=rig();offer(g,b,n);send(g,b,n);send(g,b,n,ident=2)
    g.observe(b,(0,0,.003),n[0],finished=(1,))
    assert g.reserve(b,(0,0,.003),n[0]) is None
    t=g.reserve(b,(0,0,.005),n[0]);assert t.position==(0.,0.,.012)


def test_small_direction_noise_keeps_fixed_heading_without_accumulating_turn():
    import math
    g,b,n=rig();offer(g,b,n);send(g,b,n)
    for seq,deg in enumerate((1,-2,3,-4,5),2):
        offer(g,b,n,seq,(math.sin(math.radians(deg)),0,math.cos(math.radians(deg))))
        assert not g.draining and g.latest==(0.,0.,1.)
    offer(g,b,n,7,(math.sin(math.radians(6)),0,math.cos(math.radians(6))))
    assert g.draining


def test_noise_during_reservation_preserves_entry_but_release_still_revokes():
    g,b,n=rig();offer(g,b,n);t=g.reserve(b,(0,0,0),n[0])
    offer(g,b,n,2,(.01,0,1));g.require_entry(b,t,(0,0,0),n[0])
    offer(g,b,n,3,(0,0,0))
    with pytest.raises(ValueError,match='entry_revoked'):g.require_entry(b,t,(0,0,0),n[0])


def test_release_revokes_unsent_and_late_reply_still_drains():
    g,b,n=rig();offer(g,b,n);t=g.reserve(b,(0,0,0),n[0])
    offer(g,b,n,2,(0,0,0))
    with pytest.raises(ValueError,match='entry_revoked'):g.require_entry(b,t,(0,0,0),n[0])
    g.acknowledge(b,t,1)
    assert g.pending==[1] and g.draining
    assert g.reserve(b,(0,0,0),n[0]) is None


def test_reverse_waits_for_known_idle_and_new_fresh_input():
    g,b,n=rig();offer(g,b,n);send(g,b,n)
    offer(g,b,n,2,(0,0,-1))
    g.observe(b,(0,0,.004),n[0],finished=(1,),idle=True,stationary=True)
    assert g.reserve(b,(0,0,.004),n[0]) is None
    for _ in range(4):
        n[0]+=50_000_000
        g.observe(b,(0,0,.004),n[0],idle=True,stationary=True)
    with pytest.raises(ValueError,match='input_expired'):g.reserve(b,(0,0,.004),n[0])
    offer(g,b,n,3,(0,0,-1));t=g.reserve(b,(0,0,.004),n[0]);assert t.position==(0.,0.,0.)


def test_binding_unknown_and_stale_feedback_cannot_submit():
    g,b,n=rig();offer(g,b,n)
    with pytest.raises(ValueError,match='revoked'):g.reserve(object(),(0,0,0),n[0])
    with pytest.raises(ValueError,match='feedback_expired'):g.reserve(b,(0,0,0),n[0]-100_000_000)
    g.revoke()
    with pytest.raises(ValueError,match='revoked'):g.reserve(b,(0,0,0),n[0])


def test_queue_and_workspace_are_bounded():
    g,b,n=rig();offer(g,b,n);send(g,b,n);send(g,b,n,ident=2)
    assert g.reserve(b,(0,0,.008),n[0]) is None
    g,b,n=rig();offer(g,b,n)
    assert g.reserve(b,(0,0,.099),n[0]) is None


def test_duplicate_gap_and_drift_cannot_complete_settling():
    g,b,n=rig();offer(g,b,n);send(g,b,n)
    offer(g,b,n,2,(0,0,-1))
    g.observe(b,(0,0,.004),n[0],finished=(1,),idle=True,stationary=True)
    with pytest.raises(ValueError,match='feedback_expired'):
        g.observe(b,(0,0,.004),n[0],idle=True,stationary=True)
    n[0]+=250_000_000
    g.observe(b,(0,0,.004),n[0],idle=True,stationary=True)
    assert g.draining
    for _ in range(4):
        n[0]+=50_000_000
        g.observe(b,(0,0,.004+(n[0]%1_000_000_000)/1e9*.01),n[0],idle=True,stationary=True)
    assert g.draining


def test_reserved_target_rechecks_input_age_and_unknown_reply_revokes():
    g,b,n=rig();offer(g,b,n);t=g.reserve(b,(0,0,0),n[0])
    n[0]+=100_000_000
    with pytest.raises(ValueError,match='entry_revoked'):
        g.require_entry(b,t,(0,0,0),n[0])
    g.cancel_unsent(t)
    offer(g,b,n,2);t=g.reserve(b,(0,0,0),n[0])
    with pytest.raises(ValueError,match='reply_unknown'):g.acknowledge(b,t,None)
    with pytest.raises(ValueError,match='revoked'):offer(g,b,n,3)
