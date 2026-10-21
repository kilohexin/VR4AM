"""Bounded actual-feedback exception; no robot or SDK access."""
import importlib
import pytest


def guard():
    name='app.robots.finite_position_acceleration'
    assert importlib.util.find_spec(name),'finite actual acceleration monitor missing'
    return importlib.import_module(name).PositionAccelerationMonitor(.2)


def observe(g,stamp=1,value=.3,*,now=None,joint=0):
    values=[0.]*6;values[joint]=value
    return g.observe(tuple(values),captured_ns=stamp,now_ns=stamp if now is None else now)


def test_one_moderate_sample_is_accepted_but_third_distinct_sample_faults():
    g=guard();observe(g,1);observe(g,20_000_001)
    with pytest.raises(ValueError,match='reported_rate_limit'):observe(g,40_000_001)


@pytest.mark.parametrize('value',[.400001,-.400001,1.])
def test_large_acceleration_is_immediate(value):
    with pytest.raises(ValueError,match='reported_rate_limit'):observe(guard(),value=value)


def test_repeated_validation_of_one_snapshot_is_not_three_samples():
    g=guard()
    for _ in range(5):observe(g,1)
    observe(g,20_000_001)
    with pytest.raises(ValueError,match='reported_rate_limit'):observe(g,40_000_001)


def test_normal_sample_resets_consecutive_count():
    g=guard();observe(g,1);observe(g,20_000_001);observe(g,40_000_001,value=.2)
    observe(g,60_000_001);observe(g,80_000_001)
    with pytest.raises(ValueError,match='reported_rate_limit'):observe(g,100_000_001)


def test_distinct_joint_counts_do_not_hide_a_persistent_joint():
    g=guard();observe(g,1,joint=0);observe(g,20_000_001,joint=1)
    observe(g,40_000_001,joint=1)
    with pytest.raises(ValueError,match='reported_rate_limit'):observe(g,60_000_001,joint=1)


@pytest.mark.parametrize('kind',['stale','future','backwards','changed_duplicate','gap'])
def test_exception_requires_fresh_ordered_adjacent_samples(kind):
    g=guard();observe(g,1)
    with pytest.raises(ValueError):
        if kind=='stale':observe(g,2,now=100_000_002)
        if kind=='future':observe(g,2,now=1)
        if kind=='backwards':observe(g,0)
        if kind=='changed_duplicate':observe(g,1,value=.31)
        if kind=='gap':observe(g,100_000_001)


def test_reset_does_not_carry_old_generation_counts():
    g=guard();observe(g,1);observe(g,20_000_001);g.reset()
    observe(g,40_000_001);observe(g,60_000_001)
    with pytest.raises(ValueError):observe(g,80_000_001)


def test_stricter_configured_profile_is_not_relaxed():
    name='app.robots.finite_position_acceleration'
    assert importlib.util.find_spec(name)
    g=importlib.import_module(name).PositionAccelerationMonitor(.1)
    assert g.instant_limit==.1
    with pytest.raises(ValueError,match='reported_rate_limit'):observe(g,value=.100001)
