"""Finite jog planning is nominal geometry, never a physical stopping proof."""
from dataclasses import FrozenInstanceError, replace
import math

import pytest

from app.commissioning.native_jog_packet import (
    NativeJogLimits, NativeJogPacket, plan_jog, validate_packet,
)

SHA = 'a' * 64
LIMITS = NativeJogLimits(.005, .02, (-1, -1, -1), (1, 1, 1), (.01, .01, .01))


def packet(**changes):
    args = dict(limits=LIMITS, actual_p=(0, 0, 0), snapshot_ns=1_000_000_000,
        now_ns=1_000_000_000, generation=0, sequence=1, input_ns=1_000_000_000,
        input_expires_ns=1_200_000_000, duration_s=.08, config_sha256=SHA)
    args.update(changes)
    return plan_jog(args.pop('direction', (1, 1, 0)), args.pop('strength', 1), **args)


def test_diagonal_speed_is_norm_limited():
    p = packet()
    assert math.hypot(*p.velocity_mps) <= .005
    assert p.velocity_mps[0] == p.velocity_mps[1] == pytest.approx(.0035355339059327372)
    assert p.expires_ns == 1_100_000_000
    assert packet(input_expires_ns=1_050_000_000).expires_ns == 1_050_000_000


@pytest.mark.parametrize('value', [0, None, -.01, True, math.nan, math.inf, .009, .081])
def test_positive_duration_is_explicit(value):
    with pytest.raises(ValueError):
        packet(duration_s=value)


@pytest.mark.parametrize('value', [.01, .08])
def test_duration_endpoints(value):
    assert packet(duration_s=value).duration_s == value


@pytest.mark.parametrize('changes', [dict(direction=(0,0,0)), dict(strength=0)])
def test_no_motion_for_zero_intent(changes):
    assert packet(**changes) is None
    with pytest.raises(ValueError):
        packet(**changes, duration_s=0)


@pytest.mark.parametrize('changes', [
    dict(snapshot_ns=1_000_000_001), dict(snapshot_ns=900_000_000),
    dict(input_ns=1_000_000_001), dict(input_ns=900_000_000),
    dict(input_expires_ns=1_000_000_000), dict(generation=True), dict(sequence=-1),
    dict(now_ns=True), dict(config_sha256='A'*64), dict(config_sha256='a'*63),
    dict(strength=True), dict(strength=1.01), dict(strength=-.1), dict(strength=math.nan),
    dict(direction=(1,2)), dict(direction=(True,0,0)), dict(direction=(math.inf,0,0)),
    dict(actual_p=(0,math.nan,0)),
])
def test_expired_or_future_inputs_rejected(changes):
    with pytest.raises(ValueError):
        packet(**changes)


def test_sdk_arguments_do_not_alias_packet():
    direction = [1,0,0]
    p = packet(direction=direction)
    direction[0] = 20
    a,v,t,frame = p.sdk_arguments()
    assert (a,t) == (.02,.08)
    assert v == dict(x=.005,y=0,z=0,rx=0,ry=0,rz=0)
    assert frame == dict(x=0,y=0,z=0,rx=0,ry=0,rz=0)
    v['x'] = frame['x'] = 99
    assert p.sdk_arguments()[1]['x'] == .005
    assert p.sdk_arguments()[3]['x'] == 0
    with pytest.raises(FrozenInstanceError):
        p.duration_s = .2


def test_nominal_box_includes_explicit_reserve():
    assert packet(direction=(1,0,0), actual_p=(.9896,0,0))
    with pytest.raises(ValueError):
        packet(direction=(1,0,0), actual_p=(.9897,0,0))
    with pytest.raises(ValueError):
        packet(direction=(-1,0,0), actual_p=(.991,0,0))


@pytest.mark.parametrize('changes', [dict(now_ns=1_100_000_000), dict(generation=1),
    dict(config_sha256='b'*64), dict(actual_p=(1,0,0))])
def test_validation_cannot_extend_deadline_or_identity(changes):
    p = packet()
    args = dict(limits=LIMITS, actual_p=(0,0,0), snapshot_ns=1_000_000_000,
        now_ns=1_000_000_000, generation=0, config_sha256=SHA)
    args.update(changes)
    with pytest.raises(ValueError):
        validate_packet(p, **args)
    assert p.expires_ns == 1_100_000_000


@pytest.mark.parametrize('changes', [dict(max_speed_mps=True), dict(max_acceleration_mps2=0),
    dict(tcp_min_m=(1,1,1)), dict(planning_reserve_m=(-1,0,0))])
def test_invalid_limits(changes):
    with pytest.raises(ValueError):
        replace(LIMITS, **changes)


def test_direct_packet_construction_freezes_and_validates():
    p = packet()
    v = [0,.001,0]
    q = replace(p, velocity_mps=v)
    v[1] = 9
    assert q.velocity_mps == (0,.001,0)
    with pytest.raises(ValueError):
        replace(p, duration_s=0)
    with pytest.raises(ValueError):
        validate_packet(replace(p, velocity_mps=(9,0,0)), limits=LIMITS,
            actual_p=(0,0,0), snapshot_ns=1_000_000_000, now_ns=1_000_000_000,
            generation=0, config_sha256=SHA)
