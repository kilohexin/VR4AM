"""Immutable finite translation intent; box checks are nominal planning only.

Velocity times duration is NOT a bound on physical travel or stopping distance.
This module conveys no hardware authorization or joint constraint evidence.
"""
from dataclasses import dataclass
import math
import re

FRESH_NS = 100_000_000


def _number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('finite number required')
    return float(value)


def _vector(value):
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError('three finite components required')
    return tuple(_number(v) for v in value)


def _integer(value):
    if type(value) is not int or value < 0:
        raise ValueError('nonnegative integer required')
    return value


def _sha(value):
    if not isinstance(value, str) or re.fullmatch('[0-9a-f]{64}', value) is None:
        raise ValueError('lowercase SHA256 required')


def _duration(value):
    value = _number(value)
    if not .01 <= value <= .08:
        raise ValueError('explicit duration must be 10..80 ms')
    return value


def _fresh(stamp, now):
    _integer(stamp)
    if not 0 <= now - stamp < FRESH_NS:
        raise ValueError('future or stale input/snapshot')


@dataclass(frozen=True)
class NativeJogLimits:
    max_speed_mps: float
    max_acceleration_mps2: float
    tcp_min_m: tuple[float, float, float]
    tcp_max_m: tuple[float, float, float]
    planning_reserve_m: tuple[float, float, float]

    def __post_init__(self):
        for name in ('max_speed_mps', 'max_acceleration_mps2'):
            value = _number(getattr(self, name))
            if value <= 0:
                raise ValueError('positive limits required')
            object.__setattr__(self, name, value)
        for name in ('tcp_min_m', 'tcp_max_m', 'planning_reserve_m'):
            object.__setattr__(self, name, _vector(getattr(self, name)))
        if any(lo >= hi or reserve < 0 for lo,hi,reserve in zip(
                self.tcp_min_m, self.tcp_max_m, self.planning_reserve_m)):
            raise ValueError('invalid nominal planning box/reserve')


@dataclass(frozen=True)
class NativeJogPacket:
    velocity_mps: tuple[float, float, float]
    acceleration_mps2: float
    duration_s: float
    generation: int
    sequence: int
    input_ns: int
    expires_ns: int
    config_sha256: str

    def __post_init__(self):
        object.__setattr__(self, 'velocity_mps', _vector(self.velocity_mps))
        object.__setattr__(self, 'duration_s', _duration(self.duration_s))
        a = _number(self.acceleration_mps2)
        if a <= 0:
            raise ValueError('positive acceleration required')
        object.__setattr__(self, 'acceleration_mps2', a)
        for name in ('generation','sequence','input_ns','expires_ns'):
            _integer(getattr(self, name))
        if self.expires_ns <= self.input_ns:
            raise ValueError('expired packet')
        _sha(self.config_sha256)

    def sdk_arguments(self):
        return (self.acceleration_mps2,
            dict(zip(('x','y','z','rx','ry','rz'), (*self.velocity_mps,0.,0.,0.))),
            self.duration_s, dict.fromkeys(('x','y','z','rx','ry','rz'), 0.))


def validate_packet(packet, *, limits, actual_p, snapshot_ns, now_ns,
                    generation, config_sha256):
    if type(packet) is not NativeJogPacket or type(limits) is not NativeJogLimits:
        raise ValueError('validated packet/limits required')
    _integer(now_ns)
    _integer(generation)
    _sha(config_sha256)
    _fresh(snapshot_ns, now_ns)
    _fresh(packet.input_ns, now_ns)
    if (now_ns >= packet.expires_ns or packet.expires_ns > packet.input_ns + 2*FRESH_NS
            or generation != packet.generation or config_sha256 != packet.config_sha256):
        raise ValueError('expired or mismatched packet identity')
    speed = math.hypot(*packet.velocity_mps)
    if not 0 < speed <= limits.max_speed_mps or packet.acceleration_mps2 > limits.max_acceleration_mps2:
        raise ValueError('motion exceeds limits or is neutral')
    for p,v,lo,hi,reserve in zip(_vector(actual_p), packet.velocity_mps,
            limits.tcp_min_m, limits.tcp_max_m, limits.planning_reserve_m):
        end = p + v * packet.duration_s
        if min(p,end) - reserve < lo or max(p,end) + reserve > hi:
            raise ValueError('outside nominal planning box/reserve')


def plan_jog(direction, strength, *, limits, actual_p, snapshot_ns, now_ns,
             generation, sequence, input_ns, input_expires_ns, duration_s, config_sha256):
    direction = _vector(direction)
    strength = _number(strength)
    duration_s = _duration(duration_s)
    actual_p = _vector(actual_p)
    for value in (now_ns, generation, sequence, input_expires_ns):
        _integer(value)
    _sha(config_sha256)
    _fresh(input_ns, now_ns)
    _fresh(snapshot_ns, now_ns)
    if type(limits) is not NativeJogLimits or not 0 <= strength <= 1 or input_expires_ns <= now_ns:
        raise ValueError('invalid intent/limits/deadline')
    norm = math.hypot(*direction)
    if not math.isfinite(norm):
        raise ValueError('direction magnitude overflow')
    if norm == 0 or strength == 0:
        return None
    velocity = tuple((v/norm) * (strength*limits.max_speed_mps) for v in direction)
    # Roundoff must not make a diagonal exceed the configured norm.
    magnitude = math.hypot(*velocity)
    if magnitude > limits.max_speed_mps:
        velocity = tuple(v * math.nextafter(limits.max_speed_mps/magnitude, 0.) for v in velocity)
    packet = NativeJogPacket(velocity, limits.max_acceleration_mps2, duration_s,
        generation, sequence, input_ns, min(input_expires_ns, now_ns+FRESH_NS), config_sha256)
    validate_packet(packet, limits=limits, actual_p=actual_p, snapshot_ns=snapshot_ns,
        now_ns=now_ns, generation=generation, config_sha256=config_sha256)
    return packet
