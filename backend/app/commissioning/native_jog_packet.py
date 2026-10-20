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


_DIAGNOSTIC_BOUNDARIES = frozenset((
    'site_motion_permission', 'submission_before_model', 'submission_after_model',
    'sdk_lock_before_model', 'sdk_lock_after_model',
    'owned_entry_before_model', 'owned_entry_after_model',
))
_LOCAL_REJECTIONS = {
    'expired or mismatched packet identity': 'native_jog_packet_identity_or_expiry',
    'motion exceeds limits or is neutral': 'native_jog_packet_motion_limits',
    'outside nominal planning box/reserve': 'native_jog_packet_outside_planning_box',
}
_LOCAL_CODES = frozenset((*_LOCAL_REJECTIONS.values(),
    'native_jog_snapshot_stale', 'native_jog_snapshot_future',
    'native_jog_input_stale', 'native_jog_input_future'))


class _LocalValidationError(ValueError):
    """Evidence from a known local call only; retain legacy ValueError text."""
    def __init__(self, message, evidence):
        super().__init__(message)
        self._local_evidence = tuple(evidence.items())


def local_validation_diagnostic(error):
    # Never promote vendor text/attributes, subclasses or arbitrary payloads.
    if type(error) is not _LocalValidationError:
        return None
    entries = getattr(error, '_local_evidence', None)
    if (type(entries) is not tuple or len(entries) != 6
            or any(type(row) is not tuple or len(row) != 2 or type(row[0]) is not str
                for row in entries)):
        return None
    details = dict(entries)
    if (type(details.get('code')) is not str or details['code'] not in _LOCAL_CODES
            or type(details.get('boundary')) is not str
            or details['boundary'] not in _DIAGNOSTIC_BOUNDARIES
            or set(details) != {'schema','code','boundary','checked_ns',
                'snapshot_age_ns','input_age_ns'}
            or type(details['schema']) is not int or details['schema'] != 1
            or any(type(details[k]) is not int or abs(details[k]) > 2**63-1
                for k in ('checked_ns','snapshot_age_ns','input_age_ns'))):
        return None
    return details


def validate_packet_diagnostic(packet, *, boundary, **kwargs):
    """Run the original validator once, using its original sampled timestamp.

    Format diagnostics only after a known local rejection. This wrapper cannot
    authorize a packet, sample a new clock, or renew either freshness lease.
    """
    try:
        return validate_packet(packet, **kwargs)
    except ValueError as error:
        if (type(error) is not ValueError or type(boundary) is not str
                or boundary not in _DIAGNOSTIC_BOUNDARIES):
            raise
        now, stamp = kwargs['now_ns'], kwargs['snapshot_ns']
        if (type(packet) is not NativeJogPacket
                or any(type(v) is not int or not 0 <= v <= 2**63-1
                    for v in (now,stamp,packet.input_ns))):
            raise
        message = str(error)
        code = _LOCAL_REJECTIONS.get(message)
        if message == 'future or stale input/snapshot':
            # Match the original validator's order; do not check new criteria.
            for subject, value in (('snapshot',stamp),('input',packet.input_ns)):
                if not 0 <= now-value < FRESH_NS:
                    code = 'native_jog_'+subject+('_future' if now<value else '_stale')
                    break
        if code is None:
            raise
        evidence = dict(schema=1,code=code,boundary=boundary,checked_ns=now,
            snapshot_age_ns=now-stamp,input_age_ns=now-packet.input_ns)
        raise _LocalValidationError(message,evidence) from None


def validate_inflight_input(packet, *, now_ns, generation, config_sha256):
    """Existing RPC authority only; this cannot admit another native write.

    The entry owner must already have checked feedback freshness and geometry.
    Never renew the original input/expiry or substitute a new snapshot stamp.
    """
    if type(packet) is not NativeJogPacket:
        raise ValueError('native_jog_inflight_packet_required')
    _integer(now_ns);_integer(generation);_sha(config_sha256)
    try:_fresh(packet.input_ns,now_ns)
    except ValueError:
        raise ValueError('native_jog_inflight_input_expired') from None
    if (now_ns>=packet.expires_ns or packet.expires_ns>packet.input_ns+2*FRESH_NS
            or generation!=packet.generation or config_sha256!=packet.config_sha256):
        raise ValueError('native_jog_inflight_input_or_identity_invalid')


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
