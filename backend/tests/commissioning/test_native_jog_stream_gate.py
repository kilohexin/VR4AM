import importlib
from dataclasses import replace

import pytest

from app.commissioning.native_jog_packet import NativeJogPacket

SHA = 'a'*64


def gate():
    try:
        cls = importlib.import_module('app.commissioning.native_jog_stream_gate').JogStreamGate
    except ModuleNotFoundError:
        pytest.fail('held-input gate missing')
    clock = [1_000_000_000]
    return cls(generation=2, config_sha256=SHA, clock=lambda: clock[0]), clock


def packet(seq, stamp=1_000_000_000):
    return NativeJogPacket((0., 0., .005), .02, .08, 2, seq, stamp, stamp+100_000_000, SHA)


def test_latest_only_and_each_input_consumed_once():
    g, _ = gate()
    for i in (1, 2, 3):
        g.offer(packet(i))
    selected = g.take_latest()
    assert selected.sequence == 3
    g.mark_entered(selected)
    g.finish(selected)
    assert g.take_latest() is None
    assert g.entries == 1


def test_new_input_replaces_waiting_packet_without_mutating_its_deadline():
    g, _ = gate()
    old = packet(1)
    g.offer(old)
    assert g.take_latest() is old
    g.offer(packet(2))
    with pytest.raises(ValueError, match='superseded'):
        g.require_before_entry(old)
    assert old.input_ns == 1_000_000_000 and old.expires_ns == 1_100_000_000
    g.finish(old)
    assert g.take_latest().sequence == 2
    assert g.entries == 0


def test_new_input_does_not_renew_inflight_packet_or_admit_second_write():
    g, clock = gate()
    old = packet(1)
    g.offer(old)
    g.take_latest()
    g.mark_entered(old)
    clock[0] += 50_000_000
    g.offer(packet(2, clock[0]))
    g.require_inflight(old)
    assert g.take_latest() is None
    clock[0] += 50_000_000
    with pytest.raises(ValueError, match='expired'):
        g.require_inflight(old)


@pytest.mark.parametrize('change', [dict(sequence=1), dict(sequence=0), dict(generation=3),
    dict(config_sha256='b'*64), dict(input_ns=999_999_999), dict(input_ns=1_000_000_001)])
def test_invalid_offer_does_not_replace_valid_identity(change):
    g, _ = gate()
    g.offer(packet(1))
    with pytest.raises(ValueError):
        g.offer(replace(packet(2), **change))


@pytest.mark.parametrize('phase', ['queued', 'entered'])
def test_revoke_closes_every_phase_permanently(phase):
    g, _ = gate()
    p = packet(1)
    g.offer(p)
    g.take_latest()
    if phase == 'entered':
        g.mark_entered(p)
    g.revoke()
    with pytest.raises(ValueError):
        g.require_inflight(p) if phase == 'entered' else g.require_before_entry(p)
    with pytest.raises(ValueError):
        g.offer(packet(2))


def test_tenth_packet_can_finish_but_eleventh_cannot_enter():
    g, clock = gate()
    for i in range(1, 11):
        p = packet(i, clock[0])
        g.offer(p)
        assert g.take_latest() is p
        g.mark_entered(p)
        g.require_inflight(p)
        g.finish(p)
        clock[0] += 10_000_000
    assert g.entries == 10 and g.revoked
    with pytest.raises(ValueError):
        g.offer(packet(11, clock[0]))


def test_one_second_is_a_fixed_session_limit():
    g, clock = gate()
    clock[0] += 990_000_000
    p = packet(1, clock[0])
    g.offer(p)
    g.take_latest()
    clock[0] += 10_000_000
    with pytest.raises(ValueError, match='closed'):
        g.mark_entered(p)


def test_queued_expiry_and_mutation_cannot_be_reused():
    g, clock = gate()
    p = packet(1)
    g.offer(p)
    selected = g.take_latest()
    object.__setattr__(p, 'velocity_mps', (0., 0., .004))
    with pytest.raises(ValueError, match='changed'):
        g.mark_entered(selected)
    g, clock = gate()
    g.offer(packet(1))
    clock[0] += 100_000_000
    with pytest.raises(ValueError, match='expired'):
        g.take_latest()


@pytest.mark.parametrize('field,value', [('sequence', True), ('velocity_mps', (float('nan'), 0., .005)),
    ('acceleration_mps2', False), ('duration_s', 0.), ('duration_s', .6)])
def test_mutated_packet_cannot_enter_gate_as_valid_input(field, value):
    g, _ = gate()
    p = packet(1)
    object.__setattr__(p, field, value)
    with pytest.raises(ValueError):
        g.offer(p)
