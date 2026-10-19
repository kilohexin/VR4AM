"""Bounded latest-input ownership for injected offline speedl sequences only."""
from dataclasses import fields

from app.commissioning.native_jog_packet import NativeJogPacket, validate_inflight_input, _integer, _sha


def _signature(packet):
    return tuple((type(v), v) for v in (getattr(packet, f.name) for f in fields(NativeJogPacket)))


class JogStreamGate:
    def __init__(self, *, generation, config_sha256, clock):
        _integer(generation)
        _sha(config_sha256)
        if not callable(clock):
            raise ValueError('native_jog_stream_clock_required')
        self.generation, self.config_sha256, self.clock = generation, config_sha256, clock
        self.started_ns = clock()
        _integer(self.started_ns)
        self.session_deadline_ns = self.started_ns + 1_000_000_000
        self.deadline_ns = self.session_deadline_ns
        self.entries = 0
        self.revoked = False
        self._latest = self._active = None
        self._latest_signature = self._active_signature = None
        self._entered = False
        self._last_sequence = self._last_input_ns = -1

    def _live(self):
        now = self.clock()
        _integer(now)
        if self.revoked or not self.started_ns <= now < self.session_deadline_ns:
            self.revoke()
            raise ValueError('native_jog_stream_closed')
        if now >= self.deadline_ns:
            self.revoke()
            raise ValueError('native_jog_stream_input_expired')
        return now

    def _validate(self, packet, signature):
        now = self._live()
        if type(packet) is not NativeJogPacket or _signature(packet) != signature:
            raise ValueError('native_jog_stream_packet_changed')
        # Revalidate a frozen object at the boundary without modifying it.
        NativeJogPacket(*(getattr(packet, f.name) for f in fields(NativeJogPacket)))
        validate_inflight_input(packet, now_ns=now, generation=self.generation,
            config_sha256=self.config_sha256)

    def offer(self, packet):
        self._live()
        if type(packet) is not NativeJogPacket:
            raise ValueError('native_jog_stream_packet_required')
        signature = _signature(packet)
        self._validate(packet, signature)
        if packet.sequence <= self._last_sequence or packet.input_ns < self._last_input_ns:
            raise ValueError('native_jog_stream_input_not_new')
        self._last_sequence, self._last_input_ns = packet.sequence, packet.input_ns
        self._latest, self._latest_signature = packet, signature
        self.deadline_ns = min(self.session_deadline_ns, packet.expires_ns, packet.input_ns+100_000_000)

    def take_latest(self):
        self._live()
        if self._active is not None or self._latest is None:
            return None
        self._validate(self._latest, self._latest_signature)
        self._active, self._active_signature = self._latest, self._latest_signature
        self._latest = self._latest_signature = None
        self._entered = False
        return self._active

    def require_before_entry(self, packet):
        self._validate(packet, self._active_signature)
        if packet is not self._active or self._entered or self.entries >= 10:
            raise ValueError('native_jog_stream_entry_unavailable')
        if packet.sequence != self._last_sequence:
            raise ValueError('native_jog_stream_superseded')

    def mark_entered(self, packet):
        self.require_before_entry(packet)
        self.entries += 1
        self._entered = True

    def require_inflight(self, packet):
        self._validate(packet, self._active_signature)
        if packet is not self._active or not self._entered:
            raise ValueError('native_jog_stream_inflight_unavailable')

    def finish(self, packet):
        if packet is not self._active:
            raise ValueError('native_jog_stream_completion_owner_changed')
        self._active = self._active_signature = None
        self._entered = False
        if self.entries >= 10:
            self.revoke()

    def revoke(self):
        self.revoked = True
        self._latest = self._latest_signature = None
