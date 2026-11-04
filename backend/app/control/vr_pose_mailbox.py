"""Full-pose transport candidate with injected signal I/O only.

Two banks (240..255), each with a header, micrometre XYZ and quaternion*1e7.
Invalidate the inactive header BEFORE payload mutation, commit it LAST, and
recheck the chosen header after reading. No endpoint factory lives here; the
opt-in native Docker executor owns admission, clock binding and completion.
"""
import asyncio
from dataclasses import dataclass
import math

from app.schemas.messages import Pose

BASE = 240
BANK_SIZE = 8
STAMP_MOD = 2**21
SEQ_MOD = 1024
TTL_MS = 100
TTL_NS = TTL_MS * 1_000_000


def header(stamp_ms, seq):
    if type(stamp_ms) is not int or stamp_ms < 0 or type(seq) is not int or seq < 1:
        raise ValueError('pose_mailbox_header')
    value = (stamp_ms % STAMP_MOD) * SEQ_MOD + seq % SEQ_MOD
    if value == 0:
        raise ValueError('pose_mailbox_reserved_epoch')
    return value


def pack(pose):
    if (not isinstance(pose, Pose) or len(pose.p) != 3 or len(pose.q) != 4
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in (*pose.p, *pose.q))):
        raise ValueError('pose_mailbox_payload')
    if abs(math.hypot(*pose.q) - 1) > 1e-5:
        raise ValueError('pose_mailbox_quaternion')
    scaled = tuple(x * 1e6 for x in pose.p) + tuple(x * 1e7 for x in pose.q)
    if any(not math.isfinite(x) or not -(2**31) <= x < 2**31 for x in scaled):
        raise ValueError('pose_mailbox_range')
    values = tuple(round(x) for x in scaled)
    if any(not -(2**31) <= x < 2**31 for x in values):
        raise ValueError('pose_mailbox_range')
    return values


def unpack(values):
    if len(values) != 7 or any(type(x) is not int or not -(2**31) <= x < 2**31 for x in values):
        raise ValueError('pose_mailbox_payload')
    q = tuple(x / 1e7 for x in values[3:])
    norm = math.hypot(*q)
    if abs(norm - 1) > 1e-5:
        raise ValueError('pose_mailbox_quaternion')
    return Pose(p=tuple(x / 1e6 for x in values[:3]), q=tuple(x / norm for x in q))


@dataclass(frozen=True)
class CapturedPose:
    seq: int
    captured_ns: int  # Host monotonic time; never refreshed by republishing.
    pose: Pose


@dataclass(frozen=True)
class ReceivedPose:
    sequence_mod: int
    captured_ms: int  # Controller time, deliberately distinct from host time.
    pose: Pose


class PoseMailboxWriter:
    def __init__(self, client, lease, *, clock, stamp_for_capture, authorize, bulk=False, before_commit=None, ttl_ms=TTL_MS):
        if type(ttl_ms) is not int or not 1<=ttl_ms<=1000:
            raise ValueError('pose_mailbox_ttl')
        self.ttl_ns=ttl_ms*1_000_000
        self.rpc_timeout_s=ttl_ms/1000
        if type(bulk) is not bool:
            raise ValueError('pose_mailbox_bulk_mode')
        self.bulk = bulk
        self.before_commit = before_commit
        self.client, self.lease = client, lease
        self.token = lease.acquire()
        self.clock, self.stamp_for_capture, self.authorize = clock, stamp_for_capture, authorize
        self.latest = None
        self.last_seq = self.written_seq = 0
        self.last_stamp_ns = self.last_controller_ms = -1
        self.last_written_capture_ns = None
        self.active = 1
        self.opened = self.closed = self.unknown = self.busy = False
        self.read_unknown = False

    def _check(self):
        if self.closed:
            raise ValueError('pose_mailbox_closed')
        try:
            self.lease.require(self.token)
            self.authorize()
        except BaseException:
            self.closed = True
            raise

    async def open(self):
        self._check()
        if self.opened or self.busy:
            raise ValueError('pose_mailbox_already_open')
        self.busy = True
        try:
            for channel in (BASE, BASE + BANK_SIZE):
                try:
                    value = await asyncio.wait_for(self.client.get_signal(channel), self.rpc_timeout_s)
                except BaseException:
                    self.read_unknown = True
                    raise
                self._check()
                if type(value) is not int or value != 0:
                    raise ValueError('pose_mailbox_occupied')
            self.opened = True
        except BaseException:
            self.closed = True
            raise
        finally:
            self.busy = False

    def offer(self, sample):
        self._check()
        if self.last_stamp_ns >= 0 and not 0 <= self.clock() - self.last_stamp_ns < self.ttl_ns:
            self.closed = True
            raise ValueError('pose_mailbox_input_expired')
        if (not self.opened or type(sample) is not CapturedPose or type(sample.seq) is not int
                or sample.seq <= self.last_seq or type(sample.captured_ns) is not int
                or sample.captured_ns // 1_000_000 <= self.last_stamp_ns // 1_000_000):
            raise ValueError('pose_mailbox_input_order')
        if not 0 <= self.clock() - sample.captured_ns < self.ttl_ns:
            self.closed = True
            raise ValueError('pose_mailbox_input_expired')
        payload = pack(sample.pose)  # Snapshot mutable Pose before any await.
        stamp = self.stamp_for_capture(sample.captured_ns)
        word = header(stamp, sample.seq)
        if stamp <= self.last_controller_ms:
            raise ValueError('pose_mailbox_clock_order')
        self.latest = (sample.seq, sample.captured_ns, word, payload)
        self.last_seq, self.last_stamp_ns, self.last_controller_ms = sample.seq, sample.captured_ns, stamp

    def _fresh(self, captured_ns):
        if not 0 <= self.clock() - captured_ns < self.ttl_ns:
            self.closed = True
            raise ValueError('pose_mailbox_input_expired')

    async def _write(self, channel, value, captured_ns, *, bulk=False):
        self._check()
        self._fresh(captured_ns)
        try:
            operation = self.client.set_signals(channel, list(value)) if bulk else self.client.set_signal(channel, value)
            await asyncio.wait_for(operation, self.rpc_timeout_s)
        except BaseException:
            self.unknown = self.closed = True
            raise
        self._check()

    async def publish_latest(self):
        self._check()
        if not self.opened:
            raise ValueError('pose_mailbox_not_open')
        if self.busy:
            raise ValueError('pose_mailbox_write_inflight')
        sample = self.latest
        if sample is None or sample[0] <= self.written_seq:
            return False
        self.busy = True
        try:
            seq, captured_ns, word, payload = sample
            bank = 1 - self.active
            offset = BASE + bank * BANK_SIZE
            await self._write(offset, 0, captured_ns)
            # No payload is visible yet. Coalesce to the freshest complete
            # capture AFTER invalidation, so that RPC wait does not age it.
            self._check()
            seq, captured_ns, word, payload = self.latest
            if self.bulk:
                await self._write(offset + 1, payload, captured_ns, bulk=True)
            else:
                for i, value in enumerate(payload, 1):
                    await self._write(offset + i, value, captured_ns)
            # A newly received sample can confirm the exact encoded pose.
            # Its ORIGINAL capture/sequence may commit the identical payload;
            # a different payload must wait for its own publication.
            self._check()
            if self.latest is not None and self.latest[3] == payload:
                seq, captured_ns, word, payload = self.latest
            if self.before_commit is not None:
                try:
                    self.before_commit()
                except BaseException:
                    self.closed = True
                    raise
            await self._write(offset, word, captured_ns)
            self._fresh(captured_ns)
            self.active, self.written_seq = bank, seq
            self.last_written_capture_ns = captured_ns
            return True
        finally:
            self.busy = False

    def revoke(self):
        # No cancellation, mailbox clearing or lease release: caller must join
        # in-flight work AND observe known controller completion first.
        self.closed = True
        self.latest = None


class PoseMailboxReader:
    def __init__(self, client, *, controller_ms, not_before_ms, authorize, bulk=False):
        if type(bulk) is not bool:
            raise ValueError('pose_mailbox_bulk_mode')
        self.bulk = bulk
        if type(not_before_ms) is not int or not_before_ms < 0:
            raise ValueError('pose_mailbox_epoch')
        self.client, self.controller_ms, self.authorize = client, controller_ms, authorize
        self.not_before_ms = self.last_now = not_before_ms
        self.last_capture_ms = self.last_word = self.last_payload = None
        self.closed = self.busy = False

    def _check(self):
        if self.closed:
            raise ValueError('pose_mailbox_reader_retired')
        self.authorize()
        now = self.controller_ms()
        if (type(now) is not int or now < self.last_now
                or now - self.not_before_ms >= STAMP_MOD // 2):
            raise ValueError('pose_mailbox_clock')
        self.last_now = now
        return now

    def _no_fresh_sample(self):
        now = self._check()
        if self.last_capture_ms is not None and now - self.last_capture_ms + 1 >= TTL_MS:
            raise ValueError('pose_mailbox_input_expired')
        return None

    async def _read(self, channel):
        self._check()
        value = await asyncio.wait_for(self.client.get_signal(channel), .1)
        self._check()
        if type(value) is not int or not -(2**31) <= value < 2**31:
            raise ValueError('pose_mailbox_signal')
        return value

    async def read_latest(self):
        if self.busy:
            raise ValueError('pose_mailbox_read_inflight')
        self.busy = True
        try:
            self._check()
            heads = [await self._read(BASE), await self._read(BASE + BANK_SIZE)]
            now = self._check()
            candidates = []
            for bank, word in enumerate(heads):
                if word < 0:
                    raise ValueError('pose_mailbox_header')
                age = (now - word // SEQ_MOD) % STAMP_MOD
                capture = now - age
                if word and age + 1 < TTL_MS and capture >= self.not_before_ms:
                    candidates.append((capture, bank))
            if not candidates:
                return self._no_fresh_sample()
            if len(candidates) == 2 and candidates[0][0] == candidates[1][0]:
                raise ValueError('pose_mailbox_ambiguous_stamp')
            capture, bank = max(candidates)
            if self.last_capture_ms is not None and capture < self.last_capture_ms:
                raise ValueError('pose_mailbox_target_rollback')
            offset, word = BASE + bank * BANK_SIZE, heads[bank]
            if self.bulk:
                self._check()
                values = await asyncio.wait_for(self.client.get_signals(offset + 1, 7), .1)
                self._check()
                if type(values) not in (tuple, list) or len(values) != 7:
                    raise ValueError('pose_mailbox_payload')
                payload = tuple(values)
            else:
                payload = tuple([await self._read(offset + i) for i in range(1, 8)])
            if await self._read(offset) != word:
                return self._no_fresh_sample()  # Never return mixed fields.
            now = self._check()
            if now - capture + 1 >= TTL_MS:
                raise ValueError('pose_mailbox_input_expired')
            if capture == self.last_capture_ms and (word != self.last_word or payload != self.last_payload):
                raise ValueError('pose_mailbox_target_mutated')
            pose = unpack(payload)
            self.last_capture_ms, self.last_word, self.last_payload = capture, word, payload
            return ReceivedPose(word % SEQ_MOD, capture, pose)
        except BaseException:
            self.closed = True
            raise
        finally:
            self.busy = False
