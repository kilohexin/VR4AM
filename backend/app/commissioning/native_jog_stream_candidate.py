"""Loopback-only protocol continuation, NOT physical continuous-motion admission.

The existing theoretical guard still requires zero reported initial rates.
MOVING/known-ID synthetic feedback exercises ownership, not a moving robot.
"""
import asyncio

from app.commissioning.native_jog_model_guard import ModelJogRegionGuard
from app.commissioning.native_jog_packet import NativeJogLimits, validate_packet, _sha
from app.commissioning.native_jog_stream_gate import JogStreamGate
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.lebai_motion_calls import OwnedMotionCalls
from app.schemas.messages import BackendState


def _snapshot_key(snapshot):
    vectors = (snapshot.actual_q, snapshot.actual_qd, snapshot.actual_qdd,
        snapshot.target_q, snapshot.target_qd, snapshot.target_qdd,
        snapshot.actual_tcp.p, snapshot.actual_tcp.q, snapshot.target_tcp.p, snapshot.target_tcp.q,
        snapshot.tcp_setting.p, snapshot.tcp_setting.q)
    return (snapshot.captured_ns, snapshot.read_requested_ns, snapshot.read_completed_ns,
        snapshot.raw_robot_state, snapshot.robot_state, snapshot.estop, snapshot.raw_running_motion,
        snapshot.running_motion, snapshot.motion_state,
        *(tuple((type(v), v) for v in vector) for vector in vectors))


class LoopbackJogStreamAdapter(RealLebaiAdapter):
    _stop_blocking_motion_methods = RealLebaiAdapter._stop_blocking_motion_methods | {'speedl'}
    _immediate_stop_latch_methods = RealLebaiAdapter._immediate_stop_latch_methods | {'speedl'}

    def __init__(self, settings, *, limits, config_sha256, client_factory, clock, sleep,
                 event_callback, model_region_guard, backend_label='LEBAI_FAKE'):
        if (settings.ip != '127.0.0.1' or settings.mode != 'control'
                or not callable(client_factory) or backend_label != 'LEBAI_FAKE'
                or type(limits) is not NativeJogLimits or type(model_region_guard) is not ModelJogRegionGuard):
            raise ValueError('native_jog_stream_explicit_loopback_model_required')
        _sha(config_sha256)
        model_region_guard.require_binding(settings, limits, config_sha256)
        super().__init__(settings, client_factory=client_factory, clock=clock,
            pvat_clock=clock, sleep=sleep, event_callback=event_callback, backend_label=backend_label)
        self._stream_settings, self._stream_limits, self._stream_sha = settings, limits, config_sha256
        self._stream_model = self._bound_stream_model = model_region_guard
        self._motion_calls = OwnedMotionCalls(verify_completion_authority=True)
        self._gate = None
        self._stream_client = None
        self._connected_once = False
        self._dispatching = False
        self._current_packet = None
        self._entered = False
        self._known_ids = set()
        self.native_jog_entries = 0
        self._wake = asyncio.Event()
        self._watchdog = self._stream_stop_task = None
        self.stream_stop_error = None
        self.bind_motion_authority(self._stream_authorized)

    def _binding(self):
        if (self.settings is not self._stream_settings or self.settings.ip != '127.0.0.1'
                or self._stream_model is not self._bound_stream_model):
            raise BackendCommandError('native_jog_stream_binding_changed')
        self._stream_model.require_binding(self.settings, self._stream_limits, self._stream_sha)

    async def connect(self):
        self._binding()
        if self._connected_once:
            raise BackendCommandError('native_jog_stream_connection_consumed')
        self._connected_once = True
        await super().connect()
        self._stream_client = self._client
        self._gate = JogStreamGate(generation=self._pump.generation,
            config_sha256=self._stream_sha, clock=self._clock)
        self._watchdog = asyncio.create_task(self._watch_deadline(), name='loopback-jog-input-deadline')

    def _stream_authorized(self):
        try:
            self._binding()
            if self._gate is None or self._client is not self._stream_client:
                return False
            if self._current_packet is None:
                return not self._gate.revoked
            if self._entered:
                self._gate.require_inflight(self._current_packet)
            else:
                self._gate.require_before_entry(self._current_packet)
            return True
        except (ValueError, BackendCommandError, TypeError, OverflowError):
            return False

    def offer_jog(self, packet):
        try:
            self._binding()
            if self._gate is None:
                raise ValueError('native_jog_stream_not_connected')
            self._gate.offer(packet)
            self._wake.set()
        except (ValueError, BackendCommandError, TypeError, OverflowError) as error:
            self._request_stop(StopReason.GRIP_RELEASED)
            raise BackendCommandError('native_jog_stream_input_rejected') from error

    def _request_stop(self, reason):
        if self._gate is not None:
            self._gate.revoke()
        self._wake.set()
        if self.native_jog_entries and self._stream_stop_task is None:
            async def stop():
                try:
                    await super(LoopbackJogStreamAdapter, self).stop(reason)
                except BaseException as error:
                    self.stream_stop_error = type(error).__name__
                    raise
            self._stream_stop_task = asyncio.create_task(stop(), name='loopback-jog-original-stop')
            self._stream_stop_task.add_done_callback(lambda t: None if t.cancelled() else t.exception())
        return self._stream_stop_task

    async def revoke_stream(self, reason):
        task = self._request_stop(reason)
        if task is not None:
            await asyncio.shield(task)

    async def stop(self, reason):
        await self.revoke_stream(reason)

    async def _watch_deadline(self):
        while not self._gate.revoked:
            self._wake.clear()
            remaining = (self._gate.deadline_ns-self._clock())/1e9
            if remaining <= 0:
                self._request_stop(StopReason.GRIP_RELEASED)
                return
            try:
                await asyncio.wait_for(self._wake.wait(), timeout=remaining)
            except TimeoutError:
                pass

    def _check_stream_snapshot(self, snapshot, owned_motion_state):
        self._check_submission_ready()
        if self._motion_calls.unknown_methods or self._snapshot_calls.unknown_methods:
            raise BackendCommandError('native_jog_stream_result_unknown')
        if self._latched_fault is not None:
            raise BackendCommandError(self._latched_fault)
        if not self._known_ids:
            preflight = self._preflight_from_snapshot(snapshot)
            if not preflight.ready:
                raise BackendCommandError(preflight.reason or 'preflight_not_ready')
        else:
            # Never rewrite the real snapshot to trick an IDLE-only preflight.
            if snapshot.raw_robot_state in ('MOVING', 7):
                if (snapshot.robot_state is not BackendState.MOVING
                        or snapshot.running_motion not in self._known_ids or owned_motion_state != 'RUNNING'):
                    raise BackendCommandError('native_jog_stream_foreign_motion')
            elif snapshot.raw_robot_state in ('IDLE', 5):
                if (snapshot.robot_state is not BackendState.IDLE or snapshot.running_motion is not None
                        or snapshot.raw_running_motion not in (None, 0, *self._known_ids)):
                    raise BackendCommandError('native_jog_stream_foreign_motion')
            else:
                raise BackendCommandError('native_jog_stream_state_not_allowed')
            if (snapshot.estop is not None or not self._tcp_matches(snapshot.tcp_setting)
                    or not self._joints_inside_limits(snapshot.actual_q)
                    or not self._tcp_inside_startup_envelope(snapshot.actual_tcp)
                    or not self._capabilities.control_ready
                    or self._cartesian_singularity_guarded(snapshot.actual_q)):
                raise BackendCommandError('native_jog_stream_preflight_rejected')

    async def dispatch_latest(self):
        if self._dispatching:
            return None
        if self._gate is None:
            raise BackendCommandError('native_jog_stream_not_connected')
        self._dispatching = True
        packet = None
        entered = False
        try:
            packet = self._gate.take_latest()
            if packet is None:
                return None
            self._current_packet, self._entered = packet, False
            snapshot = await self._read_snapshot(emit_kinematics=False, read_purpose='motion')
            client, owner, read = self._client, self._snapshot_calls, self._latest_snapshot_read
            generation, key = self._pump.generation, _snapshot_key(snapshot)
            owned_motion_state = snapshot.motion_state
            if snapshot.raw_robot_state in ('MOVING', 7) and snapshot.running_motion in self._known_ids:
                # The original snapshot reader resolves IDs only in settled
                # states. Own this additional query; do not forge its snapshot.
                def read_revoked():
                    return (owner.closed or owner is not self._snapshot_calls
                        or client is not self._client or generation != self._pump.generation
                        or not self._stream_authorized())
                owner.active += 1
                try:
                    state = await owner.call('stream_motion_state',
                        lambda: client.get_motion_state(snapshot.running_motion), .05, revoked=read_revoked)
                    owned_motion_state = state.strip().upper() if isinstance(state, str) else None
                finally:
                    owner.active -= 1

            def current():
                self._binding()
                self._gate.require_before_entry(packet)
                if (client is not self._stream_client or client is not self._client
                        or generation != self._gate.generation or not self._pump.is_current(generation)
                        or self._stop_lock.locked() or self._snapshot is not snapshot
                        or self._latest_snapshot_read is not read or owner is not self._snapshot_calls
                        or owner.closed or owner.active or read is None or read.owner is not owner
                        or read.client is not client or read.generation != generation
                        or read.snapshot is not snapshot or _snapshot_key(snapshot) != key):
                    raise BackendCommandError('native_jog_stream_snapshot_owner_changed')
                self._check_known_snapshot_fault(client, generation)
                self._check_stream_snapshot(snapshot, owned_motion_state)
                validate_packet(packet, limits=self._stream_limits, actual_p=snapshot.actual_tcp.p,
                    snapshot_ns=snapshot.captured_ns, now_ns=self._clock(), generation=generation,
                    config_sha256=self._stream_sha)
                self._stream_model.validate(packet, snapshot, settings=self.settings,
                    limits=self._stream_limits, config_sha256=self._stream_sha)
                self._gate.require_before_entry(packet)
                validate_packet(packet, limits=self._stream_limits, actual_p=snapshot.actual_tcp.p,
                    snapshot_ns=snapshot.captured_ns, now_ns=self._clock(), generation=generation,
                    config_sha256=self._stream_sha)

            current()
            self._begin_motion_epoch()
            async with self._sdk_lock:
                current()
                async def operation():
                    nonlocal entered
                    current()
                    a, v, duration, frame = packet.sdk_arguments()
                    self._gate.mark_entered(packet)
                    self._entered = entered = True
                    self.native_jog_entries += 1
                    self._motion_write_stop_required = True
                    value = await client.speedl(a, v, duration, frame)
                    if type(value) is not int or value <= 0 or value in self._known_ids:
                        raise BackendCommandError('invalid_native_jog_motion_id')
                    return value
                try:
                    value = await self._await_motion_call('speedl', operation, .06,
                        generation=generation, entered=lambda: entered)
                except TimeoutError:
                    raise BackendCommandError('sdk_timeout:speedl') from None
            self._known_ids.add(value)
            self._motion_accepted = True
            self._gate.finish(packet)
            packet = None
            if self._gate.revoked:
                await self.revoke_stream(StopReason.GRIP_RELEASED)
            return value
        except BaseException as error:
            # A displaced, proven-unentered input is consumed, not retried.
            superseded = (isinstance(error, ValueError) and str(error) == 'native_jog_stream_superseded'
                and not entered and not self._motion_calls.unknown_methods and not self._gate.revoked)
            if superseded:
                return None
            task = self._request_stop(StopReason.GRIP_RELEASED)
            if task is not None:
                try:
                    await asyncio.shield(task)
                except BackendCommandError:
                    pass  # Preserve the original adapter latch and primary failure.
            if isinstance(error, asyncio.CancelledError):
                raise
            if isinstance(error, (BackendCommandError, ValueError)):
                raise
            raise BackendCommandError('native_jog_stream_dispatch_failed') from error
        finally:
            if packet is not None and self._gate._active is packet:
                self._gate.finish(packet)
            self._current_packet, self._entered = None, False
            self._dispatching = False

    async def disconnect(self):
        self._request_stop(StopReason.SHUTDOWN)
        if self._watchdog is not None:
            await self._watchdog
        try:
            if self._stream_stop_task is not None:
                try:
                    await asyncio.shield(self._stream_stop_task)
                except BackendCommandError:
                    pass
        finally:
            await super().disconnect()

    async def submit_tcp(self, *args, **kwargs):
        raise BackendCommandError('native_jog_stream_candidate_only')

    submit_catch_up = submit_tcp
    prepare = submit_tcp
    home = submit_tcp
    set_gripper = submit_tcp
