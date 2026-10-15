"""One injected, IPv4-loopback-only speedl candidate; no production selector.

No continuous mode: native_jog_joint_bounds_unverified. This adapter supplies
neither physical stopping evidence nor a real robot execution entry.
"""
from collections.abc import Callable

from app.commissioning.native_jog_packet import NativeJogLimits, validate_packet, _sha
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter


class NativeJogCandidateAdapter(RealLebaiAdapter):
    _stop_blocking_motion_methods = RealLebaiAdapter._stop_blocking_motion_methods | {'speedl'}
    _immediate_stop_latch_methods = RealLebaiAdapter._immediate_stop_latch_methods | {'speedl'}

    def __init__(self, settings, *, limits: NativeJogLimits, config_sha256: str,
                 client_factory, clock, sleep, event_callback):
        if settings.ip != '127.0.0.1' or not callable(client_factory):
            raise ValueError('explicit injected IPv4 loopback factory required')
        _sha(config_sha256)
        if (type(limits) is not NativeJogLimits
                or limits.max_speed_mps > settings.control.max_tcp_speed_mps
                or limits.max_acceleration_mps2 > settings.control.max_tcp_acceleration_mps2
                or any(lo < allowed_lo or hi > allowed_hi for lo,hi,allowed_lo,allowed_hi in zip(
                    limits.tcp_min_m, limits.tcp_max_m,
                    settings.startup_tcp_min_m, settings.startup_tcp_max_m))):
            raise ValueError('candidate limits exceed configured limits')
        super().__init__(settings, client_factory=client_factory, clock=clock,
            pvat_clock=clock, sleep=sleep, event_callback=event_callback, backend_label='LEBAI_FAKE')
        self._jog_settings = settings
        self._jog_limits = limits
        self._jog_sha = config_sha256
        self._jog_consumed = False
        self.native_jog_entries = 0

    async def connect(self):
        if self.settings is not self._jog_settings or self.settings.ip != '127.0.0.1':
            raise BackendCommandError('loopback_configuration_changed')
        await super().connect()

    async def submit_jog(self, packet, *, authorize: Callable[[], None]) -> int:
        if self._jog_consumed:
            raise BackendCommandError('native_jog_submission_consumed')
        self._jog_consumed = True  # Rejection, cancellation and failure all consume it.
        self._check_submission_ready()
        self._begin_motion_epoch()
        client, snapshot = self._client, self._snapshot
        read, owner = self._latest_snapshot_read, self._snapshot_calls
        generation = self._pump.generation
        if read is None or snapshot is None:
            raise BackendCommandError('robot_state_stale')
        sequence, completion = read.sequence, snapshot.read_completed_ns
        entered = False

        def require_current():
            if not callable(authorize) or authorize() is not None:
                raise BackendCommandError('native_jog_authority_invalid')
            if self.settings is not self._jog_settings:
                raise BackendCommandError('loopback_configuration_changed')
            self._check_submission_ready()
            if (client is not self._client or generation != self._pump.generation
                    or not self._pump.is_current(generation) or self._stop_lock.locked()
                    or self._snapshot is not snapshot or self._latest_snapshot_read is not read
                    or read.owner is not owner or owner is not self._snapshot_calls or owner.closed
                    or read.client is not client or read.generation != generation
                    or read.snapshot is not snapshot or read.sequence != sequence
                    or snapshot.read_completed_ns != completion):
                raise BackendCommandError('native_jog_snapshot_or_owner_changed')
            self._check_known_snapshot_fault(client, generation)
            result = self._preflight_from_snapshot(snapshot)
            if not result.ready:
                raise BackendCommandError(result.reason or 'preflight_not_ready')
            validate_packet(packet, limits=self._jog_limits, actual_p=snapshot.actual_tcp.p,
                snapshot_ns=snapshot.captured_ns, now_ns=self._clock(),
                generation=generation, config_sha256=self._jog_sha)

        require_current()
        async with self._sdk_lock:
            require_current()

            async def operation():
                nonlocal entered
                require_current()  # Inside owned task, at actual SDK boundary.
                a,v,t,frame = packet.sdk_arguments()
                entered = True
                self.native_jog_entries += 1
                self._motion_write_stop_required = True
                value = await client.speedl(a,v,t,frame)
                if type(value) is not int or value <= 0:
                    raise BackendCommandError('invalid_native_jog_motion_id')
                return value

            try:
                return await self._await_motion_call('speedl', operation, .06,
                    generation=generation, entered=lambda:entered)
            except TimeoutError:
                raise BackendCommandError('sdk_timeout:speedl') from None
            except (BackendCommandError, ValueError):
                raise
            except Exception:
                raise BackendCommandError('sdk_call_failed:speedl') from None

    async def submit_tcp(self, *args, **kwargs):
        raise BackendCommandError('native_jog_candidate_only')

    submit_catch_up = submit_tcp
    home = submit_tcp
    prepare = submit_tcp
    set_gripper = submit_tcp
