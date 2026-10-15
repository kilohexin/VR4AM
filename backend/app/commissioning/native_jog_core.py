"""Internal one-submission ownership shared by explicit diagnostic adapters."""
from collections.abc import Callable
from app.commissioning.native_jog_packet import validate_packet
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter
from app.robots.lebai_motion_calls import OwnedMotionCalls

class _SingleJogSubmission(RealLebaiAdapter):
    _stop_blocking_motion_methods = RealLebaiAdapter._stop_blocking_motion_methods | {'speedl'}
    _immediate_stop_latch_methods = RealLebaiAdapter._immediate_stop_latch_methods | {'speedl'}

    def _initialize_jog(self, settings, limits, config_sha256, model_region_guard):
        # Strict result publication is opt-in here; retain legacy PVAT/claw
        # classification when a completed RPC races the start of a stop.
        self._motion_calls = OwnedMotionCalls(verify_completion_authority=True)
        self._jog_settings = settings
        self._jog_limits = limits
        self._jog_sha = config_sha256
        self._jog_consumed = False
        self.native_jog_entries = 0
        self._jog_authorize = None
        self._jog_revoked = False
        self._model_region_guard = self._bound_model_region_guard = model_region_guard
        # Feed this candidate's own permission into the existing bounded owner,
        # including while native work is pending. This binding is permanent.
        self.bind_motion_authority(self._submission_authorized)

    def _submission_authorized(self):
        if self._jog_revoked:
            return False
        try:
            valid = callable(self._jog_authorize) and self._jog_authorize() is None
        except Exception:
            valid = False
        if not valid:
            self._jog_revoked = True
        return valid

    def _require_model_binding(self):
        if self._model_region_guard is not self._bound_model_region_guard:
            raise BackendCommandError('native_jog_region_guard_changed')
        if self._model_region_guard is not None:
            self._model_region_guard.require_binding(self.settings, self._jog_limits, self._jog_sha)

    async def submit_jog(self, packet, *, authorize: Callable[[], None]) -> int:
        if self._jog_consumed:
            raise BackendCommandError('native_jog_submission_consumed')
        self._jog_consumed = True  # Rejection, cancellation and failure all consume it.
        self._jog_authorize = authorize
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
            if not self._submission_authorized():
                raise BackendCommandError('native_jog_authority_revoked')
            if self.settings is not self._jog_settings:
                raise BackendCommandError('loopback_configuration_changed')
            self._require_model_binding()
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
            if self._model_region_guard is not None:
                self._model_region_guard.validate(packet, snapshot, settings=self.settings,
                    limits=self._jog_limits, config_sha256=self._jog_sha)
                # Local model work cannot renew the input or snapshot lease.
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
