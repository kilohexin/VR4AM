"""One separately scoped PAUSED stop_move. Never grants motion readiness."""
import asyncio
import copy
from dataclasses import replace
import math
import time

from app.commissioning.idle_stop_baseline import AdmissionRejected, StopOnlyClient, validate_position_continuity
from app.commissioning.stop_observation import observe_after_stop
from app.commissioning.readonly_kinematics import _explicit_clear_estop
from app.robots.base import StopReason
from app.robots.lebai_codec import estop_fault
from app.robots.lebai_adapter import RealLebaiAdapter
from app.schemas.messages import Pose

READ_TIMEOUT_S = .2
READ_BUDGET_S = 15.
MAX_READ_ENTRIES = 512


class BoundedStopOnlyClient(StopOnlyClient):
    """Own native read tasks; a local timeout never means remote cancellation."""

    def __init__(self, client, permission, read_authorize, owner):
        super().__init__(client, permission)
        self.read_authorize, self.owner = read_authorize, owner
        self.read_deadline_ns = time.monotonic_ns() + int(READ_BUDGET_S * 1e9)
        self.read_gate_closed = False
        self.read_remote_result_unknown = False
        self.raw_estop_clear = False
        self.read_calls = []
        self.pending_read = None

    def read_guard(self):
        self.read_authorize()
        if (self.read_gate_closed or self.owner is None or self.owner.done() or self.owner.cancelling()
                or time.monotonic_ns() >= self.read_deadline_ns):
            raise AdmissionRejected('paused_cleanup_read_permission_revoked')

    def __getattr__(self, name):
        method = getattr(self.client, name)
        if not callable(method):
            raise AttributeError(name)
        if name not in self.READS:
            return super().__getattr__(name)

        async def bounded(*args, **kwargs):
            return await self.read(name, lambda: method(*args, **kwargs))
        return bounded

    async def is_connected(self):
        return await self.read('is_connected', self.client.is_connected) is True

    async def read(self, name, invoke):
        self.read_guard()
        if (name not in self.READS or len(self.read_calls) >= MAX_READ_ENTRIES
                or self.pending_read is not None and not self.pending_read.done()):
            self.read_gate_closed = True
            raise AdmissionRejected('paused_cleanup_read_budget_or_pending')
        item = dict(method=name, started_ns=time.monotonic_ns(), entered=False,
                    outcome='not_started', remote_result_unknown=False)
        self.read_calls.append(item)
        deadline = min(self.read_deadline_ns, item['started_ns'] + int(READ_TIMEOUT_S * 1e9))

        async def query():
            self.read_guard()
            if time.monotonic_ns() >= deadline:
                raise AdmissionRejected('paused_cleanup_read_expired_before_entry')
            item['entered'] = True
            # Wrapping the await supports SDK Futures as well as coroutines.
            raw = await invoke()
            return raw, time.monotonic_ns()

        task = self.pending_read = asyncio.create_task(query(), name='paused-cleanup-' + name)
        def consume(done):
            if not done.cancelled():
                done.exception()
        task.add_done_callback(consume)
        try:
            done, _ = await asyncio.wait({task}, timeout=max(0., (deadline - time.monotonic_ns()) / 1e9))
            if not done:
                item['outcome'] = 'timeout'
                raise AdmissionRejected('paused_cleanup_read_timeout')
            raw, completed = task.result()
            item['native_completed_ns'] = completed
            self.read_guard()
            if completed >= deadline or time.monotonic_ns() >= deadline:
                item['outcome'] = 'late_return'
                raise AdmissionRejected('paused_cleanup_read_late_return')
            if name == 'get_estop_reason':
                self.raw_estop_clear = _explicit_clear_estop(raw)
                if not self.raw_estop_clear and estop_fault(raw) is None:
                    raise AdmissionRejected('paused_cleanup_ambiguous_estop')
                item['raw_estop'] = copy.deepcopy(raw)
            item['outcome'] = 'returned'
            return copy.deepcopy(raw)
        except BaseException as error:
            self.read_gate_closed = True
            item['remote_result_unknown'] = item['entered'] and not task.done()
            self.read_remote_result_unknown |= item['remote_result_unknown']
            if isinstance(error, asyncio.CancelledError):
                item['outcome'] = 'cancelled'
            elif item['outcome'] == 'not_started':
                item['outcome'] = 'error' if item['entered'] else 'not_started'
            if not task.done() and not task.cancelling():
                task.cancel()  # Do not await cancellation acknowledgment or a late payload.
            raise
        finally:
            item['completed_ns'] = time.monotonic_ns()

    def require_clear_read_gate(self):
        self.read_guard()
        if not self.raw_estop_clear:
            raise AdmissionRejected('paused_cleanup_raw_estop_not_clear')


class PausedStopOnlyAdapter(RealLebaiAdapter):
    def _preflight_from_snapshot(self, snapshot, **kwargs):
        result = super()._preflight_from_snapshot(snapshot, **kwargs)
        return replace(result, ready=False, reason=result.reason or 'paused_cleanup_stop_only')


def _raw_state(value):
    if type(value) is int and value in (5, 6):
        return 'IDLE' if value == 5 else 'PAUSED'
    if type(value) is str and value.strip().upper() in ('IDLE', 'PAUSED'):
        return value.strip().upper()
    raise AdmissionRejected('paused_cleanup_state_not_settled')


def validate_sample(sample, now_ns, *, admission=True):
    if not isinstance(sample, dict) or not {'raw_robot_state', 'estop', 'latched_fault', 'running_motion'}.issubset(sample):
        raise AdmissionRejected('paused_cleanup_health_missing')
    state = _raw_state(sample['raw_robot_state'])
    if admission and state != 'PAUSED':
        raise AdmissionRejected('paused_cleanup_not_paused')
    if state == 'PAUSED' and (sample.get('motion_state') != 'FINISHED'
            or type(sample.get('raw_running_motion')) is not int or sample['raw_running_motion'] <= 0):
        raise AdmissionRejected('paused_cleanup_finished_motion_evidence_missing')
    allowed_faults = (None,) if admission else (None, 'stop_unverified', 'stop_incomplete')
    if sample['estop'] is not None or sample['latched_fault'] not in allowed_faults or sample['running_motion'] is not None:
        raise AdmissionRejected('paused_cleanup_health_or_motion_rejected')
    requested, captured, completed = (sample.get(k) for k in ('read_requested_ns', 'captured_ns', 'read_completed_ns'))
    if (not all(type(v) is int and v >= 0 for v in (requested, captured, completed, now_ns))
            or not requested <= captured <= completed <= now_ns or not 0 <= now_ns - captured < 100_000_000):
        raise AdmissionRejected('paused_cleanup_sample_not_current')
    for field in ('actual_q', 'actual_qd'):
        values = sample.get(field)
        if (not isinstance(values, (list, tuple)) or len(values) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise AdmissionRejected('paused_cleanup_invalid_joint_feedback')
    if any(v != 0 for v in sample['actual_qd']):
        raise AdmissionRejected('paused_cleanup_velocity_not_zero')
    try:
        Pose.model_validate(sample.get('actual_tcp'))
    except Exception:
        raise AdmissionRejected('paused_cleanup_invalid_tcp_feedback') from None
    return state


def _cached_sample(adapter):
    snapshot = adapter._snapshot
    if snapshot is None or adapter._snapshot_read_fault is not None:
        raise AdmissionRejected('paused_cleanup_cached_read_unavailable')
    return dict(raw_robot_state=snapshot.raw_robot_state, estop=snapshot.estop,
        latched_fault=adapter._latched_fault, running_motion=snapshot.running_motion,
        raw_running_motion=snapshot.raw_running_motion, motion_state=snapshot.motion_state,
        read_requested_ns=snapshot.read_requested_ns, captured_ns=snapshot.captured_ns,
        read_completed_ns=snapshot.read_completed_ns, actual_q=list(snapshot.actual_q),
        actual_qd=list(snapshot.actual_qd), actual_tcp=snapshot.actual_tcp.model_dump(mode='json'))


def _geometry(adapter):
    snapshot = adapter._snapshot
    if (snapshot is None or not adapter._tcp_matches(snapshot.tcp_setting)
            or not adapter._joints_inside_limits(snapshot.actual_q)
            or not adapter._tcp_inside_startup_envelope(snapshot.actual_tcp)
            or adapter._cartesian_singularity_guarded(snapshot.actual_q)
            or not adapter._capabilities.control_ready):
        raise AdmissionRejected('paused_cleanup_geometry_or_capability_rejected')


async def run_cleanup(settings, recorder, client_factory, authorization_check,
                      *, clock=time.monotonic_ns, sleep=asyncio.sleep, report=None, read_authorization_check=None):
    if not callable(client_factory) or not callable(authorization_check):
        raise TypeError('explicit_factory_and_authorization_required')
    report = {} if report is None else report
    report.update(complete=False, stop_confirmed=False, sdk_stop_entries=0, latched_fault=None,
        idle_tail_samples=0, tail_samples=0, tail_all_idle=True, tail_rejected=False,
        local_resources_released=False, native_connection_release_verified=False,
        hardware_stop_verified=False, motion_authorized=False, process_exit_required=True,
        read_gate_closed=True, read_remote_result_unknown=False, read_calls=[],
        cleanup_errors=[], native_sdk_await_qpc_ms=None)
    owner = asyncio.current_task()
    proxy = adapter = anchor = None
    permitted = connected_once = recorder_started = False
    primary_error = None

    def authorize():
        check_authorization(authorization_check)

    def check_authorization(callback):
        value = callback()
        if value is not None:
            if asyncio.iscoroutine(value):
                value.close()
            raise AdmissionRejected('paused_cleanup_invalid_authorization_result')

    def permission():
        authorize()
        if not permitted or owner is None or owner.done() or owner.cancelling() or adapter is None or anchor is None:
            raise AdmissionRejected('paused_cleanup_permission_revoked')
        if getattr(recorder, 'fatal_error', None) is not None or getattr(recorder, 'dropped_normal_events', 0):
            raise AdmissionRejected('paused_cleanup_recording_unavailable')
        latest = _cached_sample(adapter)
        proxy.require_clear_read_gate()
        validate_sample(latest, clock())
        _geometry(adapter)
        validate_position_continuity(latest, anchor)

    async def factory(ip):
        nonlocal connected_once, proxy
        authorize()
        if connected_once:
            raise AdmissionRejected('paused_cleanup_connection_already_used')
        connected_once = True
        native = await client_factory(ip)
        if not callable(getattr(native, 'stop_move', None)):
            raise AdmissionRejected('paused_cleanup_stop_unavailable')
        proxy = BoundedStopOnlyClient(native, permission,
            lambda: check_authorization(read_authorization_check or authorization_check), owner)
        return proxy

    try:
        if settings.mode != 'control':
            raise AdmissionRejected('paused_cleanup_requires_control_configuration')
        authorize()
        await recorder.start()
        recorder_started = True
        adapter = PausedStopOnlyAdapter(settings, client_factory=factory, clock=clock, sleep=sleep,
                                   event_callback=recorder.write_event)
        await adapter.connect()
        # Actual PAUSED stays HOLD; the subclass keeps motion readiness false.
        # No pump start, target submission or normal RobotControl is used here.
        authorize()
        anchor = await adapter.read_stop_observation()
        proxy.require_clear_read_gate()
        validate_sample(anchor, clock())
        _geometry(adapter)
        await recorder.write_critical_event('paused_stop_admission', dict(sample=anchor,
            scope='paused_finished_zero_speed_one_stop_move', hardware_stop_verified=False), clock())
        authorize()
        sample = await adapter.read_stop_observation()
        proxy.require_clear_read_gate()
        validate_sample(sample, clock())
        _geometry(adapter)
        validate_position_continuity(sample, anchor)
        permitted = True
        try:
            await adapter.stop(StopReason.SHUTDOWN)
        except BaseException as error:
            primary_error = error
        finally:
            permitted = False
        if proxy.sdk_stop_entries and not isinstance(primary_error, asyncio.CancelledError):
            class Tail:
                async def read_stop_observation(self):
                    authorize()
                    observed = await adapter.read_stop_observation()
                    try:
                        proxy.require_clear_read_gate()
                        state = validate_sample(observed, clock(), admission=False)
                        _geometry(adapter)
                        validate_position_continuity(observed, anchor)
                    except AdmissionRejected:
                        report['tail_rejected'] = True
                        raise
                    report['tail_samples'] += 1
                    report['idle_tail_samples'] += int(state == 'IDLE')
                    report['tail_all_idle'] &= state == 'IDLE'
                    return observed
            try:
                await observe_after_stop(Tail(), recorder, 2., prior_error=type(primary_error).__name__ if primary_error else None,
                                         clock=clock, sleep=sleep)
            except BaseException as error:
                if primary_error is None:
                    primary_error = error
    except BaseException as error:
        primary_error = error
    finally:
        permitted = False
        if proxy is not None:
            proxy.read_gate_closed = True
            report['read_gate_closed'] = proxy.read_gate_closed
            report['read_remote_result_unknown'] = proxy.read_remote_result_unknown
            report['read_calls'] = copy.deepcopy(proxy.read_calls)
            report['sdk_stop_entries'] = proxy.sdk_stop_entries
            if proxy.native_started_ns is not None and proxy.native_completed_ns is not None:
                report['native_sdk_await_qpc_ms'] = (proxy.native_completed_ns - proxy.native_started_ns) / 1e6
        if adapter is not None:
            report['latched_fault'] = adapter.read_motion_view().latched_fault
            transaction = adapter._stop_transaction
            report['stop_confirmed'] = transaction is not None and transaction.confirmed
            report['local_stop_requests'] = len(transaction.requests) if transaction else 0
            try:
                await adapter.disconnect()  # No accepted motion; proxy permission also revoked.
                report['local_resources_released'] = adapter._client is None and not adapter._pump.running
            except BaseException as error:
                report['cleanup_errors'].append(type(error).__name__)
        if primary_error is not None:
            report['error_type'] = type(primary_error).__name__
            if isinstance(primary_error, AdmissionRejected):
                report['admission_rejection'] = str(primary_error)
        report['complete'] = (primary_error is None and not report['cleanup_errors']
            and report['sdk_stop_entries'] == 1 and report['stop_confirmed'] and report['latched_fault'] is None
            and not report['tail_rejected'] and report['tail_samples'] > 0 and report['tail_all_idle'])
        if not report['tail_samples']:
            report['tail_all_idle'] = False
        if recorder_started:
            try:
                await recorder.write_critical_event('paused_stop_cleanup_result', report, clock())
            except BaseException as error:
                report['complete'] = False
                report['cleanup_errors'].append(type(error).__name__)
            try:
                await recorder.close()
            except BaseException as error:
                report['complete'] = False
                report['cleanup_errors'].append(type(error).__name__)
    if isinstance(primary_error, (asyncio.CancelledError, KeyboardInterrupt, SystemExit)):
        raise primary_error
    return report
