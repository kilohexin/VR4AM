"""Dedicated idle-only baseline; callers must supply explicit scope admission."""
import asyncio
import math
import time

from app.commissioning.stop_observation import observe_after_stop
from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter, _STOP_JOINT_DRIFT_RAD, _STOP_TCP_DRIFT_M


class AdmissionRejected(BackendCommandError):
    pass


def validate_idle_sample(sample, now_ns):
    if not isinstance(sample, dict) or not {'raw_robot_state', 'estop', 'latched_fault', 'running_motion'}.issubset(sample):
        raise AdmissionRejected('idle_baseline_health_missing')
    raw = sample.get('raw_robot_state')
    if type(raw) not in (str, int) or (raw.strip().upper() if isinstance(raw, str) else raw) not in ('IDLE', 5):
        raise AdmissionRejected('idle_baseline_not_idle')
    if sample.get('estop') is not None or sample.get('latched_fault') is not None:
        raise AdmissionRejected('idle_baseline_fault_present')
    if sample.get('running_motion') is not None:
        raise AdmissionRejected('idle_baseline_motion_running')
    requested, captured, completed = (sample.get(key) for key in
        ('read_requested_ns', 'captured_ns', 'read_completed_ns'))
    if (not all(type(value) is int and value >= 0 for value in (requested, captured, completed, now_ns))
        or not requested <= captured <= completed <= now_ns
        or not 0 <= now_ns - captured < 100_000_000):
        raise AdmissionRejected('idle_baseline_sample_not_current')
    speeds = sample.get('actual_qd')
    if (not isinstance(speeds, (list, tuple)) or len(speeds) != 6
        or any(type(value) not in (int, float) or not math.isfinite(value) or value != 0 for value in speeds)):
        # Diagnostic eligibility only; the production stop tolerance is unchanged.
        raise AdmissionRejected('idle_baseline_velocity_not_zero')


class StopOnlyClient:
    READS = frozenset(('is_connected', 'get_robot_state', 'get_estop_reason', 'get_kin_data',
                      'get_tcp', 'get_claw', 'get_running_motion', 'get_motion_state'))

    def __init__(self, client, permission):
        self.client, self.permission = client, permission
        self.sdk_stop_entries = 0
        self.native_started_ns = self.native_completed_ns = None

    def __getattr__(self, name):
        method = getattr(self.client, name)  # Preserve missing native capabilities.
        if not callable(method):
            raise AttributeError(name)
        if name in self.READS:
            return method

        async def denied(*args, **kwargs):
            raise AdmissionRejected('idle_baseline_write_forbidden')
        return denied

    async def stop_move(self):
        if self.sdk_stop_entries:
            raise AdmissionRejected('idle_baseline_stop_already_used')
        self.permission()  # Runs inside StopRpc's scheduled task, before native entry.
        self.sdk_stop_entries += 1
        self.native_started_ns = time.perf_counter_ns()
        try:
            return await self.client.stop_move()
        finally:
            self.native_completed_ns = time.perf_counter_ns()

    async def is_connected(self):
        return await self.client.is_connected() is True


def validate_position_continuity(sample, anchor):
    if (max(abs(value - previous) for value, previous in zip(sample['actual_q'], anchor['actual_q'])) > _STOP_JOINT_DRIFT_RAD
        or math.dist(sample['actual_tcp']['p'], anchor['actual_tcp']['p']) > _STOP_TCP_DRIFT_M):
        raise AdmissionRejected('idle_baseline_position_changed')


async def run_baseline(settings, recorder, client_factory, authorization_check,
                       *, clock=time.monotonic_ns, sleep=asyncio.sleep):
    """One injected connection; no RobotControl or generic smoke cleanup.

    Live use requires a standalone, approved CLI. Local reference release is
    not proof that the native SDK has closed its transport.
    """
    if not callable(authorization_check) or not callable(client_factory):
        raise TypeError('explicit_admission_and_factory_required')
    report = dict(complete=False, stop_confirmed=False, sdk_stop_entries=0, latched_fault=None,
                  local_resources_released=False, native_connection_release_verified=False,
                  process_exit_required=True, hardware_stop_verified=False, cleanup_errors=[], tail_rejected=False,
                  native_sdk_await_qpc_ms=None)
    owner = asyncio.current_task()
    proxy, adapter, sample = None, None, None
    permitted = False
    primary_error = None
    recorder_started = False

    def authorize():
        value = authorization_check()
        if value is not None:
            if asyncio.iscoroutine(value):
                value.close()
            raise AdmissionRejected('idle_baseline_invalid_authorization_result')

    def permission():
        authorize()
        if not permitted or owner is None or owner.done() or owner.cancelling():
            raise AdmissionRejected('idle_baseline_permission_revoked')
        if sample is None:
            raise AdmissionRejected('idle_baseline_sample_missing')
        if getattr(recorder, 'fatal_error', None) is not None or getattr(recorder, 'dropped_normal_events', 0):
            raise AdmissionRejected('idle_baseline_recording_unavailable')
        # Reuse the adapter's full cached preflight rules for the latest read,
        # including TCP/geometry changes, without another SDK await here.
        if (adapter is None or adapter._snapshot is None
            or not adapter._preflight_from_snapshot(adapter._snapshot).ready):
            raise AdmissionRejected('idle_baseline_preflight_changed')
        validate_idle_sample(sample, clock())

    async def factory(ip):
        nonlocal proxy
        authorize()
        native = await client_factory(ip)
        if not callable(getattr(native, 'stop_move', None)):
            raise AdmissionRejected('idle_baseline_stop_capability_missing')
        proxy = StopOnlyClient(native, permission)
        return proxy

    try:
        if settings.mode != 'control':
            raise AdmissionRejected('idle_baseline_requires_control_config')
        authorize()
        await recorder.start()
        recorder_started = True
        adapter = RealLebaiAdapter(settings, client_factory=factory, clock=clock, sleep=sleep,
                                   event_callback=recorder.write_event)
        await adapter.connect()
        authorize()
        preflight = await adapter.preflight()
        if not preflight.ready:
            raise AdmissionRejected('idle_baseline_preflight_not_ready')
        sample = await adapter.read_stop_observation()
        validate_idle_sample(sample, clock())
        anchor = sample
        await recorder.write_critical_event('idle_stop_admission',
            {'sample': sample, 'scope': 'idle_only_one_stop_move', 'hardware_stop_verified': False}, clock())
        authorize()
        # Recording or lock wait must not turn an old read into write permission.
        sample = await adapter.read_stop_observation()
        validate_idle_sample(sample, clock())
        validate_position_continuity(sample, anchor)
        permitted = True
        try:
            await adapter.stop(StopReason.GRIP_RELEASED)
        except BaseException as error:
            primary_error = error
        finally:
            permitted = False
        if proxy.sdk_stop_entries and not isinstance(primary_error, asyncio.CancelledError):
            class IdleTail:
                async def read_stop_observation(self):
                    observation = await adapter.read_stop_observation()
                    try:
                        validate_idle_sample(observation, clock())
                        validate_position_continuity(observation, sample)
                    except AdmissionRejected as error:
                        report['tail_rejected'] = True
                        await recorder.write_critical_event('idle_stop_tail_rejected',
                            {'sample': observation, 'reason': str(error)}, clock())
                        raise
                    return observation
            try:
                await observe_after_stop(IdleTail(), recorder, 2.0,
                    prior_error=type(primary_error).__name__ if primary_error else None, clock=clock, sleep=sleep)
            except BaseException as error:
                if primary_error is None:
                    primary_error = error
    except BaseException as error:
        primary_error = error
    finally:
        permitted = False
        if proxy is not None:
            report['sdk_stop_entries'] = proxy.sdk_stop_entries
        if adapter is not None:
            report['latched_fault'] = adapter.read_motion_view().latched_fault
            transaction = adapter._stop_transaction
            report['stop_confirmed'] = transaction is not None and transaction.confirmed
            report['local_stop_requests'] = len(transaction.requests) if transaction is not None else 0
            try:
                # No motion source exists in this workflow. The proxy remains
                # revoked even if a future cleanup path tries to issue a write.
                await adapter.disconnect()
                report['local_resources_released'] = (not adapter._pump.running and adapter._client is None)
            except BaseException as error:
                report['cleanup_errors'].append(type(error).__name__)
        if proxy is not None and proxy.native_started_ns is not None and proxy.native_completed_ns is not None:
            report['native_sdk_await_qpc_ms'] = (proxy.native_completed_ns - proxy.native_started_ns) / 1e6
        report['complete'] = (primary_error is None and not report['cleanup_errors']
                              and not report['tail_rejected']
                              and report['sdk_stop_entries'] == 1 and report['stop_confirmed']
                              and report['latched_fault'] is None)
        if primary_error is not None:
            report['error_type'] = type(primary_error).__name__
            if isinstance(primary_error, AdmissionRejected):
                report['admission_rejection'] = str(primary_error)
        if recorder_started:
            try:
                await recorder.write_critical_event('idle_stop_baseline_result', report, clock())
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
