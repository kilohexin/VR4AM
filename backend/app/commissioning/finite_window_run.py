"""One separately scoped +Z 0.2 mm window; no normal VR/API import.

The caller binds code/config/SDK and owns one connection. Admission reserves
one original stop for up to five seconds, including motion lease revocation or
cancellation; this never permits another motion or a stop_sys escalation.
"""
import asyncio
import copy
import math
import time

from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter, _angle
from app.commissioning.idle_stop_baseline import AdmissionRejected, validate_idle_sample, validate_position_continuity
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient, _cached_sample, _geometry
from app.commissioning.stop_observation import observe_after_stop
from app.robots.base import StopReason
from app.schemas.messages import Pose

CLEANUP_STOP_S = 5.
DISTANCE_M = .0002
INTERRUPTIONS = (asyncio.CancelledError, KeyboardInterrupt, SystemExit)


def _callback(callback):
    value = callback()
    if value is not None:
        if asyncio.iscoroutine(value): value.close()
        raise AdmissionRejected('finite_scope_invalid_authorization_result')


class FiniteScopeClient(BoundedStopOnlyClient):
    READS = BoundedStopOnlyClient.READS | {'kinematics_inverse', 'kinematics_forward'}

    def __init__(self, native, motion_permission, stop_permission, hot, owner, clock):
        super().__init__(native, stop_permission, hot, owner)
        self.motion_permission, self.clock = motion_permission, clock
        self.pvat_entries = 0
        self.pvat_attempted = False
        self.pvat_remote_result_unknown = False
        self.pending_pvat = None
        self.pvat_call = None
        self.pvat_started_ns = self.pvat_horizon_s = None
        self.math_entries = dict(kinematics_inverse=0, kinematics_forward=0)

    async def _math(self, name, limit, args):
        async def invoke():
            self.read_guard()
            if self.math_entries[name] >= limit:
                raise AdmissionRejected('finite_scope_math_already_used')
            self.math_entries[name] += 1
            return await getattr(self.client, name)(*copy.deepcopy(args))
        return await self.read(name, invoke)

    async def kinematics_inverse(self, pose, joints):
        return await self._math('kinematics_inverse', 1, (pose, joints))

    async def kinematics_forward(self, joints):
        return await self._math('kinematics_forward', 4, (joints,))

    async def move_pvat(self, p, v, a, t):
        if self.pvat_attempted:
            raise AdmissionRejected('finite_scope_pvat_already_used')
        self.motion_permission(p, v, a, t)
        self.pvat_attempted = True
        args = (list(p), list(v), list(a), t)
        deadline = time.monotonic_ns() + 60_000_000
        item = self.pvat_call = dict(entered=False, outcome='not_started', remote_result_unknown=False)
        async def invoke():
            self.motion_permission(*args)  # This scheduled child is the actual native boundary.
            if time.monotonic_ns() >= deadline:
                raise AdmissionRejected('finite_scope_pvat_expired_before_native_entry')
            self.pvat_entries += 1
            self.pvat_started_ns, self.pvat_horizon_s = self.clock(), t
            item.update(entered=True, native_started_ns=self.pvat_started_ns)
            return await self.client.move_pvat(*copy.deepcopy(args))
        task = self.pending_pvat = asyncio.create_task(invoke(), name='scoped-native-pvat')
        def consume(done):
            if not done.cancelled(): done.exception()
        task.add_done_callback(consume)
        try:
            done, _ = await asyncio.wait({task}, timeout=max(0.,(deadline-time.monotonic_ns())/1e9))
            if not done:
                item['outcome']='timeout'
                raise TimeoutError
            result=task.result()
            if time.monotonic_ns() >= deadline:
                item['outcome']='late_return'
                raise TimeoutError
            item['outcome']='returned'
            return result
        except BaseException as error:
            item['remote_result_unknown']=item['entered']
            self.pvat_remote_result_unknown |= item['remote_result_unknown']
            if item['outcome']=='not_started': item['outcome']='cancelled' if isinstance(error,asyncio.CancelledError) else 'error'
            if not task.done() and not task.cancelling(): task.cancel()
            # Never await native cancellation acknowledgment while holding the adapter lock.
            raise
        finally:
            item['completed_ns']=self.clock()


def _rest(sample, now_ns):
    validate_idle_sample(sample, now_ns)
    acceleration = sample.get('actual_qdd')
    if (not isinstance(acceleration, (tuple, list)) or len(acceleration) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) or v != 0 for v in acceleration)):
        raise AdmissionRejected('finite_scope_initial_acceleration_not_zero')


async def run_window(settings, recorder, client_factory, authorization_check, hot_check,
                     *, clock=time.monotonic_ns, sleep=asyncio.sleep, report=None, write_check=None):
    if not all(callable(v) for v in (client_factory, authorization_check, hot_check)):
        raise TypeError('finite_scope_explicit_callbacks_required')
    if write_check is not None and not callable(write_check): raise TypeError('finite_scope_write_check_required')
    report = {} if report is None else report
    report.update(complete=False, hardware_verified=False, motion_authorized=False, scope_admitted=False,
        stop_confirmed=False, latched_fault=None, pvat_entries=0, stop_entries=0,
        tail_complete=False, displacement_consistent=False, displacement_samples=0, cleanup_errors=[], interrupted=False,
        read_remote_result_unknown=False, read_calls=[], native_connection_release_verified=False,
        local_resources_released=False, process_exit_required=True, pvat_remote_result_unknown=False)
    owner = asyncio.current_task()
    proxy = adapter = anchor = None
    cleanup_deadline = None
    connected_once = admitted = closed = False
    primary_error = None

    def hot():
        _callback(hot_check)

    def stop_permission():
        # This is a reserved cleanup write, not renewed motion authority.
        if (not admitted or closed or owner is None or owner.done() or cleanup_deadline is None
                or not 0 <= cleanup_deadline - clock() <= int(CLEANUP_STOP_S * 1e9)):
            raise AdmissionRejected('finite_scope_cleanup_stop_not_admitted')

    def motion_permission(p, v, a, t):
        _callback(write_check if write_check is not None else hot_check)
        hot()
        stop_permission()
        if cleanup_deadline - clock() < 1_000_000_000 or proxy.sdk_stop_entries:
            raise AdmissionRejected('finite_scope_cleanup_reservation_insufficient')
        if (not admitted or closed or owner is None or owner.done() or owner.cancelling()
                or getattr(recorder, 'fatal_error', None) is not None or getattr(recorder, 'dropped_normal_events', 0)):
            raise AdmissionRejected('finite_scope_motion_permission_revoked')
        proxy.require_clear_read_gate()
        latest = _cached_sample(adapter)
        latest['actual_qdd'] = list(adapter._snapshot.actual_qdd)
        _rest(latest, clock())
        _geometry(adapter)
        validate_position_continuity(latest, anchor)
        ticket = adapter._finite_ticket
        if ticket is None or ticket[0] is not proxy:
            raise AdmissionRejected('finite_scope_audit_ticket_missing')
        _, snapshot, _, selected = ticket
        adapter._guard(proxy, snapshot)
        if any(len(values) != 6 or any(type(value) not in (int, float) or not math.isfinite(value)
                for value in values) for values in (p, v, a)):
            raise AdmissionRejected('finite_scope_invalid_packet')
        if (tuple(p) != selected.point.q or tuple(v) != (0.,) * 6 or tuple(a) != (0.,) * 6
                or t != selected.point.horizon_s or not .01 <= t <= .08):
            raise AdmissionRejected('finite_scope_packet_not_bound')
        # The fast callback can consume the remaining lease; guard again last.
        adapter._guard(proxy, snapshot)
        stop_permission()
        if cleanup_deadline-clock()<1_000_000_000:
            raise AdmissionRejected('finite_scope_cleanup_reservation_insufficient')

    async def factory(ip):
        nonlocal proxy, connected_once
        _callback(authorization_check)
        if connected_once:
            raise AdmissionRejected('finite_scope_connection_already_used')
        connected_once = True
        native = await client_factory(ip)
        if not all(callable(getattr(native, name, None)) for name in
                   ('move_pvat', 'stop_move', 'kinematics_inverse', 'kinematics_forward')):
            raise AdmissionRejected('finite_scope_capability_missing')
        proxy = FiniteScopeClient(native, motion_permission, stop_permission, hot, owner, clock)
        return proxy

    try:
        if settings.mode != 'control': raise AdmissionRejected('finite_scope_control_config_required')
        _callback(authorization_check)
        await recorder.start()
        adapter = FiniteWindowDiagnosticAdapter(settings, client_factory=factory, clock=clock,
                                               sleep=sleep, event_callback=recorder.write_event)
        await adapter.connect()
        _callback(authorization_check)  # Full hashes remain outside the 100 ms motion lease.
        preflight = await adapter.preflight()
        if not preflight.ready: raise AdmissionRejected('finite_scope_preflight_rejected')
        anchor = await adapter.read_stop_observation()
        proxy.require_clear_read_gate()
        _rest(anchor, clock()); _geometry(adapter)
        await recorder.write_critical_event('finite_scope_admission', dict(sample=anchor,
            allowed_pvat=1, allowed_cleanup_stop=1, cleanup_stop_s=CLEANUP_STOP_S), clock())
        _callback(authorization_check)
        sample = await adapter.read_stop_observation()
        proxy.require_clear_read_gate()
        _rest(sample, clock()); _geometry(adapter); validate_position_continuity(sample, anchor)
        admitted = report['scope_admitted'] = True
        cleanup_deadline = clock() + int(CLEANUP_STOP_S * 1e9)
        origin = Pose.model_validate(sample['actual_tcp'])
        target = Pose(p=(*origin.p[:2], origin.p[2] + DISTANCE_M), q=origin.q)
        receipt = await adapter.submit_tcp(target, 1)
        outcome = await asyncio.wait_for(receipt.completion.wait(), 1.)
        report.update(command_sdk_outcome=outcome.sdk_outcome, command_error=outcome.error)
        if proxy.pvat_entries and outcome.sdk_outcome == 'returned':
            remaining = (proxy.pvat_started_ns + int(proxy.pvat_horizon_s * 1e9) - clock()) / 1e9
            if remaining > 0: await sleep(remaining)
        try:
            await adapter.stop(StopReason.SHUTDOWN)
        except BaseException as error:
            primary_error = error
        if isinstance(primary_error, INTERRUPTIONS):
            raise primary_error
        if proxy.sdk_stop_entries:
            tail_anchor = _cached_sample(adapter)
            class Tail:
                async def read_stop_observation(self):
                    observed = await adapter.read_stop_observation()
                    proxy.require_clear_read_gate()
                    # Diagnostic reads preserve these original latches in output.
                    check = copy.deepcopy(observed)
                    if check['latched_fault'] in ('stop_unverified', 'stop_incomplete'):
                        check['latched_fault'] = None
                    validate_idle_sample(check, clock())
                    _geometry(adapter)
                    validate_position_continuity(observed, tail_anchor)
                    delta = [a - b for a, b in zip(Pose.model_validate(observed['actual_tcp']).p, origin.p)]
                    progress, cross = delta[2], math.hypot(*delta[:2])
                    consistent = (0 < progress <= DISTANCE_M + 1e-6
                        and cross <= max(1e-6, progress * math.tan(math.radians(5)))
                        and _angle(Pose.model_validate(observed['actual_tcp']).q, origin.q) <= .1)
                    report['displacement_consistent'] = consistent if not report['displacement_samples'] else report['displacement_consistent'] and consistent
                    report['displacement_samples'] += 1
                    report['last_displacement'] = dict(progress_m=progress, cross_m=cross)
                    return observed
            await observe_after_stop(Tail(), recorder, 2., clock=clock, sleep=sleep)
            report['tail_complete'] = True
    except BaseException as error:
        primary_error = error
    finally:
        if adapter is not None:
            try: await adapter.disconnect()
            except BaseException as error:
                report['cleanup_errors'].append(type(error).__name__)
                if primary_error is None or isinstance(error, INTERRUPTIONS): primary_error = error
            report['local_resources_released'] = adapter._client is None and not adapter._pump.running
            if proxy is not None and proxy.pvat_remote_result_unknown:
                adapter._latched_fault='stop_unverified'
            report['latched_fault'] = adapter.read_motion_view().latched_fault
            report['original_stop_confirmed'] = adapter._stop_transaction is not None and adapter._stop_transaction.confirmed
            report['stop_confirmed'] = report['original_stop_confirmed'] and not (proxy is not None and proxy.pvat_remote_result_unknown)
        closed = True
        if proxy is not None:
            proxy.read_gate_closed = True
            report.update(pvat_entries=proxy.pvat_entries, stop_entries=proxy.sdk_stop_entries,
                math_entries=copy.deepcopy(proxy.math_entries), read_calls=copy.deepcopy(proxy.read_calls),
                read_remote_result_unknown=proxy.read_remote_result_unknown,
                pvat_remote_result_unknown=proxy.pvat_remote_result_unknown, pvat_call=copy.deepcopy(proxy.pvat_call),
                owned_pvat_pending_at_exit=proxy.pending_pvat is not None and not proxy.pending_pvat.done())
        report['interrupted'] = isinstance(primary_error, INTERRUPTIONS)
        if primary_error is not None: report['error_type'] = type(primary_error).__name__
        report['complete'] = (primary_error is None and not report['cleanup_errors'] and report.get('command_sdk_outcome') == 'returned'
            and report['pvat_entries'] == report['stop_entries'] == 1 and report['stop_confirmed']
            and report['latched_fault'] is None and report['tail_complete'] and report['displacement_consistent'])
        try: await recorder.write_critical_event('finite_scope_result', report, clock())
        except BaseException as error:
            report['complete'] = False; report['cleanup_errors'].append(type(error).__name__)
            if isinstance(error, INTERRUPTIONS):
                primary_error = error
                report.update(interrupted=True, error_type=type(error).__name__)
        try: await recorder.close()
        except BaseException as error:
            report['complete'] = False; report['cleanup_errors'].append(type(error).__name__)
            if isinstance(error, INTERRUPTIONS):
                primary_error = error
                report.update(interrupted=True, error_type=type(error).__name__)
    if isinstance(primary_error, INTERRUPTIONS): raise primary_error
    return report
