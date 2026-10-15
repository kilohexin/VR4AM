"""One separately scoped +Z window up to 0.2 mm; no normal VR/API import.

The caller binds code/config/SDK and owns one connection. Admission reserves
one original stop for up to five seconds, including motion lease revocation or
cancellation; this never permits another motion or a stop_sys escalation.
"""
import asyncio
import copy
from contextlib import contextmanager
from contextvars import ContextVar
import math
import time

from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter, _angle
from app.commissioning.idle_stop_baseline import AdmissionRejected, validate_idle_sample, validate_position_continuity
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient, _cached_sample, _geometry
from app.commissioning.stop_observation import observe_after_stop
from app.commissioning.readonly_kinematics import computational_distance
from app.robots.base import StopReason
from app.schemas.messages import Pose

CLEANUP_STOP_S = 5.
DISTANCE_M = .0002
READY_Q_TOLERANCE_RAD = .0001
MAX_READY_Q_TOLERANCE_RAD = .001
INTERRUPTIONS = (asyncio.CancelledError, KeyboardInterrupt, SystemExit)


def _callback(callback):
    value = callback()
    if value is not None:
        if asyncio.iscoroutine(value): value.close()
        raise AdmissionRejected('finite_scope_invalid_authorization_result')


def validate_ready_tolerance(value):
    if (type(value) not in (int,float) or not math.isfinite(value)
            or not READY_Q_TOLERANCE_RAD <= value <= MAX_READY_Q_TOLERANCE_RAD):
        raise ValueError('finite_scope_ready_tolerance_invalid')
    return float(value)


class FiniteScopeClient(BoundedStopOnlyClient):
    READS = BoundedStopOnlyClient.READS | {'kinematics_inverse', 'kinematics_forward'}

    def __init__(self, native, motion_permission, stop_permission, hot, owner, clock, *, parallel_fk=False,
                 parallel_motion_reads=False):
        if type(parallel_fk) is not bool:
            raise TypeError('finite_parallel_fk_requires_boolean')
        if type(parallel_motion_reads) is not bool:
            raise TypeError('finite_parallel_snapshot_requires_boolean')
        super().__init__(native, stop_permission, hot, owner)
        self.parallel_fk = parallel_fk
        self.fk_read_owners = []
        self.parallel_motion_reads = parallel_motion_reads
        self.snapshot_read_owners = {}
        self._snapshot_context = ContextVar('finite_snapshot_group', default=None)
        self._active_snapshot_group = None
        self.motion_permission, self.clock = motion_permission, clock
        self.pvat_entries = 0
        self.pvat_attempted = False
        self.pvat_remote_result_unknown = False
        self.pending_pvat = None
        self.pvat_call = None
        self.pvat_started_ns = self.pvat_horizon_s = None
        self.math_entries = dict(kinematics_inverse=0, kinematics_forward=0)
        self.local_motion_rejection = None

    def _fk_read_pending(self):
        return any(child.pending_read is not None and not child.pending_read.done()
                   for child in self.fk_read_owners)

    def _snapshot_read_pending(self):
        return any(child.pending_read is not None and not child.pending_read.done()
                   for child in self.snapshot_read_owners.values())

    def _native_snapshot_entry_count(self, method):
        children=list(self.snapshot_read_owners.values())
        owned_ids={id(getattr(child,'native_item',None)) for child in children}
        return (sum(item['entered'] for item in self.read_calls
                    if item['method']==method and id(item) not in owned_ids)
                + sum(child.native_entries for child in children if child.method==method))

    def _native_snapshot_result_unknown(self, method):
        child=self.snapshot_read_owners.get(method)
        matching=[item for item in self.read_calls if item['method']==method]
        if child is not None and matching and matching[-1] is getattr(child,'native_item',None):
            item=matching[-1];task=child.pending_read
            if item['remote_result_unknown'] or child.read_remote_result_unknown:return True
            if not child.native_entries:return False
            return task is None or not task.done() or task.cancelled()
        return super()._native_snapshot_result_unknown(method)

    def read_guard(self):
        super().read_guard()
        group=self._snapshot_context.get()
        if group is not None:
            if group is not self._active_snapshot_group or not group['active']:
                raise AdmissionRejected('finite_scope_snapshot_group_revoked')
            group['guard']()

    @contextmanager
    def _parallel_snapshot_scope(self, guard):
        self.require_clear_read_gate();guard()
        if (not self.parallel_motion_reads or self._active_snapshot_group is not None
                or self.snapshot_read_owners or self.pending_read is not None and not self.pending_read.done()):
            raise AdmissionRejected('finite_scope_snapshot_group_not_admitted')
        group=dict(active=True,guard=guard,used=set())
        self._active_snapshot_group=group;token=self._snapshot_context.set(group)
        try:yield
        finally:
            group['active']=False;self._active_snapshot_group=None;self._snapshot_context.reset(token)

    async def _snapshot_read(self, name, invoke, group):
        self.read_guard()
        if (name in group['used'] or self._fk_read_pending()
                or self.pending_read is not None and not self.pending_read.done()):
            self.read_gate_closed=True
            raise AdmissionRejected('finite_scope_snapshot_duplicate_or_pending')
        group['used'].add(name)
        def authorize():
            self.read_guard()
            if self._active_snapshot_group is not group or not group['active']:
                raise AdmissionRejected('finite_scope_snapshot_group_revoked')
            if self.pending_read is not None and not self.pending_read.done():
                raise AdmissionRejected('finite_scope_parent_read_pending')
        child=_SnapshotReadOwner(self.client,lambda:None,authorize,self.owner)
        child.method=name;child.native_entries=0;child.native_item=None
        child.read_deadline_ns=self.read_deadline_ns;child.read_calls=self.read_calls
        self.snapshot_read_owners[name]=child
        async def native():
            child.native_item=next(item for item in reversed(self.read_calls) if item['method']==name)
            authorize()
            child.native_entries+=1
            return await invoke()
        try:
            value=await child.read(name,native)
            if name=='get_estop_reason':self.raw_estop_clear=child.raw_estop_clear
            return value
        except BaseException:
            self.read_gate_closed=True
            raise
        finally:
            self.read_gate_closed|=child.read_gate_closed
            self.read_remote_result_unknown|=child.read_remote_result_unknown

    async def read(self, name, invoke):
        group=self._snapshot_context.get()
        if group is not None and name in _SnapshotReadOwner.READS:
            return await self._snapshot_read(name,invoke,group)
        if group is not None:
            self.read_guard()
            if name not in {'is_connected','get_motion_state'} or name in group['used']:
                self.read_gate_closed=True
                raise AdmissionRejected('finite_scope_snapshot_method_not_admitted')
            group['used'].add(name)
        elif self._active_snapshot_group is not None:
            self.read_gate_closed=True
            raise AdmissionRejected('finite_scope_snapshot_foreign_read')
        if self._snapshot_read_pending():
            self.read_gate_closed=True
            raise AdmissionRejected('finite_scope_snapshot_read_pending')
        if self._fk_read_pending():
            self.read_gate_closed = True
            raise AdmissionRejected('finite_scope_fk_read_pending')
        return await super().read(name, invoke)

    def require_clear_read_gate(self):
        if (self._active_snapshot_group is not None or self._snapshot_read_pending()
                or any(child.read_gate_closed or child.read_remote_result_unknown
                for child in self.snapshot_read_owners.values())):
            self.read_gate_closed=True
            raise AdmissionRejected('finite_scope_snapshot_read_not_clear')
        if (self._fk_read_pending() or any(child.read_gate_closed or child.read_remote_result_unknown
                for child in self.fk_read_owners)):
            self.read_gate_closed = True
            raise AdmissionRejected('finite_scope_fk_read_not_clear')
        return super().require_clear_read_gate()

    async def _kinematics_forward_guarded(self, joints, guard):
        if not self.parallel_fk or not callable(guard):
            raise AdmissionRejected('finite_scope_parallel_fk_not_admitted')
        self.read_guard(); guard()
        if (self._snapshot_read_pending() or self._active_snapshot_group is not None
                or self.pending_read is not None and not self.pending_read.done()
                or len(self.fk_read_owners) >= 4 or sum(
                    child.pending_read is not None and not child.pending_read.done()
                    for child in self.fk_read_owners) >= 3):
            self.read_gate_closed = True
            raise AdmissionRejected('finite_scope_fk_budget_or_pending')
        def authorize():
            self.read_guard(); guard()
            if self.pending_read is not None and not self.pending_read.done():
                raise AdmissionRejected('finite_scope_parent_read_pending')
        child = _ForwardReadOwner(self.client, lambda: None, authorize, self.owner)
        child.read_deadline_ns = self.read_deadline_ns
        child.read_calls = self.read_calls
        self.fk_read_owners.append(child)
        async def invoke():
            authorize()  # Actual native entry, after both scheduling boundaries.
            if self.math_entries['kinematics_forward'] >= 4:
                raise AdmissionRejected('finite_scope_math_already_used')
            self.math_entries['kinematics_forward'] += 1
            return await self.client.kinematics_forward(copy.deepcopy(joints))
        try:
            return await child.read('kinematics_forward', invoke)
        except BaseException:
            self.read_gate_closed = True
            raise
        finally:
            self.read_remote_result_unknown |= child.read_remote_result_unknown
            self.read_gate_closed |= child.read_gate_closed

    def _motion_permission(self, *args):
        try:
            self.motion_permission(*args)
        except (AdmissionRejected, ValueError) as error:
            reason=str(error)
            self.local_motion_rejection=(reason if reason.startswith(('idle_baseline_','finite_scope_'))
                else 'local_admission_rejected')
            raise

    def _native_motion_entry_count(self, method):
        """Trusted proxy evidence: distinguish local rejection from SDK entry."""
        return self.pvat_entries if method == 'move_pvat' else self.math_entries[method]

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
        if self.parallel_fk:
            raise AdmissionRejected('finite_scope_parallel_fk_guard_required')
        return await self._math('kinematics_forward', 4, (joints,))

    async def move_pvat(self, p, v, a, t):
        if self.pvat_attempted:
            raise AdmissionRejected('finite_scope_pvat_already_used')
        self._motion_permission(p, v, a, t)
        self.pvat_attempted = True
        args = (list(p), list(v), list(a), t)
        deadline = time.monotonic_ns() + 60_000_000
        item = self.pvat_call = dict(entered=False, outcome='not_started', remote_result_unknown=False)
        async def invoke():
            self._motion_permission(*args)  # This scheduled child is the actual native boundary.
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


class _ForwardReadOwner(BoundedStopOnlyClient):
    READS = frozenset({'kinematics_forward'})


class _SnapshotReadOwner(BoundedStopOnlyClient):
    READS = frozenset({'get_robot_state','get_estop_reason','get_kin_data',
                      'get_tcp','get_claw','get_running_motion'})


def _rest(sample, now_ns):
    validate_idle_sample(sample, now_ns)
    acceleration = sample.get('actual_qdd')
    if (not isinstance(acceleration, (tuple, list)) or len(acceleration) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) or v != 0 for v in acceleration)):
        raise AdmissionRejected('finite_scope_initial_acceleration_not_zero')


async def run_window(settings, recorder, client_factory, authorization_check, hot_check,
                     *, clock=time.monotonic_ns, sleep=asyncio.sleep, report=None, write_check=None,
                     parallel_fk_audit=False, ready_joint_tolerance_rad=READY_Q_TOLERANCE_RAD,
                     parallel_motion_reads=False):
    if type(parallel_fk_audit) is not bool:
        raise TypeError('finite_parallel_fk_requires_boolean')
    if type(parallel_motion_reads) is not bool:
        raise TypeError('finite_parallel_snapshot_requires_boolean')
    ready_joint_tolerance_rad=validate_ready_tolerance(ready_joint_tolerance_rad)
    if not all(callable(v) for v in (client_factory, authorization_check, hot_check)):
        raise TypeError('finite_scope_explicit_callbacks_required')
    if write_check is not None and not callable(write_check): raise TypeError('finite_scope_write_check_required')
    distance_m=computational_distance(settings)
    full_endpoint_required=distance_m<DISTANCE_M
    report = {} if report is None else report
    report.update(complete=False, hardware_verified=False, motion_authorized=False, scope_admitted=False,
        stop_confirmed=False, latched_fault=None, pvat_entries=0, stop_entries=0,
        tail_complete=False, displacement_consistent=False, displacement_samples=0, cleanup_errors=[], interrupted=False,
        read_remote_result_unknown=False, read_calls=[], native_connection_release_verified=False,
        local_resources_released=False, process_exit_required=True, pvat_remote_result_unknown=False,
        full_endpoint_required=full_endpoint_required, full_endpoint_feasible=False,
        target_distance_m=distance_m, parallel_fk_audit=parallel_fk_audit,
        ready_joint_tolerance_rad=ready_joint_tolerance_rad,parallel_motion_reads=parallel_motion_reads)
    owner = asyncio.current_task()
    proxy = adapter = anchor = None
    cleanup_deadline = None
    connected_once = admitted = closed = False
    primary_error = None

    def ready_pose(sample):
        if full_endpoint_required:
            delta=max(abs(a-b) for a,b in zip(sample['actual_q'],settings.teleop_ready_q))
            report['ready_max_joint_delta_rad']=delta
            if delta>ready_joint_tolerance_rad:
                raise AdmissionRejected('finite_scope_ready_pose_changed')

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
        now_ns=clock()
        report['motion_sample_age_ms']=(now_ns-latest['captured_ns'])/1e6
        _rest(latest, now_ns)
        ready_pose(latest)
        _geometry(adapter)
        validate_position_continuity(latest, anchor)
        ticket = adapter._finite_ticket
        if ticket is None or ticket[0] is not proxy:
            raise AdmissionRejected('finite_scope_audit_ticket_missing')
        _, snapshot, _, selected = ticket
        report['full_endpoint_feasible']=tuple(selected.point.q)==tuple(selected.metrics.solution_q)
        if full_endpoint_required and not report['full_endpoint_feasible']:
            raise AdmissionRejected('finite_scope_full_endpoint_required')
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
        proxy = FiniteScopeClient(native, motion_permission, stop_permission, hot, owner, clock,
                                 parallel_fk=parallel_fk_audit,parallel_motion_reads=parallel_motion_reads)
        return proxy

    try:
        if settings.mode != 'control': raise AdmissionRejected('finite_scope_control_config_required')
        _callback(authorization_check)
        await recorder.start()
        adapter = FiniteWindowDiagnosticAdapter(settings, client_factory=factory, clock=clock,
                                               sleep=sleep, event_callback=recorder.write_event,
                                               parallel_fk=parallel_fk_audit,parallel_motion_reads=parallel_motion_reads)
        await adapter.connect()
        _callback(authorization_check)  # Full hashes remain outside the 100 ms motion lease.
        preflight = await adapter.preflight()
        if not preflight.ready: raise AdmissionRejected('finite_scope_preflight_rejected')
        anchor = await adapter.read_stop_observation()
        proxy.require_clear_read_gate()
        _rest(anchor, clock()); ready_pose(anchor); _geometry(adapter)
        await recorder.write_critical_event('finite_scope_admission', dict(sample=anchor,
            allowed_pvat=1, allowed_cleanup_stop=1, cleanup_stop_s=CLEANUP_STOP_S), clock())
        _callback(authorization_check)
        sample = await adapter.read_stop_observation()
        proxy.require_clear_read_gate()
        _rest(sample, clock()); ready_pose(sample); _geometry(adapter); validate_position_continuity(sample, anchor)
        admitted = report['scope_admitted'] = True
        cleanup_deadline = clock() + int(CLEANUP_STOP_S * 1e9)
        origin = Pose.model_validate(sample['actual_tcp'])
        target = Pose(p=(*origin.p[:2], origin.p[2] + distance_m), q=origin.q)
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
                    consistent = (0 < progress <= distance_m + 1e-6
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
            report['snapshot_rpc_ownership'] = adapter._snapshot_calls.snapshot()
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
                local_motion_rejection=proxy.local_motion_rejection,
                owned_fk_pending_at_exit=proxy._fk_read_pending(),
                owned_snapshot_pending_at_exit=proxy._snapshot_read_pending(),
                snapshot_read_gate_closed=any(child.read_gate_closed for child in proxy.snapshot_read_owners.values()),
                fk_read_gate_closed=any(child.read_gate_closed for child in proxy.fk_read_owners),
                owned_pvat_pending_at_exit=proxy.pending_pvat is not None and not proxy.pending_pvat.done())
        report['interrupted'] = isinstance(primary_error, INTERRUPTIONS)
        if primary_error is not None:
            report['error_type'] = type(primary_error).__name__
            reason=str(primary_error)
            report['primary_rejection']=(reason if isinstance(primary_error,(AdmissionRejected,ValueError))
                and reason.startswith(('finite_scope_','idle_baseline_','paused_cleanup_')) else None)
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
