"""Isolated single-window diagnostic. No normal VR configuration selects it.

The supplied factory must itself be admitted by a future site scope. This
module does not connect on import or offer a generic RPC/CLI entry. Cartesian
checks sample the declared reference; they do not prove the full path or
physical execution. Stop policy and request ownership come from the original
adapter, including its unconfirmed-stop latch. There is no resume or retry.
"""
from __future__ import annotations

import asyncio
import copy
from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import replace
import math

import numpy as np
from scipy.spatial.transform import Rotation

from app.robots.base import BackendCommandError, StopReason
from app.robots.lebai_adapter import RealLebaiAdapter, _MotionSnapshotFault
from app.robots.lebai_codec import pose_from_lebai
from app.robots.lebai_finite_pvat import build_finite_pvat_candidate, _controls

_LEASE_NS = 100_000_000
_MAX_TRANSLATION_M = .0002
_FK_POSITION_TOLERANCE_M = .0001
_FK_ROTATION_TOLERANCE_DEG = .1
_FK_TIMEOUT_S = .05
_DIRECTION_ANGLE_DEG = 5.


class _FiniteRevoked(BackendCommandError):
    pass


def _angle(a, b):
    return math.degrees((Rotation.from_quat(a) * Rotation.from_quat(b).inv()).magnitude())


def _snapshot_signature(snapshot):
    return (snapshot.read_requested_ns, snapshot.captured_ns, snapshot.read_completed_ns,
            snapshot.actual_q, snapshot.actual_qd, snapshot.actual_qdd,
            snapshot.actual_tcp.p, snapshot.actual_tcp.q, snapshot.tcp_setting.p, snapshot.tcp_setting.q)


class FiniteWindowDiagnosticAdapter(RealLebaiAdapter):
    def __init__(self, settings, *, client_factory, parallel_fk=False, parallel_motion_reads=False, **kwargs):
        if settings.mode != 'control':
            raise ValueError('finite_diagnostic_requires_control_mode')
        if type(parallel_fk) is not bool:
            raise TypeError('finite_parallel_fk_requires_boolean')
        if type(parallel_motion_reads) is not bool:
            raise TypeError('finite_parallel_snapshot_requires_boolean')
        super().__init__(settings, client_factory=client_factory, **kwargs)
        self.parallel_fk = parallel_fk
        self.parallel_motion_reads = parallel_motion_reads
        self._finite_snapshot_wrappers = set()
        self._finite_submitted = False
        self._finite_entered = False
        self._finite_lease = None
        self._finite_context = None
        self._finite_ticket = None
        self._finite_input_signature = None
        self._finite_pending_fk = None
        self._finite_pending_fks = set()
        self._finite_fk_wrappers = set()
        self._finite_fk_failed = False
        self._finite_shutdown_attempted = False

    async def connect(self):
        if self._finite_pending_fks or self._finite_fk_wrappers or self._finite_snapshot_wrappers:
            raise BackendCommandError('finite_fk_read_pending')
        if self._finite_fk_failed:
            raise BackendCommandError('finite_fk_audit_failed')
        if self._finite_submitted and self._client is None:
            raise BackendCommandError('finite_single_attempt_consumed')
        await super().connect()
        if not callable(getattr(self._client, 'kinematics_forward', None)):
            self._preflight_ready = False

    async def preflight(self):
        result = await super().preflight()
        reason = ('kinematics_forward_unavailable'
                  if not callable(getattr(self._client, 'kinematics_forward', None))
                  else 'finite_single_attempt_consumed' if self._finite_submitted else None)
        if reason:
            self._preflight_ready = False
            return replace(result, ready=False, reason=reason)
        return result

    async def submit_tcp(self, target, command_id):
        if self._finite_submitted:
            raise BackendCommandError('finite_single_attempt_consumed')
        started = self._clock()
        admission = await super().submit_tcp(target, command_id)
        # The original admission coroutine contains no suspension. Lease and
        # immutable target binding are installed before the pump can dispatch.
        self._finite_submitted = True
        self._finite_lease = (admission.command, started, target.model_dump_json())
        return admission

    async def set_gripper(self, value):
        raise BackendCommandError('finite_diagnostic_action_not_supported')

    async def home(self, options, on_phase):
        raise BackendCommandError('finite_diagnostic_action_not_supported')

    async def prepare(self, options, on_phase):
        raise BackendCommandError('finite_diagnostic_action_not_supported')

    async def submit_catch_up(self, accepted_command, command_id):
        raise BackendCommandError('finite_diagnostic_action_not_supported')

    async def _send_target(self, request):
        if self._finite_context is not None:
            raise BackendCommandError('finite_diagnostic_busy')
        self._finite_context = request
        try:
            await super()._send_target(request)
        except _FiniteRevoked:
            # The result was explicitly invalidated before raising through the
            # original handler. Revocation is not a new persistent pump fault.
            pass
        finally:
            self._finite_context = None
            self._finite_ticket = None
            self._finite_input_signature = None

    async def disconnect(self):
        for pending in tuple(self._finite_pending_fks | self._finite_fk_wrappers | self._finite_snapshot_wrappers):
            if not pending.done() and not pending.cancelling():
                pending.cancel()
        # A read ignoring local cancellation remains owned by this instance.
        # Do not await it, reuse it for motion, or claim remote cancellation.
        try:
            if self._finite_entered and not self._finite_shutdown_attempted:
                # A returned call can be invalidated before original acceptance
                # bookkeeping. Native entry still requires an owned stop.
                self._finite_shutdown_attempted = True
                await self.stop(StopReason.SHUTDOWN)
        finally:
            await super().disconnect()

    def _guard_request(self, client):
        request = self._finite_context
        lease = self._finite_lease
        if request is None or lease is None or request.reference != lease[0]:
            raise _MotionSnapshotFault('finite_diagnostic_not_admitted')
        if not self._motion_dispatch_is_current(request):
            if request.completion is not None:
                request.completion._invalidate()
                request.completion._resolve('invalidated', sdk_outcome='not_started',
                                            error='finite_motion_authority_changed')
            raise _FiniteRevoked('finite_motion_authority_changed')
        if request.target.model_dump_json() != lease[2]:
            raise _MotionSnapshotFault('finite_request_changed')
        if not 0 <= self._clock() - lease[1] < _LEASE_NS:
            raise _MotionSnapshotFault('finite_diagnostic_expired')
        if client is not self._client:
            raise _MotionSnapshotFault('finite_read_client_changed')
        self._check_known_snapshot_fault(client, request.generation)

    def _guard(self, client, snapshot):
        self._guard_request(client)
        self._ensure_command_snapshot_fresh(snapshot)
        if self._finite_input_signature is not None and _snapshot_signature(snapshot) != self._finite_input_signature:
            raise _MotionSnapshotFault('finite_audit_input_changed')

    async def _read_snapshot_impl(self, read, **kwargs):
        if not self.parallel_motion_reads or kwargs['read_purpose'] != 'motion':
            return await super()._read_snapshot_impl(read, **kwargs)
        client = self._client
        self._guard_request(client)
        scope = getattr(client, '_parallel_snapshot_scope', None)
        with scope(lambda: self._guard_request(client)) if callable(scope) else nullcontext():
            return await super()._read_snapshot_impl(read, **kwargs)

    def _before_snapshot_install(self, snapshot, read):
        if self.parallel_motion_reads and snapshot.read_purpose == 'motion':
            self._guard_request(read.client)
            self._ensure_command_snapshot_fresh(snapshot)
            if read.owner is not self._snapshot_calls or read.owner.closed:
                raise _MotionSnapshotFault('finite_read_client_changed')

    async def _read_snapshot_fields(self, client, latencies, deadline_ns, **kwargs):
        if not self.parallel_motion_reads or kwargs['read_purpose'] != 'motion':
            return await super()._read_snapshot_fields(client, latencies, deadline_ns, **kwargs)
        self._guard_request(client)
        async def query(name):
            self._guard_request(client)
            value = await self._timed(name, getattr(client, name), latencies, deadline_ns=deadline_ns)
            if name == 'get_kin_data' and kwargs['include_raw_kinematics'] and isinstance(value, Mapping):
                return value, copy.deepcopy(dict(value))
            return value, None
        tasks = [asyncio.create_task(query(name), name='finite-snapshot-' + name) for name in
            ('get_robot_state', 'get_estop_reason', 'get_kin_data', 'get_tcp', 'get_claw', 'get_running_motion')]
        self._finite_snapshot_wrappers.update(tasks)
        def completed(done):
            self._finite_snapshot_wrappers.discard(done)
            if not done.cancelled(): done.exception()
        for task in tasks: task.add_done_callback(completed)
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
            for task in tasks:
                if task.done() and (task.cancelled() or task.exception() is not None):task.result()
            results = [task.result() for task in tasks]
            self._guard_request(client)
            return dict(values=[item[0] for item in results], raw_kinematics=results[2][1])
        except BaseException:
            for task in tasks:
                if not task.done() and not task.cancelling():task.cancel()
            raise

    async def _select_advancing_candidate(self, client, snapshot, request, *, diagnostic_events):
        # One IK attempt; no interpolated recovery or automatic catch-up in
        # this one-window diagnostic. Normal adapter recovery is unchanged.
        return await self._solve_candidate(client, snapshot, request.target, fraction=1.,
                                           command_id=request.command_id, diagnostic_events=diagnostic_events)

    def _build_advancing_candidate(self, snapshot, target, metrics, fraction):
        selected = super()._build_advancing_candidate(snapshot, target, metrics, fraction)
        finite = build_finite_pvat_candidate(metrics.solution_q, snapshot.actual_q,
            snapshot.actual_qd, snapshot.actual_qdd, self._pvat_limits)
        return replace(selected, point=finite.point)

    async def _fk(self, client, snapshot, role, joints, calls):
        self._guard(client, snapshot)
        if self._finite_fk_failed:
            raise _MotionSnapshotFault('finite_fk_audit_failed')
        limit = 3 if self.parallel_fk and role in ('goal', 'midpoint', 'endpoint') else 1
        if len(self._finite_pending_fks) >= limit:
            raise _MotionSnapshotFault('finite_fk_read_pending')
        call = dict(role=role, method='kinematics_forward', started_ns=self._clock(),
                    outcome='started', entered=False, remote_result_unknown=False)
        calls.append(call)

        async def query():
            # A scheduled read task is another SDK boundary. Recheck here,
            # before calling the method, and support any returned Awaitable.
            try:
                self._guard(client, snapshot)
            except BaseException:
                call['outcome'] = 'not_started'
                raise
            call.update(entered=True, sdk_await_started_ns=self._clock())
            try:
                values = [float(q) for q in joints]
                guarded = getattr(client, '_kinematics_forward_guarded', None)
                if self.parallel_fk and callable(guarded):
                    return await guarded(values, lambda: self._guard(client, snapshot))
                return await client.kinematics_forward(values)
            finally:
                call['sdk_await_completed_ns'] = self._clock()

        task = asyncio.create_task(query(), name='finite-forward-kinematics')
        self._finite_pending_fk = task
        self._finite_pending_fks.add(task)

        def completed(done):
            self._finite_pending_fks.discard(done)
            self._finite_pending_fk = next(iter(self._finite_pending_fks), None)
            if not done.cancelled():
                done.exception()

        task.add_done_callback(completed)
        try:
            done, _ = await asyncio.wait({task}, timeout=_FK_TIMEOUT_S)
            if not done:
                task.cancel()
                raise TimeoutError
            raw = task.result()
            call['outcome'] = 'returned'
            self._guard(client, snapshot)
            if not isinstance(raw, Mapping):
                raise _MotionSnapshotFault('finite_fk_invalid_payload')
            try:
                return pose_from_lebai(raw)
            except BackendCommandError:
                raise _MotionSnapshotFault('finite_fk_invalid_payload') from None
        except (_MotionSnapshotFault, _FiniteRevoked):
            raise
        except asyncio.CancelledError:
            if not task.done() and not task.cancelling():
                task.cancel()
            call.update(outcome='cancelled', remote_result_unknown=call['entered'])
            raise
        except TimeoutError:
            call.update(outcome='timeout', remote_result_unknown=call['entered'])
            raise _MotionSnapshotFault('finite_fk_timeout') from None
        except Exception:
            call['outcome'] = 'error'
            raise _MotionSnapshotFault('finite_fk_call_failed') from None
        finally:
            call['completed_ns'] = self._clock()

    async def _fk_triplet(self, client, snapshot, samples, calls):
        if self._finite_pending_fks or self._finite_fk_wrappers:
            raise _MotionSnapshotFault('finite_fk_read_pending')
        tasks = [asyncio.create_task(self._fk(client, snapshot, role, joints, calls),
                    name='finite-fk-' + role) for role, joints in samples]
        self._finite_fk_wrappers.update(tasks)
        def completed(done):
            self._finite_fk_wrappers.discard(done)
            if not done.cancelled():
                done.exception()
        for task in tasks:
            task.add_done_callback(completed)
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
            # Inspect every completed failure before obtaining ordered results.
            for task in tasks:
                if task.done() and (task.cancelled() or task.exception() is not None):
                    task.result()
            return tuple(task.result() for task in tasks)
        except BaseException:
            self._finite_fk_failed = True
            for task in tasks:
                if not task.done() and not task.cancelling():
                    task.cancel()
            raise

    async def _solve_candidate(self, client, snapshot, target, **kwargs):
        event = dict(kind='finite_candidate_audit', outcome='failed', calls=[],
                     parallel_fk=self.parallel_fk,
                     assessment='reference_samples_only_not_path_or_physics',
                     command=self._finite_context.reference.__dict__ if self._finite_context else None,
                     started_ns=self._clock())
        try:
            self._guard(client, snapshot)
            self._finite_input_signature = _snapshot_signature(snapshot)
            direction = np.array(target.p) - np.array(snapshot.actual_tcp.p)
            distance = float(np.linalg.norm(direction))
            if not 0 < distance <= _MAX_TRANSLATION_M + 1e-12 or _angle(target.q, snapshot.actual_tcp.q) > 1e-6:
                raise _MotionSnapshotFault('finite_diagnostic_translation_only')
            try:
                selected = await super()._solve_candidate(client, snapshot, target, **kwargs)
            except BackendCommandError as error:
                if str(error).startswith(('finite_', 'invalid_finite_')):
                    raise _MotionSnapshotFault(str(error)) from None
                raise
            self._guard(client, snapshot)
            origin = await self._fk(client, snapshot, 'feedback', snapshot.actual_q, event['calls'])
            if (math.dist(origin.p, snapshot.actual_tcp.p) > _FK_POSITION_TOLERANCE_M
                    or _angle(origin.q, snapshot.actual_tcp.q) > _FK_ROTATION_TOLERANCE_DEG):
                raise _MotionSnapshotFault('finite_fk_feedback_mismatch')
            point = selected.point
            base, slope = _controls(np.array(snapshot.actual_qd), np.array(snapshot.actual_qdd),
                                    np.array(point.q) - np.array(snapshot.actual_q), point.horizon_s)
            middle_q = np.array(snapshot.actual_q) + np.array(
                [math.comb(5, i) * .5 ** 5 for i in range(6)]) @ (base + slope)
            if self.parallel_fk:
                goal, middle, endpoint = await self._fk_triplet(client, snapshot,
                    [('goal', selected.metrics.solution_q), ('midpoint', middle_q), ('endpoint', point.q)], event['calls'])
            else:
                goal = await self._fk(client, snapshot, 'goal', selected.metrics.solution_q, event['calls'])
            if (math.dist(goal.p, target.p) > _FK_POSITION_TOLERANCE_M
                    or _angle(goal.q, target.q) > _FK_ROTATION_TOLERANCE_DEG):
                raise _MotionSnapshotFault('finite_fk_goal_mismatch')
            unit = direction / distance

            def check_direction(pose, role):
                delta = np.array(pose.p) - np.array(origin.p)
                progress = float(delta @ unit)
                cross = float(np.linalg.norm(delta - progress * unit))
                event.setdefault('samples', []).append(dict(role=role, progress_m=progress, cross_m=cross))
                if (progress < -1e-9 or progress > distance + 1e-6
                        or cross > max(1e-6, max(0., progress) * math.tan(math.radians(_DIRECTION_ANGLE_DEG)))
                        or _angle(pose.q, origin.q) > _FK_ROTATION_TOLERANCE_DEG):
                    raise _MotionSnapshotFault('finite_fk_direction_mismatch')
                return progress

            full_progress = check_direction(goal, 'goal')
            if abs(full_progress - distance) > max(1e-6, distance * .05):
                raise _MotionSnapshotFault('finite_fk_goal_progress_mismatch')
            if not self.parallel_fk:
                middle = await self._fk(client, snapshot, 'midpoint', middle_q, event['calls'])
            check_direction(middle, 'midpoint')
            if not self.parallel_fk:
                endpoint = await self._fk(client, snapshot, 'endpoint', point.q, event['calls'])
            if check_direction(endpoint, 'endpoint') <= 1e-9:
                raise _MotionSnapshotFault('finite_fk_endpoint_no_progress')
            self._guard(client, snapshot)
            self._finite_ticket = (client, snapshot, _snapshot_signature(snapshot), selected)
            event.update(outcome='accepted', zero_terminal=True, horizon_s=point.horizon_s)
            return selected
        except BaseException as error:
            event.update(error=str(error), error_type=type(error).__name__)
            raise
        finally:
            event['completed_ns'] = self._clock()
            self._schedule_sdk_diagnostic(copy.deepcopy(event))

    def _begin_motion_epoch(self):
        ticket = self._finite_ticket
        if self._finite_entered or ticket is None:
            raise _MotionSnapshotFault('finite_sdk_entry_not_admitted')
        client, snapshot, signature, selected = ticket
        self._guard(client, snapshot)
        if self._finite_fk_failed or any(not task.done() for task in self._finite_pending_fks | self._finite_fk_wrappers):
            raise _MotionSnapshotFault('finite_fk_read_pending')
        if (_snapshot_signature(snapshot) != signature or selected.point.reference_source != 'observed_velocity'
                or selected.point.qd != (0.,) * 6 or selected.point.qdd != (0.,) * 6):
            raise _MotionSnapshotFault('finite_audit_ticket_changed')
        super()._begin_motion_epoch()
        # Reserve before native entry. A timeout, error or local cancellation
        # cannot grant a second attempt, even if remote motion is unknown.
        self._finite_entered = True
