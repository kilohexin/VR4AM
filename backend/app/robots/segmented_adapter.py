"""Opt-in translation segments with original finite audits and stop ownership.

An SDK return or settled observation is not physical stop certification.
Each segment retains the 100-ms lease, four FK samples and an 80-ms-or-less
zero-terminal joint reference. No catch-up, interpolation or queued next move.
"""
from dataclasses import dataclass, replace
from contextvars import ContextVar
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation

from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
from app.control.transactional_settings import PursuitSettings, SegmentedSettings
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter, _MotionSnapshotFault
from app.robots.lebai_finite_pvat import build_finite_pvat_candidate
from app.schemas.messages import TargetControlStatus
from app.robots.lebai_finite_pvat import _controls
from app.robots.tool_spin import tool_spin_angle,check_spin_sample

_prepare_context = ContextVar('segmented_prepare_owner', default=None)


@dataclass(frozen=True)
class SegmentStatus:
    ready: bool
    committed: object = None
    command: object = None
    fault: str | None = None
    hold: bool = False


class SegmentedLebaiAdapter(FiniteWindowDiagnosticAdapter):
    def __init__(self, settings, *, settling: PursuitSettings, client_factory, allow_tool_spin=False,
                 allow_prepare=False, allow_unsent_expiry_hold=False, offline_parallel_snapshot_reads=False, **kwargs):
        if type(offline_parallel_snapshot_reads) is not bool:
            raise ValueError('invalid_offline_parallel_snapshot_reads')
        if offline_parallel_snapshot_reads:
            if kwargs.get('backend_label')!='LEBAI_FAKE' or settings.ip!='digital-twin':
                raise ValueError('offline_parallel_snapshot_reads_offline_only')
            if 'parallel_motion_reads' in kwargs:
                raise ValueError('conflicting_parallel_snapshot_selection')
            kwargs['parallel_motion_reads']=True
        self.offline_parallel_snapshot_reads=offline_parallel_snapshot_reads
        if type(allow_unsent_expiry_hold) is not bool:
            raise ValueError('invalid_unsent_expiry_hold')
        if allow_unsent_expiry_hold and kwargs.get('backend_label')!='LEBAI_FAKE':
            raise ValueError('unsent_expiry_hold_offline_only')
        self.allow_unsent_expiry_hold=allow_unsent_expiry_hold
        if type(allow_tool_spin) is not bool:raise ValueError('invalid_tool_spin_selection')
        if type(allow_prepare) is not bool:raise ValueError('invalid_prepare_on_arm')
        self.supports_tool_spin=allow_tool_spin
        self.supports_prepare_on_arm=allow_prepare
        SegmentedSettings(settling)  # Apply the same hard bounds to injection.
        super().__init__(settings, client_factory=client_factory, **kwargs)
        self.settling = settling
        self._segment = None
        self._segment_enabled = False
        self._segment_fault = None
        self._prepare_owner = None

    async def _read_snapshot_fields(self, client, latencies, deadline_ns, **kwargs):
        if not self.offline_parallel_snapshot_reads or kwargs['read_purpose']!='state':
            return await super()._read_snapshot_fields(client,latencies,deadline_ns,**kwargs)
        owner=self._snapshot_calls
        def check():
            if (client is not self._client or owner is not self._snapshot_calls
                    or owner.closed):
                raise BackendCommandError('snapshot_read_client_changed')
            if owner.unknown_methods:
                raise BackendCommandError('snapshot_rpc_result_unknown')
        return await self._parallel_snapshot_fields(client,latencies,deadline_ns,check=check,
            include_raw_kinematics=kwargs['include_raw_kinematics'])

    def _before_snapshot_install(self, snapshot, read):
        if self.offline_parallel_snapshot_reads and snapshot.read_purpose=='state':
            if (read.client is not self._client or read.owner is not self._snapshot_calls
                    or read.owner.closed):
                raise BackendCommandError('snapshot_read_client_changed')
            if read.owner.unknown_methods:
                raise BackendCommandError('snapshot_rpc_result_unknown')
            # Ordinary feedback can finish across a normal stop, as in the
            # original reader. The parent never installs an old-generation
            # read as current motion evidence. Motion reads retain their lease.
        return super()._before_snapshot_install(snapshot,read)

    async def prepare(self, options, on_phase, *, authorize=None):
        if not self.supports_prepare_on_arm:
            raise BackendCommandError('finite_diagnostic_action_not_supported')
        if self._prepare_owner is not None:
            raise BackendCommandError('segmented_prepare_busy')
        # Explicit recovery uses the original joint motion ownership, bounds,
        # feedback stabilization, stop latches and SDK entry revocation. This
        # opt-in does not enable Home or change finite segment dispatch.
        owner = object()
        self._prepare_owner = owner
        token = _prepare_context.set((self, owner, on_phase))
        try:
            await RealLebaiAdapter.prepare(self, options, on_phase, authorize=authorize)
        finally:
            self._prepare_owner = None
            _prepare_context.reset(token)
        self._segment = None
        self._segment_enabled = False
        self._segment_fault = None

    def _begin_motion_epoch(self):
        # Recovery is an explicit joint posture request, with the original
        # Real.prepare guards. Its nested wait_for task inherits this context;
        # the independent segment pump cannot acquire this admission. Clear
        # the owner in finally so delayed child contexts never retain it.
        context = _prepare_context.get()
        if (self._prepare_owner is not None and context is not None
                and context[:2] == (self, self._prepare_owner)):
            # Recheck control's phase callback immediately at both original
            # recovery epoch entries, including the nested native-call task.
            # A stop revokes control before its recording await; it must not
            # wait for backend.stop to invalidate the backend generations.
            context[2]('homing')
            return RealLebaiAdapter._begin_motion_epoch(self)
        return super()._begin_motion_epoch()

    async def preflight(self):
        # This mode replaces only the diagnostic's once-per-connection rule.
        # All original real adapter admission and fault checks still apply.
        result = await RealLebaiAdapter.preflight(self)
        if not callable(getattr(self._client, 'kinematics_forward', None)):
            from dataclasses import replace
            self._preflight_ready = False
            return replace(result, ready=False, reason='kinematics_forward_unavailable')
        return result

    def begin_segments(self):
        self._check_submission_ready()
        if not callable(getattr(self._client, 'kinematics_forward', None)):
            raise BackendCommandError('kinematics_forward_unavailable')
        if self._segment is not None:
            transaction = self._stop_transaction
            if (transaction is None or not transaction.confirmed
                    or self._motion_write_stop_required
                    or self._motion_calls.unknown_methods):
                raise BackendCommandError('segment_previous_owner_not_stopped')
        self._segment = None
        self._segment_fault = None
        self._segment_enabled = True

    @property
    def segment_duration_s(self):
        return self.settings.control.pvat_horizon_s

    async def submit_tcp(self, target, command_id):
        raise BackendCommandError('segmented_interface_required')

    async def submit_catch_up(self, *args):
        raise BackendCommandError('segmented_interface_required')

    async def set_gripper(self, value):
        # Keep the existing claw ownership, rate limits and stop-unknown fence.
        await RealLebaiAdapter.set_gripper(self, value)

    async def submit_segment(self, target, command_id):
        if not self._segment_enabled:
            raise BackendCommandError('segment_owner_not_anchored')
        status = self.read_segment_status()
        if status.fault:
            raise BackendCommandError(status.fault)
        if not status.ready:
            raise BackendCommandError('segment_feedback_pending')
        self._check_submission_ready()
        # Reset only the per-window audit ticket. Do not reset faults, unknown
        # RPCs, the pump, accepted history, or the original stop obligation.
        self._finite_submitted = self._finite_entered = False
        self._finite_lease = self._finite_ticket = self._finite_input_signature = None
        receipt = await FiniteWindowDiagnosticAdapter.submit_tcp(self, target, command_id)
        self._segment = dict(receipt=receipt, target=target.model_copy(deep=True),
            started_ns=self._clock(), accepted=None, feedback_floor=None,
            last_sample=None, good=0, deadline=None, ready=False,
            previous_settled=status.ready and status.command is not None,
            previous_accepted=self.read_motion_view().accepted,
            previous_client=self._client,
            previous_read_owner=self._snapshot_calls,
            previous_pvat_record=self._motion_calls.last.get('move_pvat'),unsent_expired=False)
        return receipt

    async def _send_target_impl(self,request,lifecycle):
        try:
            return await super()._send_target_impl(request,lifecycle)
        except _MotionSnapshotFault as error:
            if not (self.allow_unsent_expiry_hold and self._backend_label=='LEBAI_FAKE'
                    and str(error)=='finite_diagnostic_expired'):
                raise
            # Expiry must not hide a concurrent estop/tool/read fault whose
            # check happened later in the diagnostic parent's guard order.
            self._check_known_snapshot_fault(self._client,request.generation)
            segment=self._segment
            if not (segment is not None and segment['previous_settled']
                    and self._client is segment['previous_client']
                    and segment['receipt'].command==request.reference
                    and self._finite_context is request and not self._finite_entered and not lifecycle
                    and self._motion_dispatch_is_current(request)
                    and not self._latched_fault and not self._pump.fault
                    and not self._finite_fk_failed
                    and not self._motion_calls.unknown_methods and not self._snapshot_calls.unknown_methods
                    and self._snapshot_calls is segment['previous_read_owner']
                    and not self._snapshot_calls.closed and not self._snapshot_calls.active
                    and not any(not task.done() for task in self._motion_calls.tasks
                        | self._snapshot_calls.tasks
                        | self._finite_pending_fks | self._finite_fk_wrappers | self._finite_snapshot_wrappers)
                    and self._motion_calls.last.get('move_pvat') is segment['previous_pvat_record']
                    and self.read_motion_view().accepted==segment['previous_accepted']):
                raise
            # No await between the proof and fencing this episode. The owner
            # still runs the original stop transaction before any new anchor.
            self._segment_enabled=False
            segment['unsent_expired']=True
            request.completion._resolve('handled',sdk_outcome='not_started',error='segment_unsent_expired')
            self._schedule_sdk_diagnostic(dict(kind='segment_unsent_expired',
                command=request.reference.__dict__,previous_command=segment['previous_accepted'].command.__dict__,
                sdk_outcome='not_started',requires_release=True))

    def _build_advancing_candidate(self, snapshot, target, metrics, fraction):
        finite = build_finite_pvat_candidate(metrics.solution_q, snapshot.actual_q,
            snapshot.actual_qd, snapshot.actual_qdd, self._pvat_limits)
        # Never report a full TCP acceptance when the finite endpoint is only
        # a fraction of the IK solution. The caller can explicitly choose a
        # smaller new request in a fresh owner episode after failure handling.
        if finite.progress_fraction != 1. or finite.point.q != metrics.solution_q:
            raise _MotionSnapshotFault('segment_full_endpoint_infeasible')
        # Retain the legacy joint-step/tracking checks, but reuse this exact
        # certified finite point instead of asking the diagnostic parent to
        # calculate the same Bernstein bounds again.
        selected = RealLebaiAdapter._build_advancing_candidate(self, snapshot, target, metrics, fraction)
        return replace(selected, point=finite.point)

    async def _solve_candidate(self, client, snapshot, target, **kwargs):
        try:
            if kwargs.get('fraction') != 1.:
                raise _MotionSnapshotFault('segment_no_interpolation')
            if (max(abs(v) for v in snapshot.actual_qd) > self.settling.velocity_tolerance_radps
                    or max(abs(v) for v in snapshot.actual_qdd) > 1e-6):
                raise _MotionSnapshotFault('segment_initial_not_settled')
            if self._segment is None:
                raise _MotionSnapshotFault('segment_audit_not_admitted')
            self._segment['origin_q'] = tuple(snapshot.actual_q)
            angle=(Rotation.from_quat(snapshot.actual_tcp.q).inv()*Rotation.from_quat(target.q)).magnitude()
            # A translated reference can retain numerical IK rotation residue.
            # Select spin only when its existing position-only gate applies;
            # translation retains the parent's stricter rotation audit.
            if (self.supports_tool_spin and angle>1e-9
                    and math.dist(snapshot.actual_tcp.p,target.p)<=1e-9):
                return await self._solve_spin_candidate(client,snapshot,target,**kwargs)
            return await super()._solve_candidate(client, snapshot, target, **kwargs)
        except BackendCommandError as error:
            # Original solver wraps constructor failures as candidate errors;
            # this new hard failure must retain the outside-lock stop path.
            if str(error).startswith('segment_'):
                raise _MotionSnapshotFault(str(error)) from None
            raise

    async def _solve_spin_candidate(self,client,snapshot,target,**kwargs):
        event=dict(kind='finite_tool_spin_audit',outcome='failed',calls=[],started_ns=self._clock(),
            assessment='reference_samples_only_not_path_or_physics')
        try:
            self._guard(client,snapshot)
            from app.commissioning.finite_diagnostic import _snapshot_signature
            self._finite_input_signature=_snapshot_signature(snapshot)
            angle=tool_spin_angle(snapshot.actual_tcp,target)
            try:
                selected=await RealLebaiAdapter._solve_candidate(self,client,snapshot,target,**kwargs)
            except BackendCommandError as error:
                if str(error).startswith(('finite_','invalid_finite_')):
                    raise _MotionSnapshotFault(str(error)) from None
                raise
            self._guard(client,snapshot)
            origin=await self._fk(client,snapshot,'feedback',snapshot.actual_q,event['calls'])
            feedback_angle=(Rotation.from_quat(origin.q).inv()*Rotation.from_quat(snapshot.actual_tcp.q)).magnitude()
            if math.dist(origin.p,snapshot.actual_tcp.p)>.0001 or feedback_angle>min(math.radians(.001),abs(angle)*.05):
                raise _MotionSnapshotFault('finite_fk_feedback_mismatch')
            goal=await self._fk(client,snapshot,'goal',selected.metrics.solution_q,event['calls'])
            check_spin_sample(origin,goal,angle,complete=True)
            point=selected.point
            base,slope=_controls(np.array(snapshot.actual_qd),np.array(snapshot.actual_qdd),
                np.array(point.q)-np.array(snapshot.actual_q),point.horizon_s)
            middle_q=np.array(snapshot.actual_q)+np.array([math.comb(5,i)*.5**5 for i in range(6)])@(base+slope)
            middle=await self._fk(client,snapshot,'midpoint',middle_q,event['calls'])
            check_spin_sample(origin,middle,angle)
            endpoint=await self._fk(client,snapshot,'endpoint',point.q,event['calls'])
            check_spin_sample(origin,endpoint,angle,complete=True)
            self._guard(client,snapshot)
            self._finite_ticket=(client,snapshot,_snapshot_signature(snapshot),selected)
            event.update(outcome='accepted',zero_terminal=True,horizon_s=point.horizon_s,tool_axis='Z',angle_rad=angle)
            return selected
        except BaseException as error:
            event.update(error=str(error),error_type=type(error).__name__)
            raise
        finally:
            event['completed_ns']=self._clock()
            self._schedule_sdk_diagnostic(copy.deepcopy(event))

    def read_segment_status(self):
        view, now = self.read_motion_view(), self._clock()
        fault = view.latched_fault or view.pump_fault or self._segment_fault
        if fault:
            self._segment_fault = fault
            return SegmentStatus(False, fault=fault)
        if (not view.client_attached or not view.preflight_ready or view.stop_in_progress
                or not view.is_current(view.generation)):
            self._segment_fault = 'segment_readiness_lost'
            return SegmentStatus(False, fault=self._segment_fault)
        segment = self._segment
        if segment is None:
            return SegmentStatus(self._segment_enabled)
        if segment['unsent_expired']:
            return SegmentStatus(False,hold=True)
        receipt = segment['receipt']
        done = receipt.completion.result
        if receipt.completion.invalidated or not view.is_current(receipt.command.generation):
            self._segment_fault = 'segment_authority_revoked'
        elif now - segment['started_ns'] >= self.settling.budget_ms * 1_000_000 and not segment['ready']:
            self._segment_fault = 'segment_feedback_expired'
        elif done is None:
            return SegmentStatus(False)
        elif (done.command != receipt.command or done.invalidated or done.outcome != 'sdk_returned'
                or done.sdk_outcome != 'returned' or done.pvat_mode != 'advance'
                or done.accepted_command != receipt.command or done.error is not None
                or view.accepted is None or view.accepted.command != receipt.command
                or view.accepted.position != segment['target'].p
                # Pose/Fake recovery normalizes quaternions. Equivalent
                # rotations can therefore differ by a floating-point ULP;
                # retain exact command/position identity and a 1e-12-rad
                # representation allowance, separate from motion tolerances.
                or (Rotation.from_quat(view.accepted.orientation).inv()
                    * Rotation.from_quat(segment['target'].q)).magnitude() > 1e-12):
            self._segment_fault = 'segment_acceptance_failed'
        elif segment['accepted'] is not None and view.accepted != segment['accepted']:
            self._segment_fault = 'segment_acceptance_changed'
        if self._segment_fault:
            return SegmentStatus(False, fault=self._segment_fault)
        if segment['accepted'] is None:
            segment['accepted'] = view.accepted
            # Conservative floor: even a prompt SDK reply cannot grant another
            # move before this entire reference horizon plus fresh feedback.
            segment['feedback_floor'] = now + int(self.settings.control.pvat_horizon_s * 1e9)
            segment['deadline'] = segment['feedback_floor'] + self.settling.max_observation_gap_ms * 1_000_000
        sample = view.sample
        valid = sample is not None and (
            type(sample.captured_ns) is int and type(sample.read_completed_ns) is int
            and type(sample.generation) is int and sample.generation == receipt.command.generation
            and sample.read_purpose in {'state', 'motion'}
            and segment['feedback_floor'] < sample.captured_ns <= sample.read_completed_ns <= now
            and now - sample.captured_ns <= self.settling.max_observation_age_ms * 1_000_000
            and (segment['last_sample'] is None or sample.captured_ns >= segment['last_sample'])
            and sample.actual_qdd is not None
            and all(len(v) == 6 for v in (sample.actual_q, sample.actual_qd, sample.actual_qdd))
            and all(math.isfinite(x) for v in (sample.actual_q, sample.actual_qd, sample.actual_qdd) for x in v))
        if valid and (segment['last_sample'] is None or sample.captured_ns > segment['last_sample']):
            # The tolerance must distinguish this tiny target from its origin.
            # A broad global tolerance must not declare unchanged feedback an
            # arrival when the whole segment is smaller than that tolerance.
            tolerance = min(self.settling.position_tolerance_rad,
                .1 * max(abs(a-b) for a,b in zip(segment['accepted'].solution_q, segment['origin_q'])))
            close = (max(abs(a-b) for a,b in zip(sample.actual_q, segment['accepted'].solution_q))
                     <= tolerance
                     and max(abs(v) for v in sample.actual_qd) <= self.settling.velocity_tolerance_radps
                     and max(abs(v) for v in sample.actual_qdd) <= 1e-6)
            previous = segment['last_sample']
            consecutive = previous is None or 0 < sample.captured_ns - previous <= self.settling.max_observation_gap_ms * 1_000_000
            segment['good'] = (segment['good'] if consecutive else 0) + 1 if close else 0
            segment['last_sample'] = sample.captured_ns
            segment['deadline'] = sample.captured_ns + self.settling.max_observation_gap_ms * 1_000_000
            segment['ready'] = segment['good'] >= self.settling.settle_samples
        elif not valid:
            segment['ready'] = False
            segment['good'] = 0
        if now > segment['deadline']:
            from dataclasses import asdict
            self._schedule_sdk_diagnostic(dict(kind='segment_feedback_timeout',now_ns=now,
                floor_ns=segment['feedback_floor'],deadline_ns=segment['deadline'],
                last_sample=segment['last_sample'],good=segment['good'],
                origin_q=segment.get('origin_q'),accepted=asdict(segment['accepted']),
                sample=asdict(sample) if sample else None))
            self._segment_fault = 'segment_feedback_expired'
            return SegmentStatus(False, fault=self._segment_fault)
        return SegmentStatus(segment['ready'], segment['target'].model_copy(deep=True), receipt.command)

    def segment_display(self):
        """Copy cached display facts; never collect a receipt or grant motion."""
        view = self.read_motion_view()
        segment = self._segment
        observed = min(segment['good'], self.settling.settle_samples) if segment else 0
        if view.latched_fault or view.pump_fault or self._segment_fault:
            phase = 'fault'
        elif not self._segment_enabled:
            phase = 'inactive'
        elif segment is None:
            phase = 'ready'
        elif segment['accepted'] is None:
            phase = 'awaiting_reply'
        elif segment['ready']:
            phase = 'ready'
        elif self._clock() <= segment['feedback_floor']:
            phase = 'awaiting_window'
        else:
            phase = 'awaiting_feedback'
        return TargetControlStatus(policy='segmented_experimental', phase=phase,
            tool_spin_supported=self.supports_tool_spin,
            samples_observed=observed, samples_required=self.settling.settle_samples,
            hardware_verified=False)
