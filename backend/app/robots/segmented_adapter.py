"""Opt-in translation segments with original finite audits and stop ownership.

An SDK return or settled observation is not physical stop certification.
Each segment retains the 100-ms lease, four FK samples and an 80-ms-or-less
zero-terminal joint reference. No catch-up, interpolation or queued next move.
"""
from dataclasses import dataclass
import math

from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
from app.control.transactional_settings import PursuitSettings, SegmentedSettings
from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter, _MotionSnapshotFault
from app.robots.lebai_finite_pvat import build_finite_pvat_candidate


@dataclass(frozen=True)
class SegmentStatus:
    ready: bool
    committed: object = None
    command: object = None
    fault: str | None = None


class SegmentedLebaiAdapter(FiniteWindowDiagnosticAdapter):
    def __init__(self, settings, *, settling: PursuitSettings, client_factory, **kwargs):
        SegmentedSettings(settling)  # Apply the same hard bounds to injection.
        super().__init__(settings, client_factory=client_factory, **kwargs)
        self.settling = settling
        self._segment = None
        self._segment_enabled = False
        self._segment_fault = None

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
            last_sample=None, good=0, deadline=None, ready=False)
        return receipt

    def _build_advancing_candidate(self, snapshot, target, metrics, fraction):
        finite = build_finite_pvat_candidate(metrics.solution_q, snapshot.actual_q,
            snapshot.actual_qd, snapshot.actual_qdd, self._pvat_limits)
        # Never report a full TCP acceptance when the finite endpoint is only
        # a fraction of the IK solution. The caller can explicitly choose a
        # smaller new request in a fresh owner episode after failure handling.
        if finite.progress_fraction != 1. or finite.point.q != metrics.solution_q:
            raise _MotionSnapshotFault('segment_full_endpoint_infeasible')
        return super()._build_advancing_candidate(snapshot, target, metrics, fraction)

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
            return await super()._solve_candidate(client, snapshot, target, **kwargs)
        except BackendCommandError as error:
            # Original solver wraps constructor failures as candidate errors;
            # this new hard failure must retain the outside-lock stop path.
            if str(error).startswith('segment_'):
                raise _MotionSnapshotFault(str(error)) from None
            raise

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
                or view.accepted.tcp != segment['target']):
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
            self._segment_fault = 'segment_feedback_expired'
            return SegmentStatus(False, fault=self._segment_fault)
        return SegmentStatus(segment['ready'], segment['target'].model_copy(deep=True), receipt.command)
