"""One optional, owned diagnostic read. Never changes an adapter snapshot.

The reference timer never awaits this reader or its cancellation. A pending or
failed reader closes subsequent read admission; late data cannot restore it.
This is cooperative asyncio isolation, not a real-time scheduling guarantee.
"""
import asyncio
import copy
import time
from dataclasses import dataclass
from app.commissioning.native_jog_packet import _integer, _duration
from app.commissioning.native_jog_site_scope import permission

from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient
from app.robots.lebai_codec import joint_vector, pose_from_lebai


class _ObservationReadOwner(BoundedStopOnlyClient):
    READS = frozenset({'get_kin_data'})


@dataclass(frozen=True)
class MotionReference:
    started_ns: int
    duration_s: float
    authorized: object

    def __post_init__(self):
        _integer(self.started_ns)
        _duration(self.duration_s)
        if not callable(self.authorized):
            raise ValueError('motion_reference_authorization_required')


class MotionObservation:
    def __init__(self, proxy, *, reference=None):
        if proxy.clock != proxy.deadline_clock:
            raise ValueError('motion_observation_requires_one_clock')
        if reference is not None and type(reference) is not MotionReference:
            raise ValueError("motion_reference_required")
        self.reference = reference
        self.proxy = proxy
        self.client = proxy.client
        self.clock = proxy.clock
        self.active = False
        self.used = False
        self.task = None
        self.call_index = None
        self.read_owner = _ObservationReadOwner(
            self.client, lambda: None, self._authorize, proxy.owner,
            deadline_clock=self.clock)
        self.read_owner.read_calls = proxy.read_calls
        self.result = dict(enabled=True, max_native_entries=1, outcome='not_started',
                           sample_offset_fraction=.5,
                           event_clock='time.perf_counter_ns' if self.clock is time.perf_counter_ns else 'injected',
                           sample=None, within_reference=False,
                           physical_motion_verified=False)

    def _authorize(self):
        self.proxy.read_guard()
        if self.reference is None:
            admitted = (self.proxy.pvat_entries == 1
                and self.proxy.pvat_call.get('outcome') == 'returned'
                and self.proxy.pending_pvat is not None and self.proxy.pending_pvat.done())
        else:
            permission(self.reference.authorized)
            admitted = True
        if (not self.active or self.proxy.client is not self.client
                or self.proxy.sdk_stop_entries or not admitted
                or self.clock() >= self.reference_end_ns):
            raise ValueError('motion_observation_permission_revoked')

    def _reference(self):
        if self.reference is not None:
            return self.reference.started_ns, self.reference.duration_s
        return self.proxy.pvat_started_ns, self.proxy.pvat_horizon_s

    def start(self):
        if self.used:
            raise ValueError('motion_observation_already_used')
        self.used = True
        started, duration = self._reference()
        self.reference_end_ns = started + round(duration * 1e9)
        self.active = True
        self.read_owner.read_deadline_ns = min(
            self.proxy.read_deadline_ns, self.reference_end_ns)
        try:
            self.proxy.require_clear_read_gate()
            self._authorize()
        except Exception as error:
            self.active = False
            self.result.update(outcome='not_admitted', error_type=type(error).__name__)
            return
        self.task = asyncio.create_task(self._take(), name='finite-motion-observation')
        self.task.add_done_callback(lambda done: None if done.cancelled() else done.exception())

    async def _take(self):
        try:
            self._authorize()
            started, duration = self._reference()
            midpoint = started + round(duration * .5e9)
            while (remaining := (midpoint-self.clock())/1e9) > 0:
                await asyncio.sleep(remaining)
                self._authorize()
            self.read_owner.read_deadline_ns = min(
                self.read_owner.read_deadline_ns,self.clock()+50_000_000)
            self.result['read_requested_ns'] = self.clock()
            self.call_index = len(self.proxy.read_calls)
            raw = await self.read_owner.read('get_kin_data', self.client.get_kin_data)
            self._authorize()
            sample = dict(
                actual_q=list(joint_vector(raw['actual_joint_pose'], 'observation_q')),
                actual_qd=list(joint_vector(raw['actual_joint_speed'], 'observation_qd')),
                actual_qdd=list(joint_vector(raw['actual_joint_acc'], 'observation_qdd')),
                actual_tcp=pose_from_lebai(raw['actual_tcp_pose']).model_dump(mode='json'),
                target_q=list(joint_vector(raw['target_joint_pose'], 'observation_target_q')),
                target_tcp=pose_from_lebai(raw['target_tcp_pose']).model_dump(mode='json'))
            # Validation consumes time too. Never install a late sample.
            self.read_owner.read_guard()
            self._authorize()
            completed = self.clock()
            if completed >= min(self.reference_end_ns,self.read_owner.read_deadline_ns):
                raise ValueError('motion_observation_late_sample')
            self.result.update(outcome='returned', read_completed_ns=completed,
                               sample=sample, within_reference=True)
        except BaseException as error:
            self.read_owner.read_gate_closed = True
            self._preserve_unknown()
            if self.active:
                self.result.update(outcome='error', error_type=type(error).__name__)
            if isinstance(error, (KeyboardInterrupt, SystemExit)):
                raise

    def revoke(self):
        """No await: a slow native read cannot own the stop start time."""
        if not self.active:
            return
        self.active = False
        if self.task is None or self.task.done():
            return
        self.result.update(outcome='revoked', sample=None, within_reference=False)
        item = self._call()
        native = self.read_owner.pending_read
        if item is not None:
            self.read_owner.read_gate_closed = True
            self._preserve_unknown()
        self.task.cancel()
        if native is not None and not native.done() and not native.cancelling():
            native.cancel()  # Never wait for acknowledgement.

    def _preserve_unknown(self):
        item = self._call()
        native = self.read_owner.pending_read
        if item is not None and item['entered']:
            unknown = native is None or not native.done() or native.cancelled()
            item['remote_result_unknown'] |= unknown
            self.read_owner.read_remote_result_unknown |= unknown

    def _call(self):
        if self.call_index is not None and self.call_index < len(self.proxy.read_calls):
            return self.proxy.read_calls[self.call_index]
        return None

    @property
    def blocks_reads(self):
        native = self.read_owner.pending_read
        return (self.read_owner.read_gate_closed or self.read_owner.read_remote_result_unknown
                or native is not None and not native.done())

    def snapshot(self):
        item = self._call()
        native = self.read_owner.pending_read
        return copy.deepcopy(dict(self.result,
            native_entries=int(item is not None and item['entered']),
            remote_result_unknown=self.read_owner.read_remote_result_unknown,
            pending_native=native is not None and not native.done(),
            read_gate_closed=self.read_owner.read_gate_closed))
