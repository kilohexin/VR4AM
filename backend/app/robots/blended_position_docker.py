"""Explicit Docker-only port for the unregistered blended session candidate."""
import asyncio
from dataclasses import dataclass
import math

from app.control.blended_position_session import BlendFeedback
from app.control.finite_position_types import INPUT_NS
from app.robots.base import StopReason
from app.robots.blended_position_path import audit_blended_path
from app.robots.lebai_codec import joint_vector, pose_to_lebai
from app.schemas.messages import Pose


@dataclass(frozen=True)
class DockerBlendPlan:
    target: object
    pose: dict
    audit: object
    binding: object
    start: tuple
    joints: tuple = ()


class DockerBlendPort:
    policy = 'docker_blended_candidate'
    # Known rejection of an unsent plan is a local motion boundary. Native
    # submission, unknown results and feedback failures still use fault stops.
    audit_boundaries=frozenset({'blend_joint_jump','blend_joint_limit','blend_workspace',
        'blend_ik_unreachable','blend_singular_path','blend_rotation_range'})

    def __init__(self, adapter, binding, *, authorize):
        from app.robots.finite_position_docker import DockerFinitePositionAdapter
        if (type(adapter) is not DockerFinitePositionAdapter or adapter.settings.ip != '127.0.0.1'
                or binding is not adapter.position_binding):
            raise ValueError('blend_docker_adapter_required')
        self.adapter, self.binding, self.authorize = adapter, binding, authorize
        self.client = adapter._client
        self.generation = adapter._pump.generation
        self.ids = set()
        self.sample = None
        self.orientation = None
        self.plan = None
        self.model_sha = adapter.position_model.model_sha256
        self.closed = False
        self._page_owner=None

    def begin_cycle(self):
        self.check()
        a=self.adapter
        if a._docker_entry_owner is not None:raise ValueError('position_busy')
        self._page_owner=object()
        a._docker_entry_owner=self._page_owner
        a._docker_page_reads.clear()

    def end_cycle(self):
        if self._page_owner is not None:
            self.adapter._release_page_reads(self._page_owner)
            self._page_owner=None

    def check(self):
        a = self.adapter
        self.authorize()
        if (self.closed or a._client is not self.client or a._pump.generation != self.generation
                or a.position_model.model_sha256 != self.model_sha):
            raise ValueError('blend_connection_revoked')
        a._position_health(self.binding)

    async def read(self, pending):
        self.check()
        if any(ident not in self.ids for ident in pending):
            raise ValueError('blend_foreign_task')
        finished = []
        a = self.adapter
        for ident in pending:
            state = await a._position_math('get_motion_state', lambda:self.client.get_motion_state(ident), self.check)
            if state == 'FINISHED':
                finished.append(ident)
            elif state not in ('WAIT', 'RUNNING'):
                raise ValueError('blend_task_unknown')
        s = await a._read_snapshot(read_purpose='blended_progress')
        self.check()
        if not 0 <= a._clock()-s.captured_ns < INPUT_NS:
            raise ValueError('blend_feedback_expired')
        a._position_feedback_limits(s)
        if str(s.robot_state) not in ('IDLE', 'MOVING'):
            raise ValueError('blend_robot_state')
        if s.running_motion is not None and s.running_motion not in self.ids:
            raise ValueError('blend_foreign_task')
        if any(not lo <= x <= hi for x, lo, hi in zip(s.actual_tcp.p, a.settings.startup_tcp_min_m, a.settings.startup_tcp_max_m)):
            raise ValueError('blend_workspace')
        if self.sample is None:
            if (str(s.robot_state) != 'IDLE' or not a._preflight_from_snapshot(s).ready
                    or any(abs(v)>1e-8 for rates in (s.actual_qd,s.target_qd,s.actual_qdd,s.target_qdd) for v in rates)):
                raise ValueError('blend_preflight')
            self.orientation = tuple(s.actual_tcp.q)
            self._initialize_reference(s)
        self._validate_pose(s)
        self.sample = s
        a._docker_display_sample = (self.client, self.binding, self.generation, s)
        stationary = all(abs(v) <= .002 for rates in (s.actual_qd, s.target_qd) for v in rates)
        return BlendFeedback(self._coordinates(s), s.captured_ns, tuple(finished),
                             str(s.robot_state) == 'IDLE', stationary)

    def _initialize_reference(self,s):pass

    def _coordinates(self,s):return tuple(s.actual_tcp.p)

    def _validate_pose(self,s):
        from app.robots.finite_position_plan import angle
        if angle(s.actual_tcp.q, self.orientation) > math.radians(.2):
            raise ValueError('blend_orientation_changed')

    async def audit(self, target):
        self.check()
        a = self.adapter
        if self.sample is None:
            raise ValueError('blend_feedback_missing')
        start = tuple(self.sample.actual_tcp.p)
        points = [tuple(x+(y-x)*f for x, y in zip(start, target.position)) for f in (0., .25, .5, .75, 1.)]
        joints = [tuple(self.sample.actual_q)]
        async with asyncio.timeout(1.):
            for point in points[1:]:
                pose = pose_to_lebai(Pose(p=point, q=self.orientation))
                raw = await a._position_math('kinematics_inverse',
                    lambda:self.client.kinematics_inverse(pose, list(joints[-1])), self.check)
                if raw is None:
                    raise ValueError('blend_ik_unreachable')
                joints.append(joint_vector(raw, 'blend_ik_invalid'))
            audit = await asyncio.to_thread(audit_blended_path, points, joints, self.orientation,
                                           model=a.position_model, settings=a.settings)
        self.check()
        self.plan = DockerBlendPlan(target, pose_to_lebai(Pose(p=target.position, q=self.orientation)), audit, self.binding, start,tuple(joints))
        return self.plan

    async def submit(self, plan, *, entering):
        self.check()
        if plan is not self.plan or plan.binding is not self.binding:
            raise ValueError('blend_plan_obsolete')
        self.plan = None
        a, client = self.adapter, self.client
        entered = False
        async def send():
            nonlocal entered
            self.check()
            s = self.sample
            if s is None or not 0 <= a._clock()-s.captured_ns < INPUT_NS:
                raise ValueError('blend_feedback_expired')
            a._position_feedback_limits(s)
            delta = tuple(y-x for x,y in zip(plan.start,plan.target.position))
            length2 = sum(x*x for x in delta)
            actual = self._coordinates(s)
            projection = sum((x-y)*d for x,y,d in zip(actual,plan.start,delta))/length2
            nearest = tuple(x+min(1.,max(0.,projection))*d for x,d in zip(plan.start,delta))
            if math.dist(actual,nearest) > .0001:
                raise ValueError('blend_path_changed')
            entering()
            if not a.position_busy:
                a._begin_motion_epoch()
            a.position_busy = True
            a._motion_accepted = a._motion_write_stop_required = True
            entered = True
            # Deliberately Docker-only: the real _WebClient does not expose
            # movel. Preserve its entry/unknown accounting for fault cleanup.
            client.move_entries += 1
            client.move_returned = False
            try:
                ident = await client.native.movel(plan.pose, plan.audit.acceleration, plan.audit.speed, 0., .001)
                client.move_returned = True
                return ident
            except BaseException:
                client.move_unknown = True
                raise
        async with a._sdk_lock:
            ident = await a._motion_calls.call('movel', send, .2,
                revoked=lambda:self.closed or a.position_aborted,
                entered=lambda:entered)
        if type(ident) is not int or ident <= 0 or ident in self.ids:
            a._latched_fault = 'stop_unverified'
            raise ValueError('blend_reply_unknown')
        self.ids.add(ident)
        return ident

    def settled(self):
        """Called only after the session's all-ID + continuous quiet gate."""
        self.check()
        a = self.adapter
        a.position_busy = False
        a._motion_accepted = a._motion_write_stop_required = False
        a._docker_display_sample = None

    async def stop(self):
        self.closed = True
        a = self.adapter
        try:
            await a.stop(StopReason.FAULT)
            return not a._latched_fault and not a._motion_calls.unknown_methods
        finally:
            a.position_busy = False
            a._docker_display_sample = None
