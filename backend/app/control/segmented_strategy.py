"""Latest-input-only translation policy; no autonomous motion or recovery."""
import copy
import math
import numpy as np
from scipy.spatial.transform import Rotation

from app.control.target_strategy import TargetPlan, TargetUpdate
from app.robots.base import BackendCommandError
from app.schemas.messages import Pose


class SegmentedTargetStrategy:
    def __init__(self, backend, limits, *, context):
        if not all(callable(getattr(backend, method, None)) for method in
                   ('submit_segment', 'read_segment_status', 'begin_segments')):
            raise ValueError('segmented_backend_required')
        self.backend, self.limits, self.context = backend, limits, context
        self.epoch = None
        self.pose = self.latest_requested = self.prepared = self.receipt = None
        self.committed_command = None
        self.fault = None
        self.halted = True

    def _authorized(self):
        context = self.context()
        return context.generation == self.epoch and context.active and context.authorized

    def on_anchor(self, pose, context):
        if (self.epoch is not None and context.generation <= self.epoch):
            raise BackendCommandError('new_epoch_required')
        current = self.context()
        if not (context.active and context.authorized and current.active and current.authorized
                and current.generation == context.generation):
            raise BackendCommandError('segment_authority_unavailable')
        self.backend.begin_segments()
        self.epoch, self.pose = context.generation, pose.model_copy(deep=True)
        self.latest_requested = self.prepared = self.receipt = self.committed_command = None
        self.fault = None
        self.halted = False

    def on_revoke(self, generation):
        self.halted = True
        self.prepared = self.latest_requested = None

    def prepare(self, previous, requested, dt, *, limiter):
        self.prepared = None
        self.latest_requested = requested.model_copy(deep=True)
        if self.halted or not self._authorized():
            return TargetPlan('wait', None)
        status = self.backend.read_segment_status()
        if status.fault:
            self.fault = status.fault
            self.halted = True
            return TargetPlan('wait', None)
        if not status.ready:
            return TargetPlan('wait', None)
        if status.command is not None and status.command != self.committed_command:
            return TargetPlan('wait', None)
        trial = copy.deepcopy(self.limits)
        trial.clear()
        trial.set_pose_anchor(self.pose)
        translation = Pose(p=requested.p, q=self.pose.q)
        if getattr(self.backend,'supports_tool_spin',False) and math.dist(requested.p,self.pose.p)<=1e-9:
            local=(Rotation.from_quat(self.pose.q).inv()*Rotation.from_quat(requested.q)).as_rotvec()
            if np.linalg.norm(local[:2])<=1e-7 and abs(local[2])>1e-9:
                # Plan below the adapter's .1-degree snapshot hard limit:
                # a settled numerical IK endpoint need not equal the prior
                # accepted quaternion bit for bit. Hard audits stay unchanged.
                angle=max(-math.radians(.09),min(math.radians(.09),local[2]))
                translation=Pose(p=self.pose.p,q=tuple((Rotation.from_quat(self.pose.q)*Rotation.from_rotvec((0,0,angle))).as_quat()))
        point = trial.limit_motion(self.pose, translation, self.backend.segment_duration_s)
        delta = tuple(a-b for a,b in zip(point.p, self.pose.p))
        distance = math.sqrt(sum(v*v for v in delta))
        rotated=(Rotation.from_quat(point.q).inv()*Rotation.from_quat(self.pose.q)).magnitude()>1e-9
        if distance <= 1e-9 and not rotated:
            return TargetPlan('wait', None)
        if distance > .0002:
            point = Pose(p=tuple(a + v * (.0002/distance) for a,v in zip(self.pose.p,delta)), q=self.pose.q)
        plan = TargetPlan('candidate', point.model_copy(deep=True),
            (self.epoch, point.model_copy(deep=True)))
        self.prepared = plan
        return plan

    async def dispatch(self, plan, command_id, *, backend):
        if self.halted or not self._authorized() or plan.kind == 'wait':
            return None
        if backend is not self.backend or plan is not self.prepared or plan.payload != (self.epoch, plan.pose):
            raise BackendCommandError('segment_plan_obsolete')
        self.prepared = None
        self.receipt = await self.backend.submit_segment(plan.pose, command_id)
        return None

    def collect(self, context):
        idle = TargetUpdate(context.generation)
        if context.generation != self.epoch or not self._authorized():
            return idle
        if self.fault is not None:
            return TargetUpdate(context.generation, fault=self.fault)
        if self.halted:
            return idle
        status = self.backend.read_segment_status()
        if status.fault:
            self.fault = status.fault
            self.halted = True
            self.prepared = None
            return TargetUpdate(context.generation, fault=status.fault)
        if status.hold:
            self.halted=True
            self.prepared=self.latest_requested=None
            return TargetUpdate(context.generation,hold=True)
        if status.committed is not None and status.command != self.committed_command:
            self.pose = status.committed.model_copy(deep=True)
            self.committed_command = status.command
            return TargetUpdate(context.generation, committed=self.pose.model_copy(deep=True))
        return idle

    def status_message(self):
        """Display hook only: does not call the mutating feedback gate."""
        status = self.backend.segment_display()
        if self.fault or status.phase == 'fault':
            return status.model_copy(update={'phase':'fault'})
        if self.halted or not self._authorized():
            return status.model_copy(update={'phase':'inactive'})
        return status
