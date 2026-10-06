"""TEST ONLY: original RobotControl.tick, with transactional motion dispatch.

The limiter seam captures the original filtered input and returns the committed
pose as a hold value. Thus the original tick's optimistic last_target assignment
cannot advance history. Actual candidate generation happens at the backend seam,
after the original recording/freshness guards. Original target samples describe
the hold value; candidate_inputs/requests below are separate experiment traces.
No production module imports this policy and no websocket session is replaced.
"""

from app.control.robot_control import RobotControl
from app.control.safety import SafetyViolation
from app.robots.base import BackendCommandError
from app.schemas.messages import TeleopMode
from tests.control.braking_approach_probe import InfeasibleApproach
from tests.control.candidate_commit_probe import CandidateCoordinator
from tests.control.catch_up_probe import CatchUpProbe
from tests.robots.test_vr_profile_experiments import limiter


class InputGate:
    def __init__(self, owner):
        self.owner = owner
        self.limits = limiter()
        self.anchor_version = 0

    def __getattr__(self, name):
        return getattr(self.limits, name)

    def set_pose_anchor(self, pose):
        self.limits.set_pose_anchor(pose)
        self.anchor_version += 1
        self.owner.latest_requested = None

    def limit_motion(self, previous, requested, dt):
        assert dt == .02
        self.owner.latest_requested = requested.model_copy(deep=True)
        return previous.model_copy(deep=True)


class DispatchBackend:
    def __init__(self, owner, backend):
        self.owner, self.backend = owner, backend

    def __getattr__(self, name):
        return getattr(self.backend, name)

    async def command_tcp(self, target, command_id):
        # target is the committed hold pose, not a candidate to send.
        await self.owner.dispatch(command_id)


class TransactionalControl(RobotControl):
    def __init__(self, *, backend, **kwargs):
        self.adapter = backend
        self.coordinator = None
        self.pursuit = None
        self.latest_requested = None
        self.requests = []
        self.candidate_inputs = []
        self.exits = []
        self.halted = False
        self._captured_anchor_version = 0
        gate = InputGate(self)
        super().__init__(backend=DispatchBackend(self, backend), limiter=gate, **kwargs)

    def input_authorized(self):
        received = self.latest.snapshot()
        return (received is not None
                and self._target_frame_is_current(received, self.control_generation)
                and received.received_ns <= self.clock.now_ns()
                and received.frame.right.grip and received.frame.tracking_valid
                and received.frame.visibility == 'visible'
                and (self._last_frame_id is None
                     or received.frame.session_id == self._last_frame_id[0]))

    def pump_current(self, generation):
        return self.adapter._pump.is_current(generation) and self.adapter.pump_fault is None

    def capture_epoch(self):
        if (not self.input_authorized()
                or self.limiter.anchor_version == self._captured_anchor_version):
            return
        if self.coordinator is None:
            self.coordinator = CandidateCoordinator(self.adapter, self.limiter.limits,
                self.last_target,
                authority=lambda: (self.control_generation, self.input_authorized()),
                generation_current=self.pump_current)
        else:
            self.coordinator.settle()
            self.coordinator.reanchor(self.last_target)
        self.pursuit = None
        self.halted = False
        self.latest_requested = None
        self._captured_anchor_version = self.limiter.anchor_version

    async def collect(self):
        if self.coordinator is None or self.halted:
            return False
        settled = self.coordinator.settle()
        if settled == 'committed':
            self.last_target = self.coordinator._pose.model_copy(deep=True)
        if (self.adapter.pump_fault is not None and self.mode is TeleopMode.ACTIVE
                and self._fault is None):
            await self._enter_fault(str(self.adapter.pump_fault))
            return True
        if not self.input_authorized():
            # Let the unchanged tick handle Grip/session/stale input transitions.
            return False
        if self.coordinator.frozen_reason and self.pursuit is None:
            if self.coordinator.frozen_reason != 'catch_up':
                await self._enter_fault('candidate_dispatch_failed')
                return True
            receipt = self.requests[-1][1]
            done = receipt.completion.result
            accepted = self.adapter._accepted_target_ref
            if (done is None or done.accepted_command != accepted or accepted is None
                    or not self.pump_current(accepted.generation)):
                await self._enter_fault('candidate_dispatch_failed')
                return True
            epoch = self.control_generation
            self.pursuit = CatchUpProbe(self.adapter, accepted,
                self.adapter._last_accepted_solution_q, clock=self.clock.now_ns,
                authorized=lambda: self.input_authorized() and self.control_generation == epoch,
                generation_current=self.pump_current)
        if self.pursuit is not None:
            state = self.pursuit.settle()
            snapshot = self.adapter._snapshot
            if (state == 'pursuing' and snapshot is not None
                    and snapshot.captured_ns > self.pursuit.last_sample_ns):
                state = self.pursuit.observe(snapshot)
            if state in {'observed_settled', 'budget_expired'}:
                await self.end_pursuit(state)
                return True
            if state in {'dispatch_failed', 'admission_failed', 'authority_revoked'}:
                await self._enter_fault('candidate_dispatch_failed')
                return True
        return False

    async def end_pursuit(self, state):
        self.exits.append(state)
        self.halted = True
        await self.disarm()

    async def dispatch(self, command_id):
        if not self.input_authorized() or self.halted:
            return
        if self.pursuit is not None:
            # Gripper/recording awaits can consume the budget after collect().
            # Stop after super().tick() returns, so its optimistic assignment
            # cannot resurrect last_target after disarm cleared the anchor.
            if self.pursuit._check() != 'pursuing':
                return
            if self.pursuit.pending is not None:
                return
            receipt = await self.pursuit.submit(command_id)
            self.requests.append(('catch_up', receipt))
        else:
            if self.coordinator._pending is not None:
                return
            self.candidate_inputs.append(self.latest_requested.model_copy(deep=True))
            try:
                receipt = await self.coordinator.propose(self.latest_requested, command_id, .02)
            except InfeasibleApproach as error:
                raise SafetyViolation('target_braking_infeasible') from error
            self.requests.append(('candidate', receipt))

    async def tick(self):
        try:
            if await self.collect():
                return
            await super().tick()
            self.capture_epoch()
            if (self.pursuit is not None and not self.halted and self.input_authorized()
                    and self.pursuit._check() == 'budget_expired'):
                await self.end_pursuit('budget_expired')
        except BackendCommandError as error:
            await self._enter_fault(str(error))
