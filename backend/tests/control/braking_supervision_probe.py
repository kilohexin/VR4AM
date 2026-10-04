"""Offline bridge only: translate candidate rejection into existing fault handling.

This intentionally does not extend the runtime recovery whitelist. Resetting the
candidate's arithmetic state is not evidence that a backend or device stopped.
"""

from app.control.safety import SafetyViolation
from tests.control.braking_approach_probe import BrakingApproach, InfeasibleApproach
from tests.robots.test_vr_profile_experiments import limiter


class SupervisedApproach:
    def __init__(self):
        self.limits = limiter()
        self.candidate = BrakingApproach(self.limits)
        self.rejections = []

    def __getattr__(self, name):
        return getattr(self.limits, name)

    def reset_motion(self):
        self.limits.reset_motion()
        self.candidate.reset_motion()

    def clear(self):
        self.limits.clear()
        self.candidate.reset_motion()

    def set_pose_anchor(self, pose):
        self.limits.set_pose_anchor(pose)
        self.candidate.reset_motion()

    def limit_motion(self, previous, requested, dt):
        try:
            return self.candidate.limit_motion(previous, requested, dt)
        except InfeasibleApproach as error:
            self.rejections.append(str(error))
            raise SafetyViolation('target_braking_infeasible') from error
