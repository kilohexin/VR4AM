"""TEST ONLY: declared reference coverage, not controller execution timing."""
from app.control.robot_control import RobotControl
from app.schemas.messages import TeleopMode
import numpy as np


class WindowGuard:
    def __init__(self):
        self.fault=None
        self.window=None
        self.observation=None

    def commit(self,window,origin_ns):
        if self.fault is not None:raise ValueError('guard_faulted')
        if not isinstance(origin_ns,int) or origin_ns<0 or not np.isfinite(window.duration_s) or window.duration_s<=0:
            raise ValueError('invalid_window')
        if self.window is not None:
            if origin_ns>=self.deadline_ns:self.reject('pvat_window_expired')
            elif origin_ns<=self.origin_ns:self.reject('pvat_reference_obsolete_window')
            elif not np.allclose(window.state(0.),self.window.state((origin_ns-self.origin_ns)/1e9),rtol=0,atol=1e-10):
                self.reject('pvat_reference_discontinuous')
            if self.fault is not None:raise ValueError('guard_faulted')
        self.window=window;self.origin_ns=origin_ns
        self.deadline_ns=origin_ns+int(round(window.duration_s*1e9))
        self.observation=None

    def observe(self,q,qd,qdd):
        try:
            vectors=np.asarray((q,qd,qdd),dtype=float)
            if vectors.shape!=(3,6) or not np.all(np.isfinite(vectors)):raise ValueError
        except (ValueError,TypeError):
            self.observation=None;self.reject('pvat_reference_invalid')
            raise ValueError('invalid_observation') from None
        self.observation=vectors.copy()

    def reject(self,reason):
        if self.fault is None:self.fault=reason

    def revoke(self):
        self.window=None;self.observation=None

    def check(self,now_ns):
        if self.fault is not None:return self.fault
        if self.window is None:return None
        reason=None
        if now_ns<self.origin_ns:reason='pvat_reference_clock_reversed'
        elif now_ns>=self.deadline_ns:reason='pvat_window_expired'
        elif self.observation is None:reason='pvat_reference_missing'
        else:
            expected=np.asarray(self.window.state((now_ns-self.origin_ns)/1e9))
            # Synthetic tolerances only, not a site-approved tracking profile.
            for index,(tolerance,label) in enumerate(((1e-4,'position'),(1e-3,'velocity'),(.02,'acceleration'))):
                if np.max(np.abs(self.observation[index]-expected[index]))>tolerance:
                    reason=f'pvat_reference_{label}_mismatch';break
        self.fault=reason
        return reason


class WindowSupervisedControl(RobotControl):
    def __init__(self,*,window_guard,**kwargs):
        super().__init__(**kwargs)
        self.window_guard=window_guard

    async def tick(self):
        if self.mode is TeleopMode.ACTIVE:
            fault=self.window_guard.check(self.clock.now_ns())
            if fault is not None:
                await self._enter_fault(fault)
                return
        await super().tick()
        if self.mode is not TeleopMode.ACTIVE:self.window_guard.revoke()
