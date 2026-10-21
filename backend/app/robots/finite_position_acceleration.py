"""Finite-goal actual acceleration monitor; not a physical safety certificate."""
import math
from app.control.finite_position_types import INPUT_NS


class PositionAccelerationMonitor:
    consecutive_samples=3

    def __init__(self,limit):
        if type(limit) not in (int,float) or not math.isfinite(limit) or not 0<limit<=.2:
            raise ValueError('position_acceleration_limit_invalid')
        self.limit=float(limit);self.instant_limit=2*self.limit if self.limit==.2 else self.limit
        self.reset()

    def policy(self):
        return dict(actual_acceleration_policy='bounded_transient_at_point_two_strict_below',
            actual_acceleration_sustained_limit_radps2=self.limit,
            actual_acceleration_instant_limit_radps2=self.instant_limit,
            actual_acceleration_consecutive_samples=self.consecutive_samples,
            actual_acceleration_max_sample_gap_ms=INPUT_NS/1e6)

    def reset(self):
        self._stamp=None;self._values=None;self._counts=[0]*6

    def observe(self,values,*,captured_ns,now_ns):
        if (len(values)!=6 or any(type(v) not in (int,float) or not math.isfinite(v) for v in values)
                or type(captured_ns) is not int or captured_ns<0 or type(now_ns) is not int):
            raise ValueError('position_acceleration_feedback_invalid')
        values=tuple(values)
        if any(abs(v)>self.instant_limit for v in values):
            raise ValueError('position_reported_rate_limit')
        moderate=any(abs(v)>self.limit for v in values)
        if moderate and not 0<=now_ns-captured_ns<INPUT_NS:
            raise ValueError('position_feedback_expired')
        if self._stamp is not None:
            if captured_ns<self._stamp or captured_ns==self._stamp and values!=self._values:
                raise ValueError('position_acceleration_observer_changed')
            if captured_ns==self._stamp:return
            # Missing/interrupted observation cannot earn the transient exception.
            if moderate and captured_ns-self._stamp>=INPUT_NS:
                raise ValueError('position_reported_rate_limit')
        self._counts=[n+1 if abs(v)>self.limit else 0 for n,v in zip(self._counts,values)]
        self._stamp,self._values=captured_ns,values
        if max(self._counts)>=self.consecutive_samples:
            raise ValueError('position_reported_rate_limit')
