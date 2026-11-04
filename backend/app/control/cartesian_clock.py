"""Conservative paired-clock bounds. No timestamp refresh or actuator access."""
from dataclasses import dataclass


@dataclass(frozen=True)
class ControllerClock:
    sent_ns: int
    received_ns: int
    sampled_ms: int
    resolution_ms: int=1

    def __post_init__(self):
        if (any(type(v) is not int for v in (self.sent_ns,self.received_ns,self.sampled_ms))
                or not 0<=self.sent_ns<=self.received_ns or self.sampled_ms<0
                or self.received_ns-self.sent_ns>50_000_000
                or type(self.resolution_ms) is not int or not 1<=self.resolution_ms<=10):
            raise ValueError('cartesian_clock_pair')

    def capture_lower_ms(self,host_capture_ns):
        """Earliest possible controller time: age may grow, never shrink."""
        if type(host_capture_ns) is not int or host_capture_ns<self.received_ns:
            raise ValueError('cartesian_clock_capture_before_pair')
        return self.sampled_ms+(host_capture_ns-self.received_ns)//1_000_000-self.resolution_ms

    def sample_lower_ns(self,controller_sample_ms):
        if type(controller_sample_ms) is not int or controller_sample_ms<self.sampled_ms:
            raise ValueError('cartesian_clock_rollback')
        return self.sent_ns+(controller_sample_ms-self.sampled_ms-self.resolution_ms)*1_000_000

    def check_pair(self,sent_ns,received_ns,sampled_ms):
        other=ControllerClock(sent_ns,received_ns,sampled_ms,self.resolution_ms)
        if sampled_ms<self.sampled_ms or sent_ns<self.received_ns:
            raise ValueError('cartesian_clock_rollback')
        # One millisecond of native integer timestamp rounding at each end.
        rounding=self.resolution_ms*1_000_000
        low=self.sent_ns-self.sampled_ms*1_000_000-rounding
        high=self.received_ns-self.sampled_ms*1_000_000+rounding
        if (other.received_ns-other.sampled_ms*1_000_000+rounding<low
                or other.sent_ns-other.sampled_ms*1_000_000-rounding>high):
            raise ValueError('cartesian_clock_drift')
        return other
