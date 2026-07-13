import time


class MonotonicClock:
    def now_ns(self) -> int:
        return time.monotonic_ns()
