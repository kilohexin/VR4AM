"""Host clock correspondence for diagnostics, never a motion safety clock."""
from collections.abc import Callable
import math
from time import get_clock_info, monotonic_ns, perf_counter_ns, time_ns


def sample_clock_anchor(
    monotonic: Callable[[], int] = monotonic_ns,
    wall: Callable[[], int] = time_ns,
) -> dict[str, object] | None:
    """Bracket one wall sample with the OS monotonic clock, without I/O.

    The bracket is sampling uncertainty, not a bound on clock drift, clock
    adjustments or packet timestamp accuracy. Injected/test clocks need not
    share this host's time.monotonic_ns epoch. Missing evidence stays missing.
    """
    try:
        before = monotonic()
        timestamp = wall()
        after = monotonic()
        if (any(type(value) is not int for value in (before, timestamp, after))
                or before < 0 or timestamp < 0 or after < before):
            return None
        return {
            'clock': 'time.monotonic_ns', 'wall_clock': 'time.time_ns',
            'monotonic_before_ns': before, 'wall_time_ns': timestamp,
            'monotonic_after_ns': after, 'sampling_span_ns': after - before,
        }
    except Exception:
        # A diagnostic failure must not turn into a control/cleanup failure.
        return None


def sample_performance_ns(performance: Callable[[], int] | None = None) -> int | None:
    """Independent short-duration timestamp; never a control deadline input."""
    try:
        value = (performance or perf_counter_ns)()
        return value if type(value) is int and value >= 0 else None
    except Exception:
        return None


def sample_performance_anchor(
    performance: Callable[[], int] | None = None,
    wall: Callable[[], int] | None = None,
) -> dict[str, object] | None:
    """Bracket wall time in the performance epoch, without asserting alignment.

    Resolution and the bracket do not bound wall-clock quantization, drift,
    clock adjustments, driver timestamp errors or packet capture accuracy.
    """
    try:
        before = sample_performance_ns(performance)
        timestamp = (wall or time_ns)()
        after = sample_performance_ns(performance)
        if (before is None or after is None or after < before
                or type(timestamp) is not int or timestamp < 0):
            return None
        info = {}
        for name in ('perf_counter', 'time'):
            item = vars(get_clock_info(name))
            if not math.isfinite(item['resolution']) or item['resolution'] <= 0:
                return None
            info[name] = item
        return {
            'clock': 'time.perf_counter_ns', 'wall_clock': 'time.time_ns',
            'performance_before_ns': before, 'wall_time_ns': timestamp,
            'performance_after_ns': after, 'sampling_span_ns': after - before,
            'clock_info': info, 'alignment_verified': False,
            'assessment': 'diagnostic_only',
        }
    except Exception:
        return None
