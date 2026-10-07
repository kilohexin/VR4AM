"""Host clock correspondence for diagnostics, never a motion safety clock."""
from collections.abc import Callable
from time import monotonic_ns, time_ns


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
