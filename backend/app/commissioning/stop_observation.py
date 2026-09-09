from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Awaitable, Callable
from datetime import UTC, datetime
from typing import Protocol

from app.recording.commissioning import CommissioningRecorder


class StopObservationSource(Protocol):
    async def read_stop_observation(self) -> dict[str, object]: ...


def validate_observation_seconds(seconds: float) -> None:
    if not math.isfinite(seconds) or not 0 <= seconds <= 120:
        raise ValueError("smoke_observation_out_of_bounds")


async def observe_after_stop(
    source: StopObservationSource,
    recorder: CommissioningRecorder,
    seconds: float,
    *,
    prior_error: str | None = None,
    clock: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], Awaitable[None]] = asyncio.sleep,
) -> None:
    """Diagnostic tail only: reads, no retries of motion or changes to safety state."""
    validate_observation_seconds(seconds)
    if seconds == 0:
        return
    started_ns = clock()
    deadline_ns = started_ns + int(seconds * 1e9)
    period_ns = 200_000_000
    previous_started_ns: int | None = None
    last_success_ns: int | None = None
    successful = errors = sequence = 0
    max_gap_ms = 0.0
    dropped_before = recorder.dropped_normal_events
    interrupted = False
    await recorder.write_critical_event("stop_observation_started", {
        "utc": datetime.now(UTC).isoformat(),
        "duration_s": seconds,
        "target_interval_ms": 200,
        "read_timeout_ms": 500,
        "assessment": "diagnostic_only",
        "prior_error": prior_error,
    }, started_ns)
    try:
        while clock() < deadline_ns:
            attempt_ns = clock()
            sequence += 1
            fields: dict[str, object] = {
                "sequence": sequence,
                "utc": datetime.now(UTC).isoformat(),
                "read_started_ns": attempt_ns,
                "previous_attempt_gap_ms": None if previous_started_ns is None
                else (attempt_ns - previous_started_ns) / 1e6,
            }
            previous_started_ns = attempt_ns
            try:
                state = await asyncio.wait_for(
                    source.read_stop_observation(),
                    # A read begun within the window gets its normal budget;
                    # do not manufacture a timeout at the duration boundary.
                    timeout=.5,
                )
            except Exception as error:
                errors += 1
                fields.update(kind="stop_observation_read_failed",
                              error=str(error), error_type=type(error).__name__)
            else:
                successful += 1
                observed_ns = clock()
                gap_ms = (observed_ns - (last_success_ns if last_success_ns is not None
                                        else started_ns)) / 1e6
                max_gap_ms = max(max_gap_ms, gap_ms)
                last_success_ns = observed_ns
                fields.update(kind="stop_observation_sample", state=state,
                              successful_sample_gap_ms=gap_ms)
            finished_ns = clock()
            fields["read_finished_ns"] = finished_ns
            fields["read_duration_ms"] = (finished_ns - attempt_ns) / 1e6
            await recorder.write_event(fields, finished_ns)
            # Skip missed ticks; never burst reads to catch up with the clock.
            remaining_s = (deadline_ns - clock()) / 1e9
            if remaining_s > 0:
                await sleep(min(remaining_s, max(.02, (attempt_ns + period_ns - clock()) / 1e9)))
    except BaseException:
        interrupted = True
        raise
    finally:
        finished_ns = clock()
        trailing_gap_ms = (finished_ns - (last_success_ns if last_success_ns is not None
                                        else started_ns)) / 1e6
        max_gap_ms = max(max_gap_ms, trailing_gap_ms)
        dropped = recorder.dropped_normal_events - dropped_before
        await recorder.write_critical_event("stop_observation_summary", {
            "utc": datetime.now(UTC).isoformat(),
            "assessment": "diagnostic_only",
            "complete": not interrupted and errors == 0 and successful > 0 and dropped == 0,
            "interrupted": interrupted,
            "elapsed_s": (finished_ns - started_ns) / 1e9,
            "successful_samples": successful,
            "read_errors": errors,
            "dropped_events": dropped,
            "max_successful_sample_gap_ms": max_gap_ms,
            "trailing_gap_ms": trailing_gap_ms,
        }, finished_ns)
    if errors or not successful or dropped:
        raise RuntimeError("stop_observation_incomplete")
