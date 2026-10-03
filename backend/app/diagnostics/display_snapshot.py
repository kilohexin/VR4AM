"""Passive display snapshot prototype, not wired to a runtime sender.

This view owns no SDK, tasks, locks, fallback reads or control permissions.
An explicit publisher supplies completed reads and connection generations.
Local read age cannot establish the device source's sampling age.
"""

from collections.abc import Callable
from dataclasses import dataclass
import math
from typing import Literal

from app.schemas.messages import BackendState, RobotStateMessage


@dataclass(frozen=True)
class DisplaySnapshot:
    status: Literal["available", "stale", "missing", "disconnected", "invalid_clock"]
    generation: int
    state: RobotStateMessage | None
    fault: str | None
    local_read_age_ms: float | None = None
    local_completion_age_ms: float | None = None
    read_started_ns: int | None = None
    read_completed_ns: int | None = None
    read_purpose: str | None = None
    source_freshness: Literal["unknown"] = "unknown"


@dataclass(frozen=True)
class _Published:
    state: RobotStateMessage
    started_ns: int
    completed_ns: int
    purpose: str


class DisplaySnapshotView:
    """Single-event-loop display view; never use it for control decisions.

    The caller chooses a display-age threshold, forwards connection lifecycle
    and publishes matching state/read boundaries. The fault argument to read()
    must come from the current authoritative owner, not a cached UI message.
    """

    def __init__(self, *, clock: Callable[[], int], max_local_age_ms: float) -> None:
        if not math.isfinite(max_local_age_ms) or max_local_age_ms <= 0:
            raise ValueError("invalid_display_age_limit")
        self._clock = clock
        self._max_age_ms = max_local_age_ms
        self._generation = 0
        self._connected = False
        self._sample: _Published | None = None

    def begin_connection(self) -> int:
        self._generation += 1
        self._connected = True
        self._sample = None
        return self._generation

    def disconnect(self) -> None:
        self._generation += 1
        self._connected = False
        self._sample = None

    def publish(
        self, generation: int, state: RobotStateMessage, *,
        read_started_ns: int, read_completed_ns: int, read_purpose: str,
    ) -> bool:
        if not self._connected or generation != self._generation:
            return False
        if (
            type(read_started_ns) is not int
            or type(read_completed_ns) is not int
            or not 0 <= read_started_ns <= read_completed_ns <= self._clock()
        ):
            raise ValueError("invalid_read_boundaries")
        previous = self._sample
        if previous is not None and (read_completed_ns, read_started_ns) <= (
            previous.completed_ns, previous.started_ns,
        ):
            return False
        self._sample = _Published(
            state.model_copy(deep=True), read_started_ns, read_completed_ns, read_purpose,
        )
        return True

    def read(self, *, fault: str | None) -> DisplaySnapshot:
        if not self._connected:
            return DisplaySnapshot("disconnected", self._generation, None, fault)
        sample = self._sample
        if sample is None:
            return DisplaySnapshot("missing", self._generation, None, fault)
        now_ns = self._clock()
        effective_fault = fault or sample.state.fault
        if now_ns < sample.completed_ns:
            return DisplaySnapshot("invalid_clock", self._generation, None, effective_fault)
        age_ms = (now_ns - sample.started_ns) / 1_000_000
        state = sample.state.model_copy(deep=True)
        if effective_fault is not None:
            state.fault = effective_fault
            state.robot_state = BackendState.FAULT
        return DisplaySnapshot(
            status="stale" if age_ms >= self._max_age_ms else "available",
            generation=self._generation, state=state, fault=effective_fault,
            local_read_age_ms=age_ms,
            local_completion_age_ms=(now_ns - sample.completed_ns) / 1_000_000,
            read_started_ns=sample.started_ns, read_completed_ns=sample.completed_ns,
            read_purpose=sample.purpose,
        )
