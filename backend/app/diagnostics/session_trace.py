"""Bounded, best-effort session lifecycle diagnostics; no tasks or awaits."""

import asyncio
from time import monotonic_ns
from uuid import uuid4

from app.recording.base import try_record_event_nowait


class SessionTrace:
    def __init__(self, control: object) -> None:
        self._control = control
        self._clock = getattr(getattr(control, "clock", None), "now_ns", monotonic_ns)
        self._fields: dict[str, object] = {"session_trace_id": uuid4().hex}
        self._task_exits: list[dict[str, object]] = []
        self.dropped_events = 0

    def mark(self, phase: str, **fields: object) -> None:
        try:
            now = self._clock()
            self._fields.update(fields)
            self._fields.setdefault(f"{phase}_ns", now)
            event = {
                "kind": "teleop_session_lifecycle", **self._fields,
                "phase": phase,
                "control_generation": getattr(self._control, "control_generation", None),
                "task_exits": [dict(item) for item in self._task_exits],
                "prior_dropped_events": self.dropped_events,
            }
            if try_record_event_nowait(getattr(self._control, "recorder", None), event, now):
                return
        except Exception:
            pass
        self.dropped_events += 1

    def task_done(self, task: asyncio.Task) -> None:
        """Timestamp callback observation, not the exact coroutine exit instant."""
        try:
            cancelled = task.cancelled()
            error = None if cancelled else task.exception()
            if len(self._task_exits) < 8:
                self._task_exits.append({
                    "name": task.get_name()[:64], "observed_ns": self._clock(),
                    "outcome": "cancelled" if cancelled else "error" if error else "returned",
                    "error_type": None if error is None else type(error).__name__[:128],
                })
            self.mark("task_done_observed")
        except Exception:
            self.dropped_events += 1
