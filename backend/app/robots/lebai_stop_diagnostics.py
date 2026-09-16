"""Opt-in read-only stop diagnostics; never an input to stop confirmation."""
from __future__ import annotations

import asyncio
import copy
import math
from collections.abc import Callable
from typing import Any

from app.robots.lebai_sdk_bridge import LebaiClientProtocol


class StopDiagnosticSampler:
    def __init__(
        self, client: LebaiClientProtocol, episode_id: int,
        clock: Callable[[], int], emit: Callable[[dict[str, Any]], None], *,
        read_timeout: float = .1, duration: float = 2.,
        interval: float = .1, cleanup_timeout: float = .05,
    ) -> None:
        if any(not math.isfinite(v) or v <= 0 for v in
               (read_timeout, duration, interval, cleanup_timeout)):
            raise ValueError("invalid diagnostic sampling limits")
        self.client = client
        self.episode_id = episode_id
        self.clock = clock
        self.emit = emit
        self.read_timeout = read_timeout
        self.duration = duration
        self.interval = interval
        self.cleanup_timeout = cleanup_timeout
        self.task: asyncio.Task[None] | None = None
        self.pending: asyncio.Task[None] | None = None
        self.closing = False
        self.closed = False
        self.finishing = False
        self.dropped_events = 0

    def start(self) -> None:
        if self.task is None and not self.closed:
            self.task = asyncio.create_task(self._run(), name="lebai-stop-diagnostic-reader")

    def _emit(self, event: dict[str, Any]) -> None:
        try:
            self.emit(dict(copy.deepcopy(event), episode_id=self.episode_id))
        except Exception:
            # Diagnostics must never decide whether a stop succeeds.
            self.dropped_events += 1

    async def _read(self, event: dict[str, Any]) -> None:
        for method, field in (("get_robot_state", "raw_robot_state"),
                              ("get_kin_data", "raw_kinematics"),
                              ("get_estop_reason", "raw_estop")):
            if self.closing:
                raise asyncio.CancelledError
            timing = {"method": method, "started_ns": self.clock()}
            event["reads"].append(timing)
            try:
                event[field] = await getattr(self.client, method)()
            finally:
                timing["completed_ns"] = self.clock()

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        deadline = loop.time() + self.duration
        event = None
        try:
            while not self.closing and loop.time() < deadline:
                event = {"kind": "stop_diagnostic_read", "started_ns": self.clock(), "reads": []}
                self.pending = asyncio.create_task(self._read(event))
                done, _ = await asyncio.wait({self.pending}, timeout=min(self.read_timeout, deadline - loop.time()))
                event["completed_ns"] = self.clock()
                if not done:
                    event["outcome"] = "timeout"
                    self._emit(event)
                    break
                try:
                    self.pending.result()
                except Exception as exc:
                    event.update(outcome="error", error_type=type(exc).__name__)
                    self._emit(event)
                    break
                event["outcome"] = "sample"
                self._emit(event)
                event = None
                await asyncio.sleep(max(0., min(self.interval, deadline - loop.time())))
        except asyncio.CancelledError:
            self._emit({"kind": "stop_diagnostic_read", "outcome": "cancelled",
                        "completed_ns": self.clock(), "partial": event})
        finally:
            self.finishing = True
            self.closing = True
            if self.pending is not None:
                if not self.pending.done():
                    self.pending.cancel()
                    await asyncio.wait({self.pending}, timeout=self.cleanup_timeout)
                self.pending.add_done_callback(
                    lambda task: None if task.cancelled() else task.exception()
                )
            self.closed = True
            self._emit({"kind": "stop_diagnostic_reader_closed", "completed_ns": self.clock(),
                        "unresolved_read": self.pending is not None and not self.pending.done(),
                        "remote_cancel_confirmed": False, "dropped_events": self.dropped_events})

    async def close(self) -> None:
        if self.task is None:
            self.closed = True
            return
        if not self.closing:
            self.closing = True
            if not self.task.done() and not self.finishing:
                self.task.cancel()
        await asyncio.wait({self.task}, timeout=self.cleanup_timeout + .05)
        if self.task.done() and not self.closed:
            # Cancellation before the coroutine's first turn has no finally.
            self.closed = True
            self._emit({"kind": "stop_diagnostic_reader_closed", "completed_ns": self.clock(),
                        "unresolved_read": self.unresolved, "remote_cancel_confirmed": False})

    @property
    def unresolved(self) -> bool:
        return ((self.task is not None and not self.task.done())
                or (self.pending is not None and not self.pending.done()))
