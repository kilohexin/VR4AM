"""Offline experiment only; not connected to production or a robot SDK."""

import asyncio
import math


class DiagnosticSampler:
    def __init__(self, read, *, samples=3, timeout=.01, interval=0, cleanup=.01):
        if (type(samples) is not int or samples <= 0
                or any(not math.isfinite(v) or v <= 0 for v in (timeout, cleanup))
                or not math.isfinite(interval) or interval < 0):
            raise ValueError("invalid sampling limits")
        self.read = read
        self.samples = samples
        self.timeout = timeout
        self.interval = interval
        self.cleanup = cleanup
        self.events = []
        self.pending = None
        self.used = False

    async def run(self):
        if self.used:
            raise RuntimeError("single use")
        self.used = True
        clock = asyncio.get_running_loop().time
        try:
            for index in range(self.samples):
                started = clock()
                self.pending = asyncio.create_task(self.read())
                done, _ = await asyncio.wait({self.pending}, timeout=self.timeout)
                event = {"started": started, "finished": clock()}
                if not done:
                    self.events.append(dict(event, kind="timeout"))
                    break
                try:
                    value = self.pending.result()
                except Exception as exc:
                    self.events.append(dict(event, kind="error", error=f"{type(exc).__name__}: {exc}"))
                    break
                self.events.append(dict(event, kind="sample", value=value))
                if index + 1 < self.samples:
                    await asyncio.sleep(self.interval)
        except asyncio.CancelledError:
            self.events.append({"kind": "cancelled", "finished": clock()})
            raise
        finally:
            if self.pending is not None and not self.pending.done():
                self.pending.cancel()
                await asyncio.wait({self.pending}, timeout=self.cleanup)
            if self.pending is not None:
                # Consume eventual exceptions without interpreting late data
                # as a sample or claiming remote cancellation succeeded.
                self.pending.add_done_callback(
                    lambda task: None if task.cancelled() else task.exception()
                )
            self.events.append({"kind": "closed", "finished": clock(),
                                "unresolved": self.pending is not None and not self.pending.done()})
