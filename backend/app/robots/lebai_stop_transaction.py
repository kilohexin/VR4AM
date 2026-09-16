"""Bounded ownership of stop RPCs; RPC completion never confirms stationarity."""
from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from typing import Any


class StopRpc:
    def __init__(self, episode_id: int, request_id: int, method: str,
                 send: Callable[[], Awaitable[object]], clock: Callable[[], int],
                 emit: Callable[[dict[str, object]], None]) -> None:
        self.episode_id = episode_id
        self.request_id = request_id
        self.method = method
        self.clock = clock
        self.emit = emit
        self.started_ns = clock()
        self.deadline_ns = self.started_ns + 200_000_000
        self.deadline = asyncio.get_running_loop().time() + .2
        self.wait_failed = False
        self.outcome = 'pending'
        self.result: object = None
        self.error: Exception | None = None
        self.task = asyncio.create_task(self._run(send))

    async def _run(self, send: Callable[[], Awaitable[object]]) -> None:
        try:
            self.result = await send()
        except asyncio.CancelledError:
            self.wait_failed = True
            self.outcome = 'unknown_on_local_cancel'
            # Local cancellation is deliberately NOT remote cancellation proof.
            raise
        except Exception as exc:
            self.error = exc
            self.outcome = 'error'
        else:
            late = (self.wait_failed or self.clock() >= self.deadline_ns
                    or asyncio.get_running_loop().time() >= self.deadline)
            self.outcome = 'returned_late' if late else 'returned'
            self.wait_failed |= late
        finally:
            self.emit({
                'kind': 'stop_rpc_lifecycle', 'episode_id': self.episode_id,
                'request_id': self.request_id, 'method': self.method,
                'started_ns': self.started_ns, 'completed_ns': self.clock(),
                'deadline_ns': self.deadline_ns, 'outcome': self.outcome,
                'error_type': type(self.error).__name__ if self.error else None,
                'wait_failed': self.wait_failed,
                'remote_result_unknown': self.outcome == 'unknown_on_local_cancel',
            })

    async def wait(self) -> object:
        if self.wait_failed:
            raise TimeoutError
        if not self.task.done():
            remaining = min(self.deadline - asyncio.get_running_loop().time(),
                            (self.deadline_ns - self.clock()) / 1e9)
            if remaining <= 0:
                self.wait_failed = True
                raise TimeoutError
            try:
                await asyncio.wait_for(asyncio.shield(self.task), remaining)
            except (TimeoutError, asyncio.CancelledError):
                self.wait_failed = True
                raise
        if self.wait_failed:
            raise TimeoutError
        if self.error is not None:
            raise self.error
        return self.result


class StopTransaction:
    def __init__(self, episode_id: int, clock: Callable[[], int],
                 emit: Callable[[dict[str, object]], None],
                 on_request_started: Callable[[str], None] | None = None) -> None:
        self.episode_id = episode_id
        self.clock = clock
        self.emit = emit
        self.on_request_started = on_request_started
        self.requests: dict[str, StopRpc] = {}
        self.verification_started_ns: int | None = None
        self.stable_since_ns: int | None = None
        self.stable_anchor: Any = None
        self.confirmed = False
        self.closed = False

    @property
    def unresolved(self) -> bool:
        return any(not r.task.done() or r.wait_failed or r.error is not None
                   for r in self.requests.values())

    async def call(self, method: str, send: Callable[[], Awaitable[object]],
                   call: dict[str, Any]) -> object:
        if self.closed:
            raise RuntimeError('stop_transaction_closed')
        reused = method in self.requests
        if not reused:
            self.requests[method] = StopRpc(self.episode_id, len(self.requests) + 1,
                                            method, send, self.clock, self.emit)
            if self.on_request_started is not None:
                # StopRpc's task is queued first. This observer cannot change
                # the request deadline or turn a stop into a diagnostic error.
                try:
                    self.on_request_started(method)
                except Exception:
                    pass
        request = self.requests[method]
        call.update(episode_id=self.episode_id, request_id=request.request_id,
                    reused=reused, request_started_ns=request.started_ns,
                    deadline_ns=request.deadline_ns)
        return await request.wait()

    async def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        pending = [r for r in self.requests.values() if not r.task.done()]
        self.emit({'kind': 'stop_transaction_closed', 'episode_id': self.episode_id,
                   'unresolved_requests': [r.request_id for r in pending],
                   'requests': [{'request_id': r.request_id, 'method': r.method,
                                 'outcome': r.outcome, 'wait_failed': r.wait_failed}
                                for r in self.requests.values()],
                   'remote_cancel_confirmed': False})
        for request in pending:
            request.task.cancel()
        if pending:
            # A badly behaved SDK must not block application teardown forever.
            await asyncio.wait([r.task for r in pending], timeout=.05)
