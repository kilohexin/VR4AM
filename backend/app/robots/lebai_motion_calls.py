"""Own motion RPC tasks without waiting for native cancellation acknowledgment.

Abandonment is permanent for this owner, including after a late result. This
is local task isolation, not remote cancellation, connection release or stop.
"""
import asyncio
import copy


class OwnedMotionCalls:
    def __init__(self, *, verify_completion_authority=False):
        self.tasks = set()
        self.unknown_methods = set()
        self.last = {}
        self._verify_completion_authority = verify_completion_authority

    async def call(self, method, operation, timeout_s, *, revoked, entered=None, quarantine_errors=True,
                   native_result_unknown=None):
        if self.unknown_methods:
            raise RuntimeError('motion_rpc_result_unknown')
        loop = asyncio.get_running_loop()
        deadline = loop.time() + timeout_s
        record = dict(method=method, entered=False, outcome='not_started',
            remote_result_unknown=False, timeout_s=timeout_s,
            started_monotonic_s=loop.time(), abandon_reason=None)
        self.last[method] = record

        async def invoke():
            if loop.time() >= deadline:
                record['abandon_reason'] = 'deadline'
                raise TimeoutError
            if revoked():
                record['abandon_reason'] = 'authority_revoked'
                raise TimeoutError
            if entered is None:
                record['entered'] = True
            return await operation()

        task = asyncio.create_task(invoke(), name='owned-motion-' + method)
        self.tasks.add(task)

        def completed(done):
            self.tasks.discard(done)
            if not done.cancelled():
                done.exception()
        task.add_done_callback(completed)
        try:
            while not task.done():
                remaining = deadline - loop.time()
                if remaining <= 0:
                    record['abandon_reason'] = 'deadline'
                    raise TimeoutError
                if revoked():
                    record['abandon_reason'] = 'authority_revoked'
                    raise TimeoutError
                # Check revocation while awaiting a native RPC that cannot be
                # interrupted remotely. This is not a realtime timing bound.
                await asyncio.wait({task}, timeout=min(.005, remaining))
            result = task.result()
            if loop.time() >= deadline:
                record['abandon_reason'] = 'deadline'
                raise TimeoutError
            # Completion may have raced revocation after the last pending
            # check. Publish no successful result for a revoked owner.
            if self._verify_completion_authority and revoked():
                record['abandon_reason'] = 'authority_revoked'
                raise TimeoutError
            record['outcome'] = 'returned'
            return result
        except BaseException as error:
            if record['abandon_reason'] is None:
                record['abandon_reason'] = ('owner_cancelled' if isinstance(error, asyncio.CancelledError)
                    else 'rpc_error')
            record['outcome'] = ('cancelled' if isinstance(error, asyncio.CancelledError)
                else 'timeout' if isinstance(error, TimeoutError) else 'error')
            record['entered'] = record['entered'] if entered is None else entered()
            record['remote_result_unknown'] = record['entered'] and (
                quarantine_errors or isinstance(error, (TimeoutError, asyncio.CancelledError)) or not task.done())
            if record['entered'] and native_result_unknown is not None:
                try:
                    evidence = native_result_unknown()
                except Exception:
                    evidence = True
                if evidence is not None:
                    record['remote_result_unknown'] = evidence if type(evidence) is bool else True
            if record['remote_result_unknown']:
                self.unknown_methods.add(method)
            if not task.done() and not task.cancelling():
                task.cancel()
            raise
        finally:
            record['completed_monotonic_s'] = loop.time()
            if entered is not None:
                record['entered'] = entered()

    def snapshot(self):
        return dict(unknown_methods=sorted(self.unknown_methods),
            pending_tasks=len(self.tasks), calls=copy.deepcopy(self.last),
            remote_cancel_verified=False, native_connection_release_verified=False)

    def close(self):
        for task in tuple(self.tasks):
            if not task.done() and not task.cancelling():
                task.cancel()
