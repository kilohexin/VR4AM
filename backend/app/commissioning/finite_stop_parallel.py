"""Owned stop snapshots shared by distinct Fake and separately admitted scopes."""
from contextlib import contextmanager

from app.commissioning.finite_window_run import (
    _FinitePauseScopeClient, _FinitePauseComparisonAdapter, _FiniteSamplingPauseAdapter,
)
from app.commissioning.idle_stop_baseline import AdmissionRejected


class ParallelStopScopeClient(_FinitePauseScopeClient):
    @contextmanager
    def _parallel_stop_snapshot_scope(self, guard):
        # Completed known groups may be replaced. Failed, abandoned or pending
        # children remain in place and block all subsequent groups/commands.
        self.require_clear_read_gate()
        guard()
        if self.pending_read is not None and not self.pending_read.done():
            raise AdmissionRejected('finite_parallel_stop_parent_read_pending')
        self.snapshot_read_owners.clear()
        group = dict(active=True, guard=guard, used=set())
        self._active_snapshot_group = group
        token = self._snapshot_context.set(group)
        try:
            yield
        finally:
            group['active'] = False
            self._active_snapshot_group = None
            self._snapshot_context.reset(token)


class _ParallelStopSnapshotMixin:
    def __init__(self, settings, **kwargs):
        self._stop_read_guard = None
        super().__init__(settings, **kwargs)

    async def _read_snapshot_impl(self, read, **kwargs):
        if kwargs['read_purpose'] != 'stop_verification':
            return await super()._read_snapshot_impl(read, **kwargs)
        client, transaction = self._client, self._stop_transaction
        generation, owner = self._pump.generation, self._snapshot_calls

        def guard():
            request = transaction.requests.get('pause_move') if transaction is not None else None
            if (not isinstance(client, ParallelStopScopeClient) or client is not self._client
                    or transaction is None or transaction is not self._stop_transaction or transaction.closed
                    or request is None or not request.task.done() or request.wait_failed
                    or request.outcome != 'returned' or transaction.unresolved
                    or client.termination_entries != 1 or self._pump.generation != generation
                    or owner is not self._snapshot_calls or owner.closed):
                raise AdmissionRejected('finite_parallel_stop_read_revoked')

        if self._stop_read_guard is not None:
            raise AdmissionRejected('finite_parallel_stop_read_busy')
        guard()
        self._stop_read_guard = guard
        try:
            with client._parallel_stop_snapshot_scope(guard):
                return await super()._read_snapshot_impl(read, **kwargs)
        finally:
            self._stop_read_guard = None

    async def _read_snapshot_fields(self, client, latencies, deadline_ns, **kwargs):
        if kwargs['read_purpose'] != 'stop_verification':
            return await super()._read_snapshot_fields(client, latencies, deadline_ns, **kwargs)
        if self._stop_read_guard is None:
            raise AdmissionRejected('finite_parallel_stop_read_not_admitted')
        return await self._parallel_snapshot_fields(client, latencies, deadline_ns,
            check=self._stop_read_guard, include_raw_kinematics=kwargs['include_raw_kinematics'])

    def _before_snapshot_install(self, snapshot, read):
        if snapshot.read_purpose == 'stop_verification':
            if self._stop_read_guard is None:
                raise AdmissionRejected('finite_parallel_stop_read_not_admitted')
            self._stop_read_guard()
        super()._before_snapshot_install(snapshot, read)


class ParallelStopPauseAdapter(_ParallelStopSnapshotMixin, _FinitePauseComparisonAdapter):
    """The original Fake-only selector remains restricted to literal loopback."""
    def __init__(self, settings, **kwargs):
        if (settings.ip != '127.0.0.1' or kwargs.get('backend_label') != 'LEBAI_FAKE'
                or kwargs.get('offline_stop_start_period') is not True
                or not callable(kwargs.get('client_factory'))):
            raise ValueError('finite_parallel_stop_reads_requires_offline_pause')
        super().__init__(settings, **kwargs)


class SiteParallelStopPauseAdapter(_ParallelStopSnapshotMixin, _FiniteSamplingPauseAdapter):
    """Selected by the independently authorized runner, never the Fake flag."""
    def __init__(self, settings, **kwargs):
        from app.commissioning.finite_pause_window import validate_settings
        validate_settings(settings)
        if (kwargs.get('backend_label') != 'LEBAI'
                or kwargs.get('offline_stop_start_period', False) is not False
                or not callable(kwargs.get('client_factory'))):
            raise ValueError('finite_site_parallel_stop_scope_required')
        super().__init__(settings, **kwargs)
