"""Test-only SDK boundary experiment, never a production client factory."""

import asyncio

from tests.prototypes.stop_request_owner import StopRequestOwner


class OwnedStopClient:
    def __init__(self, client):
        self.client = client
        self.owners = {}

    def __getattr__(self, name):
        return getattr(self.client, name)

    async def _wait(self, method):
        if method not in self.owners:
            self.owners[method] = StopRequestOwner(getattr(self.client, method))
        owner = self.owners[method]
        # The unmodified adapter applies its own timeout/cancellation. Retain
        # the owned coroutine when that caller stops waiting.
        result = await owner.wait('adapter', None)
        if result == 'FAILED':
            raise RuntimeError(owner.error)
        if result not in {'RETURNED', 'RETURNED_LATE'}:
            raise RuntimeError('unresolved_test_stop_request')

    async def stop_move(self):
        await self._wait('stop_move')

    async def stop_sys(self):
        await self._wait('stop_sys')

    async def cleanup(self):
        # Test teardown only. Cancellation here is NOT a robot stop command.
        tasks = [owner.task for owner in self.owners.values() if owner.task]
        for task in tasks:
            if not task.done():
                task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
