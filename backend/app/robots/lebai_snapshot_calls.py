"""Connection-owned snapshot RPC tasks; closure never verifies native release."""
from app.robots.lebai_motion_calls import OwnedMotionCalls


class OwnedSnapshotCalls(OwnedMotionCalls):
    def __init__(self):
        super().__init__()
        self.closed = False
        self.active = 0

    async def call(self, *args, **kwargs):
        if self.closed:
            raise RuntimeError('snapshot_rpc_owner_closed')
        self.active += 1
        try:
            # A terminal read exception is a failed read, not a detached late
            # payload. Keep ordinary fault fencing; only abandonment quarantines.
            return await super().call(*args, quarantine_errors=False, **kwargs)
        finally:
            self.active -= 1

    def close(self):
        self.closed = True
        super().close()
