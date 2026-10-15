"""Bounded per-WebSocket operations while input/release remains receivable."""
import asyncio
from collections import deque
from app.robots.base import BackendCommandError


class OperatorJobs:
    def __init__(self, control, send, *, allowed=lambda: True):
        self.control, self.send = control, send
        self.allowed = allowed
        self.open = True
        self._tasks = set()
        self._arm = None
        self._claw = None
        self._seen = deque(maxlen=256)
        self._error = None

    @property
    def gripper_current(self):
        return self.open and self._claw is not None and self._claw['current']

    def check_error(self):
        if self._error is not None:
            raise self._error

    def _start(self, coroutine):
        task = asyncio.create_task(coroutine)
        self._tasks.add(task)
        def done(result):
            self._tasks.discard(result)
            if not result.cancelled():
                error = result.exception()
                if error is not None:
                    self._error = error
        task.add_done_callback(done)
        return task

    def cancel_gripper(self, request_id=None):
        if self._claw is not None and (request_id is None or self._claw['id'] == request_id):
            self._claw['current'] = False

    async def gripper(self, message):
        reason = ('duplicate_request' if message.request_id in self._seen else
                  'automation_active' if not self.allowed() else
                  'request_busy' if self._claw is not None or self._arm is not None else
                  'connection_closed' if not self.open else None)
        if reason:
            await self.send(dict(v=1,type='gripper_result',request_id=message.request_id,
                                 accepted=False,message=reason))
            return
        self._seen.append(message.request_id)
        holder = dict(id=message.request_id,current=True)
        self._claw = holder
        async def run():
            accepted, reason = False, 'request_revoked'
            try:
                current = lambda: self.open and self.allowed() and self._claw is holder and holder['current']
                if current():
                    try:
                        await self.control.command_gripper(message.value,is_current=current)
                        accepted, reason = True, 'sdk_returned_not_physical_completion'
                    except BackendCommandError as error:
                        await self.control._enter_fault(str(error))
                        reason = 'backend_fault'
                    except RuntimeError:
                        reason = 'not_authorized_or_not_idle'
                if self.open:
                    await self.send(dict(v=1,type='gripper_result',request_id=message.request_id,
                                         accepted=accepted,message=reason))
            finally:
                if self._claw is holder:
                    self._claw = None
        self._start(run())

    async def arm(self, message):
        if self._arm is not None or self._claw is not None or not self.open or not self.allowed():
            await self.send(dict(v=1,type='arm_rejected',request_id=message.request_id,message='操作正在进行，请等待结束。'))
            return
        async def run():
            try:
                try:
                    await self.control.arm()
                except RuntimeError:
                    if self.open:
                        await self.send(dict(v=1,type='arm_rejected',request_id=message.request_id,
                                             message='工作准备位未完成，请查看状态和故障。'))
                else:
                    if self.open:
                        await self.send(dict(v=1,type='arm_ack',request_id=message.request_id))
            finally:
                self._arm = None
        self._arm = self._start(run())

    async def join(self):
        while self._tasks:
            batch=tuple(self._tasks)
            results=await asyncio.gather(*batch,return_exceptions=True)
            # gather can be synchronously complete while done callbacks are
            # still queued (notably under an AnyIO cancellation shield).
            # Remove joined tasks directly; do not spin waiting for callbacks.
            self._tasks.difference_update(task for task in batch if task.done())
            for result in results:
                if isinstance(result,BaseException) and not isinstance(result,asyncio.CancelledError):
                    self._error=result
        self.check_error()

    async def close(self):
        self.open = False
        self.cancel_gripper()
        if self._arm is not None:
            self._arm.cancel()
        # Do not abandon a native claw call that may already have entered.
        # Its original adapter timeout/unknown-result ownership still applies.
        await self.join()
