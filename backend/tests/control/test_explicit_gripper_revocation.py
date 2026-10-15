"""Explicit claw hold is scoped through the original backend entry guard."""
import pytest

from app.robots.base import GripperCommandRevoked
from tests.control.test_grip_release_intent import active_control, frame


@pytest.mark.parametrize('boundary', ['preflight', 'recording', 'sdk_entry'])
async def test_explicit_claw_cancel_prevents_pending_sdk_write(boundary):
    control, latest, backend, clock = await active_control()
    latest.publish(frame(3, True), clock.now_ns())
    allowed = True
    if boundary == 'preflight':
        original = backend.preflight
        async def revoke():
            nonlocal allowed
            result = await original()
            allowed = False
            return result
        backend.preflight = revoke
    elif boundary == 'recording':
        original = control.recorder.write_critical_event
        async def revoke(*args):
            nonlocal allowed
            await original(*args)
            allowed = False
        control.recorder.write_critical_event = revoke
    else:
        async def revoke(value):
            nonlocal allowed
            allowed = False
            if not control._has_gripper_dispatch_authority():
                raise GripperCommandRevoked('revoked')
            await type(backend).set_gripper(backend, value)
        backend.set_gripper = revoke
    with pytest.raises(RuntimeError, match='gripper_command_authority_changed'):
        await control.command_gripper(1., is_current=lambda: allowed)
    assert backend.gripper_commands == []
    assert control._has_gripper_dispatch_authority(), 'explicit callback must be cleared after the request'
