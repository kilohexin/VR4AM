"""Offline RPC-return races: an old Trigger frame must not activate arm motion."""

import pytest

from app.schemas.messages import TeleopMode
from tests.control.test_robot_control import connect_release_arm, frame, make_control


@pytest.mark.parametrize('active', [False, True])
@pytest.mark.parametrize('change', ['replaced', 'expired', 'disarmed', 'rearmed'])
async def test_gripper_return_does_not_consume_revoked_input(active, change):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, active, trigger=0), clock.now_ns())
    await control.tick()
    previous_seq = control.last_seq
    expected_seq = previous_seq
    previous_targets = list(backend.targets)

    async def delayed_claw(value):
        nonlocal expected_seq
        backend.gripper_commands.append(value)
        if change == 'replaced':
            latest.publish(frame(4, False, trigger=0.5), clock.now_ns())
        elif change == 'expired':
            clock.advance_ms(100)
        else:
            await control.disarm()
            if change == 'rearmed':
                latest.publish(frame(4, False, trigger=0.5), clock.now_ns())
                await control.tick()  # Observe release before a new Arm.
                await control.arm()
                expected_seq = control.last_seq

    backend.set_gripper = delayed_claw
    latest.publish(frame(3, True, trigger=0.5), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == [0.5]  # Already issued RPC isn't undone.
    assert backend.targets == previous_targets
    assert control.last_seq == expected_seq
    if change == 'disarmed':
        assert control.mode is TeleopMode.DISARMED
    elif change == 'rearmed' or not active:
        assert control.mode is TeleopMode.ARMED
        assert not control.mapper.has_anchor
    if change == 'replaced':
        await control.tick()
        assert control.last_seq == 4
        assert control.mode is (TeleopMode.HOLD if active else TeleopMode.ARMED)
    elif change == 'expired':
        await control.tick()
        assert control.mode is TeleopMode.STALE


@pytest.mark.parametrize('delay_ms', [0, 99])
async def test_current_fresh_gripper_return_still_allows_grip_capture(delay_ms):
    control, latest, backend, clock = make_control()
    await connect_release_arm(control, latest, clock)
    latest.publish(frame(2, False, trigger=0), clock.now_ns())
    await control.tick()

    async def claw(value):
        backend.gripper_commands.append(value)
        clock.advance_ms(delay_ms)

    backend.set_gripper = claw
    latest.publish(frame(3, True, trigger=0.5), clock.now_ns())
    await control.tick()
    assert backend.gripper_commands == [0.5]
    assert control.mode is TeleopMode.ACTIVE
    assert control.mapper.has_anchor
    assert control.last_seq == 3
