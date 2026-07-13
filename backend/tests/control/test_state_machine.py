import pytest

from app.control.state_machine import TeleopStateMachine
from app.schemas.messages import TeleopMode


def arm_machine(machine: TeleopStateMachine) -> None:
    machine.connect()
    machine.observe_grip(False)
    machine.arm()


def test_normal_clutch_lifecycle() -> None:
    machine = TeleopStateMachine()
    machine.connect()
    assert machine.mode == TeleopMode.READY
    machine.observe_grip(False)
    machine.arm()
    machine.observe_grip(True)
    assert machine.mode == TeleopMode.ACTIVE
    machine.observe_grip(False)
    assert machine.mode == TeleopMode.HOLD


def test_recovery_requires_grip_release_and_explicit_rearm() -> None:
    machine = TeleopStateMachine()
    arm_machine(machine)
    machine.observe_grip(True)
    machine.stale()
    assert machine.mode == TeleopMode.STALE

    machine.stop_complete()
    assert machine.mode == TeleopMode.DISARMED
    machine.observe_grip(True)
    assert machine.mode == TeleopMode.DISARMED
    assert machine.can_arm is False
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        machine.arm()

    machine.observe_grip(False)
    assert machine.mode == TeleopMode.READY
    assert machine.can_arm is True
    assert machine.mode != TeleopMode.ARMED

    machine.arm()
    assert machine.mode == TeleopMode.ARMED


def test_hold_regrip_returns_active_without_global_rearm() -> None:
    machine = TeleopStateMachine()
    arm_machine(machine)
    machine.observe_grip(True)
    machine.observe_grip(False)
    assert machine.mode == TeleopMode.HOLD
    machine.observe_grip(True)
    assert machine.mode == TeleopMode.ACTIVE


def test_disconnect_never_preserves_armed_state() -> None:
    machine = TeleopStateMachine()
    arm_machine(machine)
    machine.disconnect()
    machine.connect()
    assert machine.mode == TeleopMode.READY
    assert machine.can_arm is False
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        machine.arm()


def test_connect_cannot_bypass_disarmed_recovery() -> None:
    machine = TeleopStateMachine()
    arm_machine(machine)
    machine.disarm()

    with pytest.raises(RuntimeError, match="connect_requires_disconnected"):
        machine.connect()

    assert machine.mode == TeleopMode.DISARMED


def test_disarmed_state_cannot_be_armed_directly() -> None:
    machine = TeleopStateMachine()
    arm_machine(machine)
    machine.disarm()
    assert machine.mode == TeleopMode.DISARMED
    with pytest.raises(RuntimeError, match="arm_requires_grip_release"):
        machine.arm()
