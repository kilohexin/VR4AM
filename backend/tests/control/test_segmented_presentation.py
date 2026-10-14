"""Display metadata is never segment permission or physical-stop evidence."""
import pytest
from pydantic import ValidationError

from tests.control.test_segmented_control import rig, send, observe, OPTIONS
from tests.robots.test_lebai_adapter_control import _target


async def test_display_reads_do_not_consume_receipt_or_feedback_gate():
    adapter, _client, _clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        before = dict(adapter._segment)
        for _ in range(4):
            status = strategy.status_message()
            assert status.policy == 'segmented_experimental'
            assert status.phase == 'awaiting_reply'
            assert status.hardware_verified is False
        assert adapter._segment == before
        assert strategy.committed_command is None
        strategy.collect(box[0])
        assert strategy.status_message().phase == 'awaiting_window'
    finally:
        await adapter.disconnect()


async def test_status_reports_distinct_fresh_samples_without_standstill_claim():
    adapter, client, clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        clock.value += 80_000_000
        for count in (1,2,3):
            await observe(adapter, client, clock, box, strategy)
            status = strategy.status_message()
            assert status.samples_observed == count and status.samples_required == 3
            assert status.phase == ('ready' if count == 3 else 'awaiting_feedback')
        strategy.on_revoke(2)
        assert strategy.status_message().phase == 'inactive'
    finally:
        await adapter.disconnect()


async def test_stop_fault_dominates_display_of_last_ready_segment():
    adapter, client, clock, box, strategy = await rig()
    try:
        await send(strategy, adapter, _target(.31), 1)
        strategy.collect(box[0])
        adapter._latched_fault = 'stop_unverified'
        assert strategy.status_message().phase == 'fault'
    finally:
        await adapter.disconnect()


@pytest.mark.parametrize('change', [dict(hardware_verified=True),dict(samples_observed=4),
    dict(samples_observed=True),dict(samples_required=2),dict(phase='stopped'),dict(extra=True)])
def test_wire_status_fails_closed_on_false_claims_and_bad_counts(change):
    from app.schemas.messages import TargetControlStatus
    with pytest.raises(ValidationError):
        TargetControlStatus(**{
            'policy':'segmented_experimental','phase':'awaiting_feedback',
            'samples_observed':0,'samples_required':3,'hardware_verified':False, **change})


async def test_state_message_carries_status_without_advancing_segment():
    from app.control.robot_control import RobotControl, LatestVRFrame
    from app.control.transactional_settings import SegmentedSettings
    from app.recording.noop import NoopRecorder
    adapter, _client, clock, box, strategy = await rig()
    try:
        control = RobotControl(backend=adapter, latest=LatestVRFrame(), clock=clock,
            recorder=NoopRecorder(), segmented_settings=SegmentedSettings(OPTIONS))
        control.target_strategy = strategy
        message = control._compose_state_message(await adapter.get_state())
        assert message.target_control.policy == 'segmented_experimental'
        assert message.target_control.phase == 'ready'
        assert adapter._segment is None
    finally:
        await adapter.disconnect()
