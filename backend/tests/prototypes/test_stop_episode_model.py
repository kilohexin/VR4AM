import pytest

from tests.prototypes.stop_episode_model import StopEpisode


def test_repeated_cleanup_reuses_request_without_resetting_deadlines():
    model = StopEpisode()
    assert model.request_stop('grip', 0) == 'PROPOSE_STOP_MOVE'
    assert model.request_stop('shutdown', 150) == 'REUSE_EPISODE'
    assert model.request_stop('disconnect', 190) == 'REUSE_EPISODE'
    assert model.request_count == 1
    assert model.tick(200) == 'REVIEW_ESCALATION'
    assert model.rpc_state == 'UNKNOWN'
    assert not model.motion_allowed


def test_late_reply_is_evidence_not_fault_reset_or_motion_permission():
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.tick(200)
    model.reply(1, 450)
    assert model.rpc_state == 'RETURNED_LATE'
    assert model.fault_latched
    assert not model.stop_confirmed
    model.observe(460, 'IDLE', stable_window_complete=False)
    assert not model.stop_confirmed
    model.observe(650, 'IDLE', stable_window_complete=True)
    assert model.stop_confirmed
    assert model.fault_latched
    assert not model.motion_allowed


@pytest.mark.parametrize('state', ['STOPPING', 'MOVING', 'UNKNOWN', 'STOP'])
def test_robot_state_alone_cannot_confirm_stop(state):
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.reply(1, 100)
    model.observe(150, state, stable_window_complete=False)
    assert not model.stop_confirmed
    assert model.tick(700) == 'REVIEW_ESCALATION'
    assert model.fault_latched


@pytest.mark.parametrize('cause', ['DISCONNECTED', 'CANCELLED'])
def test_disconnect_and_cancellation_keep_request_unknown(cause):
    model = StopEpisode()
    model.request_stop('grip', 0)
    assert model.interrupt(100, cause) == 'REVIEW_ESCALATION'
    assert model.rpc_state == 'UNKNOWN'
    assert model.request_stop('shutdown', 110) == 'REUSE_EPISODE'
    assert model.fault_latched
    assert not model.motion_allowed


def test_timely_reply_still_needs_independent_stop_evidence():
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.reply(1, 100)
    assert not model.stop_confirmed
    model.observe(400, 'IDLE', stable_window_complete=True)
    assert model.stop_confirmed
    assert not model.fault_latched
    assert not model.motion_allowed  # Stopping never grants a motion permission.
    assert model.tick(800) == 'NO_ACTION'


def test_wrong_request_reply_cannot_confirm_or_reset_deadline():
    model = StopEpisode()
    model.request_stop('grip', 0)
    assert model.reply(99, 100) == 'UNMATCHED_REPLY'
    assert model.rpc_state == 'SENT'
    assert model.tick(200) == 'REVIEW_ESCALATION'


def test_position_change_revokes_prior_confirmation():
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.reply(1, 80)
    model.observe(400, 'IDLE', stable_window_complete=True)
    assert model.observe(500, 'STOP', stable_window_complete=False,
                         position_changed=True) == 'REVIEW_ESCALATION'
    assert not model.stop_confirmed
    assert model.fault_latched


def test_confirmed_stationarity_does_not_resolve_missing_rpc_reply():
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.observe(150, 'IDLE', stable_window_complete=True)
    assert model.stop_confirmed
    assert model.tick(200) == 'REVIEW_ESCALATION'
    assert model.rpc_state == 'UNKNOWN'
    assert not model.motion_allowed


def test_cleanup_reuse_cannot_hide_pending_escalation():
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.tick(200)
    model.request_stop('shutdown', 300)
    assert model.tick(300) == 'REVIEW_ESCALATION'
    assert model.request_count == 1
    assert model.fault_latched


@pytest.mark.parametrize('bad_time', [-1, 99, 100.5, True])
def test_invalid_event_time_cannot_rewind_episode(bad_time):
    model = StopEpisode()
    model.request_stop('grip', 100)
    with pytest.raises(ValueError, match='invalid_monotonic_time'):
        model.tick(bad_time)
    assert model.tick(300) == 'REVIEW_ESCALATION'


def test_reply_at_deadline_is_unknown_then_late_not_timely():
    model = StopEpisode()
    model.request_stop('grip', 0)
    model.reply(1, 200)
    assert model.rpc_state == 'RETURNED_LATE'
    assert model.fault_latched
