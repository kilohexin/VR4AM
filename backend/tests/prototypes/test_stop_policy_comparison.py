"""Research policy only: no SDK calls or permissions to resume movement."""
import pytest

from tests.prototypes.stop_evidence_window import Sample
from tests.prototypes.stop_policy_comparison import StopPolicyComparison


def sample(ms, x=0., state="IDLE", fault=False):
    return Sample(ms, (0.,) * 6, (0.,) * 6, (x, 0., 0.), state, fault)


def model():
    # Illustrative budget, NOT a proposed production configuration.
    return StopPolicyComparison(reply_ms=200, research_budget_ms=1000)


def test_slow_reply_with_fresh_stationarity_keeps_fault_and_motion_blocked():
    m = model()
    for t in (0, 100, 200, 300):
        m.observe(sample(t))
    assert m.assess(300) == "STATIONARY_RPC_UNRESOLVED"
    assert m.fault_latched
    m.reply(625)
    assert m.assess(625) == "PROTECTION_REVIEW"  # Last sample is stale.
    assert not m.motion_allowed


def test_late_reply_does_not_clear_fault_even_with_fresh_evidence():
    m = model()
    for t in range(0, 701, 100):
        m.observe(sample(t))
    m.reply(700)
    assert m.assess(700) == "STATIONARY_REPLY_RETURNED"
    assert m.fault_latched
    assert not m.motion_allowed


def test_no_post_stop_samples_cannot_justify_waiting_at_rpc_deadline():
    m = model()
    assert m.assess(200) == "PROTECTION_REVIEW"


def test_200ms_of_samples_cannot_fill_300ms_stable_window():
    m = model()
    for t in (0, 100, 200):
        m.observe(sample(t))
    assert m.assess(200) == "AWAIT_WITHIN_RESEARCH_BUDGET"
    assert not m.window.confirmed
    assert not m.motion_allowed


@pytest.mark.parametrize("bad", [sample(400, x=.022), sample(400, state="MOVING"),
                                  sample(400, fault=True)])
def test_motion_or_fault_revokes_evidence_and_latches_review(bad):
    m = model()
    for t in (0, 100, 200, 300):
        m.observe(sample(t))
    m.observe(bad)
    assert m.assess(400) == "PROTECTION_REVIEW"
    # Subsequent stable samples must not silently undo an unsafe observation.
    for t in (500, 600, 700, 800):
        m.observe(sample(t))
    assert m.assess(800) == "PROTECTION_REVIEW"


def test_feedback_loss_latches_review_without_waiting_for_rpc_reply():
    m = model()
    for t in (0, 100, 200, 300):
        m.observe(sample(t))
    assert m.assess(551) == "PROTECTION_REVIEW"
    m.reply(600)
    m.observe(sample(600))
    assert m.assess(600) == "PROTECTION_REVIEW"


def test_unresolved_rpc_cannot_extend_research_budget_indefinitely():
    m = model()
    for t in range(0, 1001, 100):
        m.observe(sample(t))
    assert m.assess(1000) == "PROTECTION_REVIEW"


def test_stopping_state_does_not_count_as_confirmed_stationarity():
    m = model()
    for t in (0, 100, 200, 300):
        m.observe(sample(t, state="STOPPING"))
    assert m.assess(300) == "AWAIT_WITHIN_RESEARCH_BUDGET"
    assert not m.window.confirmed


def test_out_of_order_events_are_rejected():
    m = model()
    m.observe(sample(200))
    with pytest.raises(ValueError, match="monotonic"):
        m.reply(100)


def test_gap_cannot_be_hidden_by_a_new_sample_before_assessment():
    m = model()
    m.observe(sample(0))
    m.observe(sample(400))
    assert m.assess(400) == "PROTECTION_REVIEW"


def test_initial_drift_is_not_hidden_by_later_stable_window():
    m = model()
    m.observe(sample(0))
    for t in (100, 200, 300, 400):
        m.observe(sample(t, x=.001))
    m.reply(450)
    assert m.assess(450) == "PROTECTION_REVIEW"


@pytest.mark.parametrize("budgets", [(200, 200), (0, 1000), (200, float('inf'))])
def test_invalid_research_budgets_are_rejected(budgets):
    with pytest.raises(ValueError, match="budgets"):
        StopPolicyComparison(reply_ms=budgets[0], research_budget_ms=budgets[1])


def test_reply_after_budget_cannot_hide_expiry_between_assessments():
    m = model()
    for t in range(0, 901, 100):
        m.observe(sample(t))
    m.reply(1001)
    assert m.assess(1001) == "PROTECTION_REVIEW"


def test_first_stable_sample_after_budget_cannot_backdate_confirmation():
    m = model()
    m.reply(100)
    for t in (500, 700, 900):
        m.observe(sample(t, state="STOPPING"))
    for t in (1001, 1101, 1201, 1301):
        m.observe(sample(t))
    assert m.assess(1301) == "PROTECTION_REVIEW"
