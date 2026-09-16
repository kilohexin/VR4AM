"""Offline evidence evaluation: no SDK, no escalation policy or motion permission."""
import pytest

from tests.prototypes.stop_evidence_window import EvidenceWindow, Sample


def sample(t, x=0.0, q=0.0, qd=0.0, state="IDLE", fault=False):
    return Sample(t, (q,) * 6, (qd,) * 6, (x, 0., 0.), state, fault)


def window():
    # Explicit test budgets, not certified hardware limits.
    return EvidenceWindow(stable_ms=300, max_gap_ms=250,
                          joint_drift=0.001, tcp_drift=0.0005, speed=0.02)


def test_stationarity_is_independent_of_rpc_reply():
    w = window()
    assert not w.observe(sample(0))
    assert not w.observe(sample(200))
    assert w.observe(sample(400))  # No RPC reply is supplied or required.
    assert not w.motion_allowed  # Evidence is never permission to move.


def test_stop_status_does_not_hide_position_drift():
    w = window()
    for t in (0, 200, 400):
        w.observe(sample(t, state="STOP"))
    assert w.confirmed
    assert not w.observe(sample(600, x=-0.022, state="STOP"))


def test_drift_accumulates_against_window_anchor_not_previous_sample():
    w = window()
    w.observe(sample(0))
    w.observe(sample(150, x=0.0003))
    assert not w.observe(sample(300, x=0.0006))


def test_missing_samples_revoke_old_evidence():
    w = window()
    for t in (0, 200, 400):
        w.observe(sample(t))
    assert not w.fresh_confirmation(651)
    assert not w.observe(sample(800))


@pytest.mark.parametrize("bad", [sample(600, qd=0.03), sample(600, state="MOVING"),
                                 sample(600, fault=True), sample(600, q=float('nan'))])
def test_bad_snapshot_revokes_confirmation(bad):
    w = window()
    for t in (0, 200, 400):
        w.observe(sample(t))
    assert not w.observe(bad)


def test_duplicate_or_rewound_samples_cannot_fill_stable_window():
    w = window()
    w.observe(sample(200))
    with pytest.raises(ValueError, match="monotonic"):
        w.observe(sample(200))
    with pytest.raises(ValueError, match="monotonic"):
        w.observe(sample(100))
