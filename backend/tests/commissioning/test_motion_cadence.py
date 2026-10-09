import pytest

from app.commissioning.smoke import _next_motion_cycle


@pytest.mark.parametrize(
    "previous_deadline,start,finished,expected_deadline,expected_wait",
    [
        (0.0, 0.0, 0.005, 0.020, 0.015),
        (0.020, 0.025, 0.030, 0.040, 0.010),
        (0.020, 0.050, 0.055, 0.050, 0.0),
        (0.050, 0.085, 0.090, 0.085, 0.0),
        (0.020, 0.025, 0.060, 0.040, 0.0),
    ],
)
def test_motion_cycle_accounts_for_previous_reads_and_tick_work(
    previous_deadline, start, finished, expected_deadline, expected_wait,
):
    deadline, delay = _next_motion_cycle(previous_deadline, 0.020, start, finished)
    assert deadline == pytest.approx(expected_deadline)
    assert delay == pytest.approx(expected_wait)


def test_long_state_read_does_not_create_catch_up_sleep_debt():
    deadline, delay = _next_motion_cycle(0.020, 0.020, 1.0, 1.005)
    assert deadline == 1.0
    assert delay == 0.0
    deadline, delay = _next_motion_cycle(deadline, 0.020, 1.035, 1.040)
    assert deadline == 1.035
    assert delay == 0.0
