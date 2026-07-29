import pytest

from app.diagnostics.store import DiagnosticsStore


def _message(store: DiagnosticsStore, *, server_mono_ns: int = 6):
    return store.message(
        runtime="LEBAI_FAKE",
        hardware_verified=False,
        server_mono_ns=server_mono_ns,
        control_generation=2,
        log_session_dir=None,
    )


def test_store_is_bounded_and_keeps_newest_events() -> None:
    store = DiagnosticsStore(capacity=3, normal_interval_ns=0)
    for index in range(5):
        store.observe_event(
            {"kind": "normal", "index": index},
            server_mono_ns=index,
            critical=False,
        )
    message = _message(store)
    assert [event.payload["index"] for event in message.recent_events] == [2, 3, 4]
    assert message.dropped_events == 2


def test_normal_events_are_coalesced_but_critical_events_are_retained() -> None:
    store = DiagnosticsStore(capacity=10, normal_interval_ns=200_000_000)
    store.observe_event({"kind": "robot_kinematics", "n": 1}, 0, critical=False)
    store.observe_event({"kind": "robot_kinematics", "n": 2}, 1, critical=False)
    store.observe_event({"kind": "stop_requested"}, 2, critical=True)
    message = store.message(
        runtime="LEBAI",
        hardware_verified=False,
        server_mono_ns=3,
        control_generation=4,
        log_session_dir="logs/commissioning/session",
    )
    assert [event.kind for event in message.recent_events] == [
        "robot_kinematics",
        "stop_requested",
    ]


def test_store_tracks_finite_kinematics_and_two_second_pvat_rate() -> None:
    store = DiagnosticsStore(normal_interval_ns=0)
    store.observe_event(
        {
            "kind": "robot_kinematics",
            "actual_qd": [0.0] * 6,
            "actual_qdd": [0.0] * 6,
            "target_q": [0.0] * 6,
            "target_qd": [0.0] * 6,
            "target_qdd": [0.0] * 6,
            "target_tcp": {"p": [0.3, 0.0, 0.4], "q": [0.0, 0.0, 0.0, 1.0]},
            "sdk_latencies_ms": {"get_kin_data": 4.5},
        },
        0,
        critical=False,
    )
    store.observe_event(
        {"kind": "pvat_sent", "command_id": 1, "sdk_latency_ms": 7.0},
        0,
        critical=False,
    )
    store.observe_event({"kind": "pvat_sent", "command_id": 2}, 1_000_000_000, critical=False)

    message = _message(store, server_mono_ns=1_000_000_000)

    assert message.actual_qd == (0.0,) * 6
    assert message.target_tcp is not None
    assert message.sdk_latencies_ms == {"get_kin_data": 4.5, "move_pvat": 7.0}
    assert message.pvat_send_hz == pytest.approx(1.0)

    with pytest.raises(ValueError, match="finite"):
        store.observe_event(
            {"kind": "robot_kinematics", "sdk_latencies_ms": {"bad": float("nan")}},
            2_000_000_000,
            critical=False,
        )


def test_store_messages_expose_at_most_twenty_newest_events() -> None:
    store = DiagnosticsStore(capacity=30, normal_interval_ns=0)
    for index in range(25):
        store.observe_event(
            {"kind": f"sample_{index}", "index": index},
            index,
            critical=False,
        )

    assert [event.payload["index"] for event in _message(store).recent_events] == list(
        range(5, 25)
    )
