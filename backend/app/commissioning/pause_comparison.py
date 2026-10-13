"""Isolated commissioning comparison; never selected by normal VR config."""
from dataclasses import replace
import math

from app.robots.base import BackendCommandError
from app.robots.lebai_adapter import RealLebaiAdapter, _STOP_JOINT_DRIFT_RAD, _STOP_TCP_DRIFT_M


class PauseComparisonAdapter(RealLebaiAdapter):
    """Retain original deadline, ownership, verification, fault and tail logic.

    Pause retains controller motion state. There is deliberately no resume,
    reset or implicit stop_move fallback. PAUSED must remain motion-ineligible.
    """
    _motion_stop_rpc_method = "pause_move"

    async def preflight(self):
        result = await super().preflight()
        if not callable(getattr(self._client, "pause_move", None)):
            self._preflight_ready = False
            return replace(result, ready=False, reason="pause_move_unavailable")
        return result

    async def stop(self, reason):
        if not callable(getattr(self._client, "pause_move", None)):
            self._latch_unverified_stop()
            raise BackendCommandError("pause_move_unavailable")
        await super().stop(reason)


def build_pause_backend(settings, recorder, client_factory, *, stop_diagnostic_sampling=False):
    if settings.backend != "lebai" or settings.lebai is None or settings.lebai.mode != "control":
        raise ValueError("pause_comparison_requires_control_lebai")
    return PauseComparisonAdapter(
        settings.lebai, client_factory=client_factory,
        event_callback=recorder.write_event,
        stop_diagnostic_sampling=stop_diagnostic_sampling,
    )


def pause_is_settled(backend, state):
    """Post-pause read must still agree with the original verification anchor.

    This is diagnostic acceptance, not permission to resume or arm. Generic
    HOLD and raw STOP are deliberately not accepted as successful PAUSED.
    """
    if not isinstance(backend, PauseComparisonAdapter) or state.fault is not None:
        return False
    transaction, sample = backend._stop_transaction, backend._snapshot
    if (transaction is None or not transaction.confirmed or sample is None
            or transaction.stable_anchor is None or backend._latched_fault is not None):
        return False
    raw = sample.raw_robot_state
    raw = raw.strip().upper() if isinstance(raw, str) else raw
    anchor = transaction.stable_anchor
    return (
        raw in {"IDLE", "PAUSED", 5, 6}
        and sample.estop is None and sample.running_motion is None
        and max(abs(v) for v in sample.actual_qd) <= .02
        and max(abs(q - a) for q, a in zip(sample.actual_q, anchor.actual_q)) <= _STOP_JOINT_DRIFT_RAD
        and math.dist(sample.actual_tcp.p, anchor.actual_tcp.p) <= _STOP_TCP_DRIFT_M
    )
