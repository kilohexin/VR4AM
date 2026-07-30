from __future__ import annotations

import json
import math
from collections import deque
from collections.abc import Mapping
from typing import Literal

from app.schemas.messages import (
    DiagnosticEvent,
    DiagnosticsMessage,
    JointVector,
    Pose,
    RobotStateMessage,
    RuntimeBackend,
)

PVAT_RATE_WINDOW_NS = 2_000_000_000


class DiagnosticsStore:
    def __init__(
        self,
        *,
        capacity: int = 200,
        normal_interval_ns: int = 200_000_000,
    ) -> None:
        if capacity <= 0:
            raise ValueError("diagnostics_capacity_must_be_positive")
        if normal_interval_ns < 0:
            raise ValueError("diagnostics_normal_interval_must_not_be_negative")
        self.capacity = capacity
        self.normal_interval_ns = normal_interval_ns
        self._events: deque[DiagnosticEvent] = deque()
        self._next_event_id = 1
        self.dropped_events = 0
        self._actual_qd: JointVector | None = None
        self._actual_qdd: JointVector | None = None
        self._target_q: JointVector | None = None
        self._target_qd: JointVector | None = None
        self._target_qdd: JointVector | None = None
        self._target_tcp: Pose | None = None
        self._sdk_latencies_ms: dict[str, float] = {}
        self._pvat_sent_ns: deque[int] = deque()

    def observe_event(
        self,
        event: object,
        server_mono_ns: int,
        *,
        critical: bool,
    ) -> None:
        if server_mono_ns < 0:
            raise ValueError("diagnostic timestamp must be nonnegative")
        payload = _event_mapping(event)
        kind = payload.pop("kind", None)
        if not isinstance(kind, str) or not kind.strip():
            raise ValueError("diagnostic event kind must be nonempty")
        kind = kind.strip()
        event_payload = _event_payload(payload)
        diagnostic_event = DiagnosticEvent(
            event_id=self._next_event_id,
            server_mono_ns=server_mono_ns,
            kind=kind,
            critical=critical,
            payload=event_payload,
        )
        self._next_event_id += 1

        self._observe_metrics(kind, event_payload, server_mono_ns)
        if self._can_coalesce(diagnostic_event):
            self._events[-1] = diagnostic_event
            return
        if len(self._events) >= self.capacity:
            self._events.popleft()
            self.dropped_events += 1
        self._events.append(diagnostic_event)

    def observe_robot_state(
        self,
        state: RobotStateMessage,
        server_mono_ns: int,
    ) -> None:
        self.observe_event(
            {
                "kind": "robot_state_sample",
                "state": state.model_dump(mode="json"),
            },
            server_mono_ns,
            critical=False,
        )

    def message(
        self,
        *,
        runtime: RuntimeBackend,
        hardware_verified: Literal[False],
        server_mono_ns: int,
        control_generation: int,
        log_session_dir: str | None,
    ) -> DiagnosticsMessage:
        if server_mono_ns < 0:
            raise ValueError("diagnostic timestamp must be nonnegative")
        if control_generation < 0:
            raise ValueError("control generation must be nonnegative")
        return DiagnosticsMessage(
            server_mono_ns=server_mono_ns,
            runtime=runtime,
            hardware_verified=hardware_verified,
            control_generation=control_generation,
            actual_qd=self._actual_qd,
            actual_qdd=self._actual_qdd,
            target_q=self._target_q,
            target_qd=self._target_qd,
            target_qdd=self._target_qdd,
            target_tcp=(
                None
                if self._target_tcp is None
                else self._target_tcp.model_copy(deep=True)
            ),
            sdk_latencies_ms=dict(self._sdk_latencies_ms),
            pvat_send_hz=self._pvat_send_hz(server_mono_ns),
            log_session_dir=log_session_dir,
            dropped_events=self.dropped_events,
            recent_events=tuple(
                event.model_copy(deep=True) for event in tuple(self._events)[-20:]
            ),
        )

    def _can_coalesce(self, event: DiagnosticEvent) -> bool:
        if event.critical or not self._events:
            return False
        previous = self._events[-1]
        return (
            not previous.critical
            and previous.kind == event.kind
            and event.server_mono_ns - previous.server_mono_ns
            <= self.normal_interval_ns
        )

    def _observe_metrics(
        self,
        kind: str,
        payload: dict[str, object],
        server_mono_ns: int,
    ) -> None:
        if kind == "robot_kinematics":
            self._actual_qd = _joint(payload.get("actual_qd"), "actual_qd")
            self._actual_qdd = _joint(payload.get("actual_qdd"), "actual_qdd")
            self._target_q = _joint(payload.get("target_q"), "target_q")
            self._target_qd = _joint(payload.get("target_qd"), "target_qd")
            self._target_qdd = _joint(payload.get("target_qdd"), "target_qdd")
            target_tcp = payload.get("target_tcp")
            if target_tcp is not None:
                self._target_tcp = Pose.model_validate(target_tcp)
            sdk_latencies = payload.get("sdk_latencies_ms")
            if sdk_latencies is not None:
                self._sdk_latencies_ms = _latencies(sdk_latencies)
        elif kind == "pvat_sent":
            sdk_latency = payload.get("sdk_latency_ms")
            if sdk_latency is not None:
                self._sdk_latencies_ms["move_pvat"] = _latency(sdk_latency)
            self._pvat_sent_ns.append(server_mono_ns)
            self._prune_pvat(server_mono_ns)

    def _prune_pvat(self, now_ns: int) -> None:
        cutoff = now_ns - PVAT_RATE_WINDOW_NS
        while self._pvat_sent_ns and self._pvat_sent_ns[0] < cutoff:
            self._pvat_sent_ns.popleft()

    def _pvat_send_hz(self, now_ns: int) -> float | None:
        self._prune_pvat(now_ns)
        if len(self._pvat_sent_ns) < 2:
            return None
        interval_ns = self._pvat_sent_ns[-1] - self._pvat_sent_ns[0]
        if interval_ns <= 0:
            return 0.0
        return (len(self._pvat_sent_ns) - 1) * 1_000_000_000 / interval_ns


def _event_mapping(event: object) -> dict[str, object]:
    normalized = _json_value(event)
    if not isinstance(normalized, dict):
        raise ValueError("diagnostic event must be an object")
    return normalized


def _event_payload(event: dict[str, object]) -> dict[str, object]:
    if set(event) == {"payload"} and isinstance(event["payload"], dict):
        return dict(event["payload"])
    return dict(event)


def _json_value(value: object) -> object:
    if hasattr(value, "model_dump"):
        return _json_value(value.model_dump(mode="json"))
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("diagnostic values must be finite")
        return value
    if value is None or isinstance(value, (str, bool, int)):
        return value
    try:
        encoded = json.dumps(value, allow_nan=False)
    except (TypeError, ValueError, OverflowError):
        raise ValueError("diagnostic values must be JSON safe") from None
    return json.loads(encoded)


def _joint(value: object, field: str) -> JointVector | None:
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{field} must be a six-axis joint vector")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{field} must contain only finite values")
    return result  # type: ignore[return-value]


def _latencies(value: object) -> dict[str, float]:
    if not isinstance(value, Mapping):
        raise ValueError("sdk_latencies_ms must be an object")
    result: dict[str, float] = {}
    for name, latency in value.items():
        result[str(name)] = _latency(latency)
    return result


def _latency(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("SDK latency must be numeric")
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError("SDK latency must be finite and nonnegative")
    return number
