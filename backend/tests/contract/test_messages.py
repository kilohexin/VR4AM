import json
import math
from pathlib import Path
from typing import Any, get_args

import pytest
from jsonschema import Draft202012Validator
from pydantic import BaseModel, ValidationError

from app.schemas import messages
from app.schemas.messages import (
    ClientControlMessage,
    SetSimulationScaleMessage,
    SimulationScaleResultMessage,
    DiagnosticsMessage,
    RobotStateMessage,
    VRFrame,
)

ROOT = Path(__file__).resolve().parents[3]


def load_fixture(name: str) -> dict:
    return json.loads((ROOT / "schemas" / "fixtures" / name).read_text(encoding="utf-8"))


def load_message_payload(name: str) -> dict:
    if name == "client-control":
        return {
            "v": 1,
            "type": "ping",
            "request_id": "fixture",
            "client_mono_ms": 10.0,
        }
    return load_fixture(name)


def set_nested(payload: dict, path: tuple[str | int, ...], value: Any) -> None:
    target: Any = payload
    for component in path[:-1]:
        target = target[component]
    target[path[-1]] = value


def load_protocol_schema() -> dict:
    return json.loads((ROOT / "schemas" / "teleop-v1.json").read_text(encoding="utf-8-sig"))


def local_refs(value: Any) -> list[str]:
    if isinstance(value, dict):
        refs = [value["$ref"]] if isinstance(value.get("$ref"), str) else []
        return refs + [ref for child in value.values() for ref in local_refs(child)]
    if isinstance(value, list):
        return [ref for child in value for ref in local_refs(child)]
    return []


def assert_local_ref_resolves(schema: dict, ref: str) -> None:
    assert ref.startswith("#/"), f"expected a local JSON Pointer, got {ref}"
    target: Any = schema
    for raw_component in ref[2:].split("/"):
        component = raw_component.replace("~1", "/").replace("~0", "~")
        assert isinstance(target, dict) and component in target, f"unresolved local $ref: {ref}"
        target = target[component]


def test_valid_vr_frame_fixture_round_trips() -> None:
    frame = VRFrame.model_validate(load_fixture("vr-frame-valid.json"))
    assert frame.v == 1
    assert frame.right.q == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_valid_robot_state_fixture_round_trips() -> None:
    state = RobotStateMessage.model_validate(load_fixture("robot-state-valid.json"))
    assert state.mode == "ARMED"
    assert len(state.actual_q) == 6
    assert state.backend is None
    assert state.real_robot_mode is None
    assert state.preflight_ready is None
    assert state.preflight_reason is None


def test_diagnostics_fixture_round_trips_exactly() -> None:
    payload = load_fixture("diagnostics-valid.json")
    message = DiagnosticsMessage.model_validate(payload)

    assert message.model_dump(mode="json") == payload


def test_rehearsal_messages_accept_only_the_exact_lifecycle_keys() -> None:
    begin = {
        "v": 1,
        "type": "offline_rehearsal_begin",
        "request_id": "begin-1",
        "plan_version": 1,
    }
    finish = {
        "v": 1,
        "type": "offline_rehearsal_finish",
        "request_id": "finish-1",
        "run_id": "run-1",
        "outcome": "failed",
        "failure": {"reason": "offline_interlock"},
    }

    assert (
        messages.OfflineRehearsalBeginMessage.model_validate(begin).plan_version
        == 1
    )
    assert (
        messages.OfflineRehearsalFinishMessage.model_validate(finish).outcome
        == "failed"
    )
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalBeginMessage.model_validate(
            {**begin, "client_mono_ms": 1.0}
        )
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalFinishMessage.model_validate(
            {**finish, "target": {}}
        )


def test_rehearsal_phase_rejects_motion_fields_and_non_finite_metrics() -> None:
    valid = {
        "v": 1,
        "type": "offline_rehearsal_phase",
        "request_id": "phase-1",
        "run_id": "run-1",
        "phase": "home",
        "status": "passed",
        "started_client_ms": 10.0,
        "completed_client_ms": 20.0,
        "target": {},
        "measurements": {"max_error": 0.0},
        "failure": None,
    }

    messages.OfflineRehearsalPhaseMessage.model_validate(valid)
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalPhaseMessage.model_validate(
            {**valid, "target_q": [0] * 6}
        )
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalPhaseMessage.model_validate(
            {**valid, "measurements": {"max_error": math.nan}}
        )


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("request_id", ""),
        ("request_id", "界" * 65),
        ("run_id", ""),
        ("run_id", "界" * 65),
        ("phase", "unknown"),
        ("status", "aborted"),
    ],
)
def test_rehearsal_phase_enforces_ids_and_literals(field: str, value: str) -> None:
    payload = {
        "v": 1,
        "type": "offline_rehearsal_phase",
        "request_id": "界" * 64,
        "run_id": "界" * 64,
        "phase": "identity_preflight",
        "status": "passed",
        "started_client_ms": 0.0,
        "completed_client_ms": 1.0,
        "target": {},
        "measurements": {},
        "failure": None,
    }

    messages.OfflineRehearsalPhaseMessage.model_validate(payload)
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalPhaseMessage.model_validate(
            {**payload, field: value}
        )


@pytest.mark.parametrize("outcome", ["passed", "failed", "aborted"])
def test_rehearsal_finish_accepts_only_declared_outcomes(outcome: str) -> None:
    payload = {
        "v": 1,
        "type": "offline_rehearsal_finish",
        "request_id": "finish-1",
        "run_id": "run-1",
        "outcome": outcome,
        "failure": None,
    }

    assert (
        messages.OfflineRehearsalFinishMessage.model_validate(payload).outcome
        == outcome
    )
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalFinishMessage.model_validate(
            {**payload, "outcome": "unknown"}
        )


@pytest.mark.parametrize("plan_version", [0, 2, True])
def test_rehearsal_begin_requires_plan_version_one(plan_version: object) -> None:
    with pytest.raises(ValidationError):
        messages.OfflineRehearsalBeginMessage.model_validate(
            {
                "v": 1,
                "type": "offline_rehearsal_begin",
                "request_id": "begin-1",
                "plan_version": plan_version,
            }
        )


@pytest.mark.parametrize("bad_value", [-0.1, float("nan"), float("inf")])
def test_diagnostics_rejects_invalid_sdk_latency(bad_value: float) -> None:
    payload = load_fixture("diagnostics-valid.json")
    payload["sdk_latencies_ms"]["get_kin_data"] = bad_value

    with pytest.raises(ValidationError):
        DiagnosticsMessage.model_validate(payload)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
def test_diagnostics_rejects_non_finite_pvat_rate(bad_value: float) -> None:
    payload = load_fixture("diagnostics-valid.json")
    payload["pvat_send_hz"] = bad_value

    with pytest.raises(ValidationError):
        DiagnosticsMessage.model_validate(payload)


def test_robot_state_accepts_optional_real_backend_diagnostics() -> None:
    payload = load_fixture("robot-state-valid.json")
    payload.update(
        {
            "backend": "LEBAI",
            "real_robot_mode": "readonly",
            "preflight_ready": False,
            "preflight_reason": "real_robot_readonly",
        }
    )

    state = RobotStateMessage.model_validate(payload)
    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": "#/$defs/RobotStateMessage"}
    )

    assert state.backend == "LEBAI"
    assert state.real_robot_mode == "readonly"
    assert state.preflight_ready is False
    assert not list(validator.iter_errors(payload))


def test_robot_state_accepts_digital_twin_backend_diagnostics() -> None:
    payload = load_fixture("robot-state-valid.json")
    payload["backend"] = "LEBAI_FAKE"

    state = RobotStateMessage.model_validate(payload)
    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": "#/$defs/RobotStateMessage"}
    )

    assert state.backend == "LEBAI_FAKE"
    assert not list(validator.iter_errors(payload))


def test_robot_state_contract_rejects_unknown_runtime_backend() -> None:
    payload = load_fixture("robot-state-valid.json")
    payload["backend"] = "LEBAI_MOCK"
    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": "#/$defs/RobotStateMessage"}
    )

    with pytest.raises(ValidationError):
        RobotStateMessage.model_validate(payload)
    assert list(validator.iter_errors(payload))


def test_self_collision_is_a_valid_robot_state_constraint() -> None:
    payload = load_fixture("robot-state-valid.json")
    payload["constraint"] = "self_collision"

    state = RobotStateMessage.model_validate(payload)
    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": "#/$defs/RobotStateMessage"}
    )

    assert state.constraint == "self_collision"


def test_motion_continuity_is_a_valid_robot_state_constraint() -> None:
    payload = load_fixture("robot-state-valid.json")
    payload["constraint"] = "motion_continuity_boundary"

    state = RobotStateMessage.model_validate(payload)
    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": "#/$defs/RobotStateMessage"}
    )

    assert state.constraint == "motion_continuity_boundary"
    assert not list(validator.iter_errors(payload))


def test_robot_state_allows_positive_readonly_real_translation_scale() -> None:
    payload = load_fixture("robot-state-valid.json")
    payload["backend"] = "LEBAI"
    payload["translation_scale"] = 0.3

    state = RobotStateMessage.model_validate(payload)

    assert state.translation_scale == pytest.approx(0.3)


@pytest.mark.parametrize("value", [0.5, 1.5, 2.0, 10.0])
def test_set_simulation_scale_accepts_exact_tenth_steps(value: float) -> None:
    message = SetSimulationScaleMessage.model_validate(
        {
            "v": 1,
            "type": "set_simulation_scale",
            "request_id": "scale-1",
            "translation_scale": value,
        }
    )
    assert message.translation_scale == value


@pytest.mark.parametrize("value", [0.49, 10.1, 1.55, float("nan"), True])
def test_set_simulation_scale_rejects_non_tenth_or_out_of_range_values(
    value: object,
) -> None:
    with pytest.raises(ValidationError):
        SetSimulationScaleMessage.model_validate(
            {
                "v": 1,
                "type": "set_simulation_scale",
                "request_id": "scale-1",
                "translation_scale": value,
            }
        )


def test_simulation_scale_result_requires_exact_correlated_outcome() -> None:
    accepted = SimulationScaleResultMessage.model_validate(
        {
            "v": 1,
            "type": "simulation_scale_result",
            "request_id": "scale-1",
            "accepted": True,
            "translation_scale": 1.5,
        }
    )
    assert accepted.reason is None
    with pytest.raises(ValidationError):
        SimulationScaleResultMessage.model_validate(
            {
                "v": 1,
                "type": "simulation_scale_result",
                "request_id": "scale-1",
                "accepted": False,
                "translation_scale": 1.5,
            }
        )


@pytest.mark.parametrize(
    ("definition", "payload"),
    [
        (
            "SetSimulationScaleMessage",
            {
                "v": 1,
                "type": "set_simulation_scale",
                "request_id": "scale-1",
                "translation_scale": 1.5,
            },
        ),
        (
            "SimulationScaleResultMessage",
            {
                "v": 1,
                "type": "simulation_scale_result",
                "request_id": "scale-1",
                "accepted": True,
                "translation_scale": 1.5,
            },
        ),
    ],
)
def test_protocol_schema_contains_simulation_scale_messages(
    definition: str,
    payload: dict[str, object],
) -> None:
    schema = load_protocol_schema()
    validator = Draft202012Validator(
        {**schema, "$ref": f"#/$defs/{definition}"}
    )
    assert not list(validator.iter_errors(payload))


@pytest.mark.parametrize("bad_q", [[0, 0, 0, 0], [0, 0, 0, 2], [float("nan"), 0, 0, 1]])
def test_vr_frame_rejects_invalid_quaternion(bad_q: list[float]) -> None:
    payload = load_fixture("vr-frame-valid.json")
    payload["right"]["q"] = bad_q
    with pytest.raises(ValidationError):
        VRFrame.model_validate(payload)


def test_vr_frame_rejects_non_finite_position() -> None:
    payload = load_fixture("vr-frame-valid.json")
    payload["right"]["p"][1] = float("inf")
    with pytest.raises(ValidationError):
        VRFrame.model_validate(payload)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize(
    ("model", "payload_name", "path"),
    [
        (VRFrame, "vr-frame-valid.json", ("client_mono_ms",)),
        (RobotStateMessage, "robot-state-valid.json", ("server_mono_ns",)),
        (RobotStateMessage, "robot-state-valid.json", ("sample_age_ms",)),
        (ClientControlMessage, "client-control", ("client_mono_ms",)),
    ],
)
def test_messages_reject_non_finite_time_fields(
    model: type[BaseModel], payload_name: str, path: tuple[str | int, ...], bad_value: float
) -> None:
    payload = load_message_payload(payload_name)
    set_nested(payload, path, bad_value)
    with pytest.raises(ValidationError):
        model.model_validate(payload)


@pytest.mark.parametrize("bad_value", [float("nan"), float("inf"), float("-inf")])
@pytest.mark.parametrize(
    ("model", "payload_name", "path"),
    [
        (VRFrame, "vr-frame-valid.json", ("right", "p", 0)),
        (VRFrame, "vr-frame-valid.json", ("right", "trigger")),
        (RobotStateMessage, "robot-state-valid.json", ("actual_tcp", "p", 0)),
        (RobotStateMessage, "robot-state-valid.json", ("gripper",)),
    ],
)
def test_messages_reject_non_finite_nested_numeric_fields(
    model: type[BaseModel], payload_name: str, path: tuple[str | int, ...], bad_value: float
) -> None:
    payload = load_message_payload(payload_name)
    set_nested(payload, path, bad_value)
    with pytest.raises(ValidationError):
        model.model_validate(payload)


@pytest.mark.parametrize(
    "bad_actual_q",
    [
        [0.0] * 5,
        [0.0] * 7,
        [float("nan"), 0.0, 0.0, 0.0, 0.0, 0.0],
        [float("inf"), 0.0, 0.0, 0.0, 0.0, 0.0],
        [float("-inf"), 0.0, 0.0, 0.0, 0.0, 0.0],
    ],
)
def test_robot_state_rejects_invalid_joint_vectors(bad_actual_q: list[float]) -> None:
    payload = load_fixture("robot-state-valid.json")
    payload["actual_q"] = bad_actual_q
    with pytest.raises(ValidationError):
        RobotStateMessage.model_validate(payload)


@pytest.mark.parametrize(
    ("model", "payload_name", "path"),
    [
        (VRFrame, "vr-frame-valid.json", ("unknown",)),
        (VRFrame, "vr-frame-valid.json", ("right", "unknown")),
        (RobotStateMessage, "robot-state-valid.json", ("unknown",)),
        (RobotStateMessage, "robot-state-valid.json", ("actual_tcp", "unknown")),
        (ClientControlMessage, "client-control", ("unknown",)),
    ],
)
def test_messages_reject_unknown_fields(
    model: type[BaseModel], payload_name: str, path: tuple[str | int, ...]
) -> None:
    payload = load_message_payload(payload_name)
    set_nested(payload, path, True)
    with pytest.raises(ValidationError):
        model.model_validate(payload)


def test_protocol_schema_has_only_resolvable_local_refs() -> None:
    schema = load_protocol_schema()
    refs = local_refs(schema)
    assert refs
    for ref in refs:
        assert_local_ref_resolves(schema, ref)


@pytest.mark.parametrize(
    ("definition", "fixture_name"),
    [
        ("VRFrame", "vr-frame-valid.json"),
        ("RobotStateMessage", "robot-state-valid.json"),
    ],
)
def test_protocol_schema_validates_shared_fixtures(definition: str, fixture_name: str) -> None:
    schema = load_protocol_schema()
    for ref in local_refs(schema):
        assert_local_ref_resolves(schema, ref)
    root_schema = {**schema, "$ref": f"#/$defs/{definition}"}
    Draft202012Validator.check_schema(root_schema)
    errors = list(Draft202012Validator(root_schema).iter_errors(load_fixture(fixture_name)))
    assert not errors, "\n".join(error.message for error in errors)


def test_reset_fault_is_a_valid_v1_control_message() -> None:
    message = ClientControlMessage.model_validate(
        {"v": 1, "type": "reset_fault", "request_id": "reset-1"}
    )
    assert message.type == "reset_fault"

    schema = load_protocol_schema()
    validator = Draft202012Validator({**schema, "$ref": "#/$defs/ClientControlMessage"})
    assert not list(validator.iter_errors(message.model_dump(mode="json")))


def test_home_request_is_a_valid_v1_control_message() -> None:
    message = ClientControlMessage.model_validate(
        {"v": 1, "type": "home_request", "request_id": "home-1"}
    )
    assert message.type == "home_request"

    schema = load_protocol_schema()
    validator = Draft202012Validator({**schema, "$ref": "#/$defs/ClientControlMessage"})
    assert not list(validator.iter_errors(message.model_dump(mode="json")))


@pytest.mark.parametrize(
    "payload",
    [
        {
            "v": 1,
            "type": "home_result",
            "request_id": "home-1",
            "accepted": True,
            "mode": "DISARMED",
        },
        {
            "v": 1,
            "type": "home_result",
            "request_id": "home-2",
            "accepted": False,
            "reason": "home_failed",
            "message": "仿真无法返回初始姿态，请稍后重试。",
        },
    ],
)
def test_home_result_schema_accepts_exact_discriminated_variants(payload: dict) -> None:
    schema = load_protocol_schema()
    validator = Draft202012Validator({**schema, "$ref": "#/$defs/HomeResultMessage"})
    assert not list(validator.iter_errors(payload))


@pytest.mark.parametrize("accepted", [True, False])
def test_home_result_schema_rejects_extra_fields(accepted: bool) -> None:
    payload = (
        {
            "v": 1,
            "type": "home_result",
            "request_id": "home-1",
            "accepted": True,
            "mode": "DISARMED",
            "extra": True,
        }
        if accepted
        else {
            "v": 1,
            "type": "home_result",
            "request_id": "home-2",
            "accepted": False,
            "reason": "grip_pressed",
            "message": "请先松开手柄抓握键，再请求 Home。",
            "extra": True,
        }
    )
    schema = load_protocol_schema()
    validator = Draft202012Validator({**schema, "$ref": "#/$defs/HomeResultMessage"})
    assert list(validator.iter_errors(payload))


@pytest.mark.parametrize(
    "payload",
    [
        {
            "v": 1,
            "type": "fault_reset_result",
            "request_id": "reset-1",
            "accepted": True,
            "mode": "DISARMED",
        },
        {
            "v": 1,
            "type": "fault_reset_result",
            "request_id": "reset-2",
            "accepted": False,
            "reason": "unrecoverable_fault",
            "message": "该故障无法在线复位，请重启后端并重新检查。",
        },
    ],
)
def test_fault_reset_result_schema_accepts_exact_discriminated_variants(payload: dict) -> None:
    schema = load_protocol_schema()
    validator = Draft202012Validator({**schema, "$ref": "#/$defs/FaultResetResultMessage"})
    assert not list(validator.iter_errors(payload))


@pytest.mark.parametrize(
    "payload",
    [
        {
            "v": 1,
            "type": "fault_reset_result",
            "request_id": "reset-1",
            "accepted": True,
            "mode": "DISARMED",
            "extra": True,
        },
        {
            "v": 1,
            "type": "fault_reset_result",
            "request_id": "reset-2",
            "accepted": False,
            "reason": "unrecoverable_fault",
            "message": "该故障无法在线复位，请重启后端并重新检查。",
            "extra": True,
        },
    ],
)
def test_fault_reset_result_schema_rejects_extra_fields(payload: dict) -> None:
    schema = load_protocol_schema()
    validator = Draft202012Validator({**schema, "$ref": "#/$defs/FaultResetResultMessage"})
    assert list(validator.iter_errors(payload))


def test_fault_reset_reject_reasons_match_python_and_json_schema() -> None:
    expected = {
        "no_fault",
        "stop_incomplete",
        "backend_moving",
        "unrecoverable_fault",
        "control_loop_unavailable",
    }
    schema = load_protocol_schema()
    assert hasattr(messages, "FaultResetRejectReason")
    assert set(get_args(messages.FaultResetRejectReason)) == expected
    assert set(schema["$defs"]["FaultResetRejected"]["properties"]["reason"]["enum"]) == expected
