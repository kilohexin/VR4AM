import json
from pathlib import Path
from typing import Any

import pytest
from jsonschema import Draft202012Validator
from pydantic import BaseModel, ValidationError

from app.schemas.messages import ClientControlMessage, RobotStateMessage, VRFrame

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
