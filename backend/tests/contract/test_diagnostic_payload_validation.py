"""Diagnostic payload acceptance stays strict without encoding every subtree."""
import json

import pytest
from pydantic import ValidationError

from app.schemas import messages
from app.schemas.messages import DiagnosticEvent


def event(payload):
    return DiagnosticEvent(event_id=1, server_mono_ns=1, kind="sample",
                           critical=False, payload=payload)


def test_nested_payload_encodes_once_without_replacing_the_payload(monkeypatch):
    payload = {"samples": [{"q": [0.0, 1.0, -1.0], "valid": True},
                           {"nested": (None, "状态", {"reason": "stop_unverified"})}]}
    calls = []
    encode = json.dumps

    def count_encoding(value, *args, **kwargs):
        calls.append(value)
        return encode(value, *args, **kwargs)

    monkeypatch.setattr(messages.json, "dumps", count_encoding)
    result = event(payload)

    assert result.payload == payload
    assert len(calls) == 1
    assert calls[0] == payload


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("wrap", [lambda x: x, lambda x: [x], lambda x: ({"inner": x},)])
def test_nested_nonfinite_values_still_rejected(value, wrap):
    with pytest.raises(ValidationError, match="only finite values"):
        event({"data": wrap(value)})


@pytest.mark.parametrize("value", [object(), b"bytes", {1, 2}, 1 + 2j])
def test_nested_unsupported_json_values_still_rejected(value):
    with pytest.raises(ValidationError, match="JSON safe"):
        event({"data": [{"unsupported": value}]})


@pytest.mark.parametrize("key", [float("nan"), float("inf"), (1, 2)])
def test_nested_invalid_json_keys_still_rejected(key):
    with pytest.raises(ValidationError, match="JSON safe"):
        event({"data": {key: "value"}})


def test_json_wire_payload_preserves_unicode_lists_and_null():
    result = event({"status": "停止未确认", "samples": (True, None, -1.25, [0, "ok"])})
    assert json.loads(result.model_dump_json())["payload"] == {
        "status": "停止未确认", "samples": [True, None, -1.25, [0, "ok"]],
    }
