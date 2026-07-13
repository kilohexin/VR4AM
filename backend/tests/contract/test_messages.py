import json
from pathlib import Path

import pytest
from pydantic import ValidationError

from app.schemas.messages import RobotStateMessage, VRFrame

ROOT = Path(__file__).resolve().parents[3]


def load_fixture(name: str) -> dict:
    return json.loads((ROOT / "schemas" / "fixtures" / name).read_text(encoding="utf-8"))


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
