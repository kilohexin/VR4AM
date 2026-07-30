from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.robots.base import BackendCommandError
from app.robots.lebai_codec import (
    estop_fault,
    joint_vector,
    map_robot_state,
    pose_from_lebai,
    pose_to_lebai,
)
from app.schemas.messages import BackendState


def test_lebai_pose_round_trip_uses_intrinsic_zyx() -> None:
    source = {
        "x": 0.1,
        "y": -0.2,
        "z": 0.3,
        "rz": math.radians(30),
        "ry": math.radians(-20),
        "rx": math.radians(10),
    }
    pose = pose_from_lebai(source)
    expected = Rotation.from_euler(
        "ZYX",
        [source["rz"], source["ry"], source["rx"]],
    ).as_quat()

    np.testing.assert_allclose(pose.q, expected, atol=1e-9)
    recovered = pose_to_lebai(pose)
    for key, value in source.items():
        assert recovered[key] == pytest.approx(value, abs=1e-9)


@pytest.mark.parametrize(
    "payload",
    [
        {"x": 0, "y": 0, "z": 0},
        {"x": 0, "y": 0, "z": float("nan"), "rx": 0, "ry": 0, "rz": 0},
    ],
)
def test_lebai_pose_rejects_missing_and_non_finite_fields(payload) -> None:
    with pytest.raises(BackendCommandError, match="^invalid_sdk_pose$"):
        pose_from_lebai(payload)


def test_joint_vector_requires_six_finite_values() -> None:
    assert joint_vector([0, 1, 2, 3, 4, 5], "actual_joint_pose") == (
        0.0,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
    )
    with pytest.raises(
        BackendCommandError,
        match="^invalid_sdk_joint_vector:actual_joint_pose$",
    ):
        joint_vector([0, 1, 2, 3, 4, float("inf")], "actual_joint_pose")


@pytest.mark.parametrize(
    ("value", "expected"),
    [
        (-1, BackendState.FAULT),
        (0, BackendState.FAULT),
        (1, BackendState.FAULT),
        (2, BackendState.HOLD),
        (5, BackendState.IDLE),
        (7, BackendState.MOVING),
        (12, BackendState.HOLD),
        ("IDLE", BackendState.IDLE),
        ("MOVING", BackendState.MOVING),
        ("STOPPING", BackendState.HOLD),
        ("ESTOP", BackendState.FAULT),
    ],
)
def test_robot_state_mapping_matches_documented_codes(
    value: object,
    expected: BackendState,
) -> None:
    assert map_robot_state(value) is expected


def test_estop_reason_zero_is_clear_and_faults_are_public() -> None:
    assert estop_fault(0) is None
    assert estop_fault("None") is None
    assert estop_fault(4) == "estop:hard_estop"
    assert estop_fault(99) == "estop:unknown_99"
