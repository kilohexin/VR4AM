import hashlib
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from app.schemas.messages import Pose
from app.sim.ik import IKError, solve_ik
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import MODEL_CONFIG, LM3Model


def test_ik_recovers_known_nearby_joint_pose() -> None:
    model = LM3Model()
    expected = np.asarray(model.home_q) + np.array([0.05, -0.03, 0.04, 0.02, -0.02, 0.03])
    result = solve_ik(forward_pose(expected, model), model.home_q, model)
    assert result.position_error_m <= 0.002
    assert result.orientation_error_rad <= np.deg2rad(1.0)


def test_ik_rejects_far_unreachable_target() -> None:
    model = LM3Model()
    target = forward_pose(model.home_q, model).model_copy(update={"p": (10.0, 10.0, 10.0)})
    with pytest.raises(IKError, match="ik_unreachable"):
        solve_ik(target, model.home_q, model)


def test_ik_is_deterministic() -> None:
    model = LM3Model()
    target = forward_pose(np.asarray(model.home_q) + 0.02, model)
    assert solve_ik(target, model.home_q, model).q == solve_ik(target, model.home_q, model).q


def test_ik_rejects_seed_outside_session_window() -> None:
    model = LM3Model()
    seed = np.asarray(model.home_q)
    seed[0] += 2 * np.pi
    with pytest.raises(IKError, match="joint_safety_window"):
        solve_ik(forward_pose(model.home_q, model), seed, model)


def test_ik_rejects_non_finite_seed_with_explicit_failure() -> None:
    model = LM3Model()
    seed = np.asarray(model.home_q)
    seed[0] = np.nan
    with pytest.raises(IKError, match="ik_singular"):
        solve_ik(forward_pose(model.home_q, model), seed, model)


@pytest.mark.parametrize(
    "target_update",
    [
        {"p": (np.nan, 0.0, 0.0)},
        {"q": (0.0, 0.0, 0.0, 0.0)},
        {"p": (np.finfo(float).max, 0.0, 0.0)},
    ],
    ids=["nan_position", "zero_quaternion", "huge_finite_position"],
)
def test_ik_normalizes_invalid_target_numerics_to_singular_error(
    target_update: dict[str, tuple[float, ...]],
) -> None:
    model = LM3Model()
    target = forward_pose(model.home_q, model).model_copy(update=target_update)
    with pytest.raises(IKError, match="^ik_singular$"):
        solve_ik(target, model.home_q, model)


@pytest.mark.parametrize(
    "fixture_path",
    [Path(__file__).parents[3] / "schemas" / "fixtures" / "sim-reachability-v2.json"],
)
def test_ik_solves_at_least_99_percent_of_reachability_fixture(fixture_path: Path) -> None:
    fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
    assert fixture["version"] == 2
    assert fixture["generator_seed"] == 42
    assert fixture["sample_count"] == 1000
    assert fixture["model_sha256"] == hashlib.sha256(
        MODEL_CONFIG.read_bytes()
    ).hexdigest()
    rows = fixture["rows"]
    model = LM3Model()
    successes = 0
    for row in rows:
        target = Pose.model_validate(row["target"])
        try:
            result = solve_ik(target, row["seed"], model)
        except IKError:
            continue
        q = np.asarray(result.q)
        assert np.all(np.isfinite(q))
        assert np.all(np.abs(q - np.asarray(model.home_q)) <= model.joint_window_rad)
        solved = forward_pose(q, model)
        position_error = float(np.linalg.norm(np.asarray(target.p) - np.asarray(solved.p)))
        orientation_error = float(
            np.linalg.norm(
                (
                    Rotation.from_quat(target.q) * Rotation.from_quat(solved.q).inv()
                ).as_rotvec()
            )
        )
        assert position_error == pytest.approx(result.position_error_m, abs=1e-12)
        assert orientation_error == pytest.approx(result.orientation_error_rad, abs=1e-12)
        assert position_error <= 0.002
        assert orientation_error <= np.deg2rad(1.0)
        successes += 1
    assert len(rows) == 1000
    assert successes >= 990
