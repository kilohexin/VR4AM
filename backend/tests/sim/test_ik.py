import json
from pathlib import Path

import numpy as np
import pytest

from app.schemas.messages import Pose
from app.sim.ik import IKError, solve_ik
from app.sim.kinematics import forward_pose
from app.sim.lm3_model import LM3Model


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
    "fixture_path",
    [Path(__file__).parents[3] / "schemas" / "fixtures" / "sim-reachability-v1.json"],
)
def test_ik_solves_at_least_99_percent_of_reachability_fixture(fixture_path: Path) -> None:
    rows = json.loads(fixture_path.read_text(encoding="utf-8"))
    model = LM3Model()
    successes = 0
    for row in rows:
        try:
            solve_ik(Pose.model_validate(row["target"]), row["seed"], model)
        except IKError:
            continue
        successes += 1
    assert len(rows) == 1000
    assert successes >= 990
