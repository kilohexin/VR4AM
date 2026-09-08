import json
from collections import deque
from unittest.mock import AsyncMock

import pytest

from app.commissioning.ik_probe import run_ik_probe, probe_targets
from app.schemas.messages import Pose
from tests.config.test_real_robot_config import REAL_CONFIG_TEMPLATE
from tests.robots.fake_lebai import FakeLebaiClient, IDLE_Q


def config(tmp_path, mode="readonly"):
    path = tmp_path / "robot.yaml"
    path.write_text(REAL_CONFIG_TEMPLATE.format(mode=mode), encoding="utf-8")
    return path


def test_probe_targets_include_positive_z_without_changing_orientation():
    pose = Pose(p=(.3, .1, .4), q=(0., 0., 0., 1.))
    targets = dict(probe_targets(pose))
    assert len(targets) == 13
    assert targets["+z"].p == pytest.approx((.3, .1, .402))
    assert targets["-z"].p == pytest.approx((.3, .1, .398))
    assert targets["+z"].q == pose.q


@pytest.mark.asyncio
async def test_probe_collects_two_seeds_and_never_writes_robot(tmp_path):
    client = FakeLebaiClient.idle()
    client.ik_results = deque([list(IDLE_Q)] * 26)
    output = tmp_path / "result.json"
    code = await run_ik_probe(config(tmp_path), output, AsyncMock(return_value=client))
    report = json.loads(output.read_text())
    assert code == 0 and report["complete"]
    assert report["motion_authorized"] is False
    assert len(report["results"]) == 26
    assert len(client.ik_calls) == 26
    assert client.write_calls == []
    assert {r["seed_name"] for r in report["results"]} == {"actual", "configured_ready"}


@pytest.mark.asyncio
async def test_control_config_is_rejected_before_connection(tmp_path):
    factory = AsyncMock()
    with pytest.raises(RuntimeError, match="ik_probe_requires_readonly"):
        await run_ik_probe(config(tmp_path, "control"), tmp_path / "r.json", factory)
    factory.assert_not_awaited()


@pytest.mark.asyncio
@pytest.mark.parametrize("condition", ["moving", "tcp_mismatch", "speed"])
async def test_probe_refuses_unsafe_baseline_without_ik_or_writes(tmp_path, condition):
    client = FakeLebaiClient.idle()
    if condition == "moving":
        client.robot_state = "MOVING"
    elif condition == "tcp_mismatch":
        client.tcp["z"] = .25
    else:
        client.kin_data["actual_joint_speed"] = [.1] * 6
    output = tmp_path / "r.json"
    assert await run_ik_probe(config(tmp_path), output, AsyncMock(return_value=client)) == 2
    assert not json.loads(output.read_text())["complete"]
    assert not client.ik_calls and not client.write_calls


@pytest.mark.asyncio
async def test_no_solution_is_data_not_a_motion_permission(tmp_path):
    client = FakeLebaiClient.idle()
    client.ik_results = deque([None] * 26)
    output = tmp_path / "r.json"
    assert await run_ik_probe(config(tmp_path), output, AsyncMock(return_value=client)) == 0
    report = json.loads(output.read_text())
    assert all(r["status"] == "no_solution" for r in report["results"])
    assert report["motion_authorized"] is False
    assert not client.write_calls


@pytest.mark.asyncio
async def test_pose_change_during_probe_aborts_without_motion(tmp_path):
    client = FakeLebaiClient.idle()

    async def changing_ik(pose, seed):
        client.ik_calls.append((pose, seed))
        client.kin_data["actual_joint_pose"] = [.1, -1., 1., 0., 1.57, 0.]
        return list(IDLE_Q)

    client.kinematics_inverse = changing_ik
    output = tmp_path / "r.json"
    assert await run_ik_probe(config(tmp_path), output, AsyncMock(return_value=client)) == 2
    assert "pose_changed" in json.loads(output.read_text())["error"]
    assert len(client.ik_calls) == 1 and not client.write_calls


@pytest.mark.asyncio
@pytest.mark.parametrize("result", [[float("nan")] * 6, [0., 1.], "invalid"])
async def test_invalid_ik_result_is_saved_and_aborts(tmp_path, result):
    client = FakeLebaiClient.idle()
    client.ik_results = deque([result])
    output = tmp_path / "r.json"
    assert await run_ik_probe(config(tmp_path), output, AsyncMock(return_value=client)) == 2
    report = json.loads(output.read_text())
    assert report["results"][0]["status"] == "error"
    assert len(client.ik_calls) == 1 and not client.write_calls


@pytest.mark.asyncio
async def test_ik_timeout_saves_partial_report_without_retry(tmp_path):
    client = FakeLebaiClient.idle()
    client.block_ik = True
    output = tmp_path / "r.json"
    assert await run_ik_probe(config(tmp_path), output, AsyncMock(return_value=client)) == 2
    report = json.loads(output.read_text())
    assert "TimeoutError" in report["error"]
    assert len(report["results"]) == 1
    assert not client.write_calls
