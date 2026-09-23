from __future__ import annotations

from pathlib import Path
from math import dist
import json
import os
import subprocess
import sys

import pytest
import yaml
from scipy.spatial.transform import Rotation

from app.commissioning import low_speed_profile
from app.commissioning.low_speed_profile import create_low_speed_profile
from app.config import Settings
from app.digital_twin.runtime import load_digital_twin_settings
from app.main import _build_limiter
from app.schemas.messages import Pose


ROOT = Path(__file__).resolve().parents[3]


def _source(tmp_path: Path) -> Path:
    source = tmp_path / "onsite.yaml"
    source.write_bytes((ROOT / "config" / "fake-lebai.yaml").read_bytes())
    return source


def test_generates_readonly_profile_without_changing_source(tmp_path: Path) -> None:
    source = _source(tmp_path)
    before = source.read_bytes()
    output = tmp_path / "slow.yaml"

    create_low_speed_profile(source, output)

    assert source.read_bytes() == before
    payload = yaml.safe_load(output.read_text(encoding="utf-8"))
    original = yaml.safe_load(before)
    assert payload["real_robot"]["mode"] == "readonly"
    assert payload["real_robot"]["ip"] == original["real_robot"]["ip"]
    assert payload["real_robot"]["expected_tcp"] == original["real_robot"]["expected_tcp"]
    assert payload["real_robot"]["gripper"] == original["real_robot"]["gripper"]
    assert payload["real_robot"]["control"]["max_tcp_speed_mps"] == 0.005
    assert payload["real_robot"]["control"]["max_tcp_rotation_radps"] == 0.05
    assert payload["real_robot"]["control"]["max_tcp_acceleration_mps2"] == 0.02
    assert payload["real_robot"]["control"]["max_tcp_angular_acceleration_radps2"] == 0.1
    assert payload["real_robot"]["control"]["max_joint_speed_radps"] == 0.05
    assert payload["real_robot"]["control"]["max_joint_acceleration_radps2"] == 0.2
    assert payload["real_robot"]["control"]["max_tcp_step_m"] == 0.0005
    assert payload["real_robot"]["control"]["max_tcp_rotation_step_deg"] == 0.2
    assert payload["real_robot"]["control"]["max_relative_translation_m"] == 0.02
    assert payload["real_robot"]["control"]["max_relative_rotation_deg"] == 5
    assert payload["real_robot"]["control"]["translation_scale"] == 0.2
    assert Settings.load(output).lebai is not None


def test_profile_never_raises_existing_tighter_limits(tmp_path: Path) -> None:
    source = _source(tmp_path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    control = payload["real_robot"]["control"]
    control["max_tcp_speed_mps"] = 0.003
    control["max_joint_speed_radps"] = 0.03
    control["max_relative_translation_m"] = 0.01
    source.write_text(yaml.safe_dump(payload), encoding="utf-8")

    output = tmp_path / "slow.yaml"
    create_low_speed_profile(source, output)

    limited = yaml.safe_load(output.read_text(encoding="utf-8"))["real_robot"]["control"]
    assert limited["max_tcp_speed_mps"] == 0.003
    assert limited["max_joint_speed_radps"] == 0.03
    assert limited["max_relative_translation_m"] == 0.01


def test_rejects_control_source_and_does_not_create_output(tmp_path: Path) -> None:
    source = _source(tmp_path)
    source.write_text(
        source.read_text(encoding="utf-8").replace("mode: readonly", "mode: control"),
        encoding="utf-8",
    )
    output = tmp_path / "slow.yaml"

    with pytest.raises(ValueError, match="source_requires_readonly"):
        create_low_speed_profile(source, output)

    assert not output.exists()


def test_existing_output_is_never_overwritten(tmp_path: Path) -> None:
    source = _source(tmp_path)
    output = tmp_path / "slow.yaml"
    output.write_bytes(b"existing")

    with pytest.raises(FileExistsError):
        create_low_speed_profile(source, output)

    assert output.read_bytes() == b"existing"


def test_generated_profile_reaches_fake_runtime_motion_limiter(tmp_path: Path) -> None:
    source = _source(tmp_path)
    output = tmp_path / "slow.yaml"
    create_low_speed_profile(source, output)
    settings = load_digital_twin_settings(output)
    limiter = _build_limiter(settings, "LEBAI_FAKE")
    current = Pose(p=(0.0, 0.0, 0.0), q=(0.0, 0.0, 0.0, 1.0))
    requested = Pose(
        p=(0.02, 0.0, 0.0),
        q=tuple(Rotation.from_euler("z", 5.0, degrees=True).as_quat()),
    )

    for _ in range(50):
        next_pose = limiter.limit_motion(current, requested, 0.02)
        assert dist(current.p, next_pose.p) <= 0.0001 + 1e-12
        angular_step = (
            Rotation.from_quat(next_pose.q)
            * Rotation.from_quat(current.q).inv()
        ).magnitude()
        assert angular_step <= 0.001 + 1e-12
        current = next_pose

    assert 0 < current.p[0] <= 0.005 + 1e-12
    assert Rotation.from_quat(current.q).magnitude() <= 0.05 + 1e-12


def test_verifier_accepts_only_generated_readonly_limits(tmp_path: Path) -> None:
    source = _source(tmp_path)
    candidate = tmp_path / "slow.yaml"
    create_low_speed_profile(source, candidate)

    report = low_speed_profile.verify_low_speed_profile(source, candidate)

    assert report["mode"] == "readonly"
    assert report["limits"]["max_tcp_speed_mps"] == 0.005
    assert report["limits"]["max_tcp_rotation_radps"] == 0.05
    assert len(report["source_sha256"]) == 64
    assert len(report["candidate_sha256"]) == 64


def test_verifier_rejects_non_limit_config_changes(tmp_path: Path) -> None:
    source = _source(tmp_path)
    candidate = tmp_path / "slow.yaml"
    create_low_speed_profile(source, candidate)
    payload = yaml.safe_load(candidate.read_text(encoding="utf-8"))
    payload["real_robot"]["expected_tcp"]["z"] = 0.2
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="candidate_unexpected_change"):
        low_speed_profile.verify_low_speed_profile(source, candidate)


def test_verifier_rejects_speed_raised_back_to_source(tmp_path: Path) -> None:
    source = _source(tmp_path)
    candidate = tmp_path / "slow.yaml"
    create_low_speed_profile(source, candidate)
    payload = yaml.safe_load(candidate.read_text(encoding="utf-8"))
    payload["real_robot"]["control"]["max_tcp_speed_mps"] = 0.60
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="candidate_limit_mismatch"):
        low_speed_profile.verify_low_speed_profile(source, candidate)


def test_verifier_requires_explicit_control_mode_and_confirmation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _source(tmp_path)
    candidate = tmp_path / "slow.yaml"
    create_low_speed_profile(source, candidate)
    payload = yaml.safe_load(candidate.read_text(encoding="utf-8"))
    payload["real_robot"]["mode"] = "control"
    candidate.write_text(yaml.safe_dump(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="candidate_mode_mismatch"):
        low_speed_profile.verify_low_speed_profile(source, candidate)
    with pytest.raises(RuntimeError, match="real_robot_confirmation_required"):
        low_speed_profile.verify_low_speed_profile(
            source, candidate, expected_mode="control"
        )

    monkeypatch.setenv("VR4ARM_REAL_ROBOT_CONFIRM", "I_UNDERSTAND_REAL_ROBOT_MOTION")
    report = low_speed_profile.verify_low_speed_profile(
        source, candidate, expected_mode="control"
    )
    assert report["mode"] == "control"


def test_verifier_cli_reports_readonly_candidate_without_device_access(
    tmp_path: Path,
) -> None:
    source = _source(tmp_path)
    candidate = tmp_path / "slow.yaml"
    create_low_speed_profile(source, candidate)
    env = {**os.environ, "PYTHONPATH": str(ROOT / "backend")}

    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "verify_vr_low_speed_profile.py"),
            "--source", str(source),
            "--candidate", str(candidate),
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )

    assert completed.returncode == 0
    assert json.loads(completed.stdout)["mode"] == "readonly"
    assert completed.stderr == ""


def test_verifier_requires_separate_source_and_candidate(tmp_path: Path) -> None:
    source = _source(tmp_path)
    candidate = tmp_path / "slow.yaml"
    create_low_speed_profile(source, candidate)

    with pytest.raises(ValueError, match="candidate_must_differ_from_source"):
        low_speed_profile.verify_low_speed_profile(candidate, candidate)


def test_control_copy_needs_confirmation_and_preserves_both_inputs(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _source(tmp_path)
    readonly = tmp_path / "slow-readonly.yaml"
    create_low_speed_profile(source, readonly)
    source_before = source.read_bytes()
    readonly_before = readonly.read_bytes()
    control = tmp_path / "slow-control.yaml"
    monkeypatch.delenv("VR4ARM_REAL_ROBOT_CONFIRM", raising=False)

    with pytest.raises(RuntimeError, match="real_robot_confirmation_required"):
        low_speed_profile.create_low_speed_control_copy(source, readonly, control)
    assert not control.exists()

    monkeypatch.setenv("VR4ARM_REAL_ROBOT_CONFIRM", "I_UNDERSTAND_REAL_ROBOT_MOTION")
    low_speed_profile.create_low_speed_control_copy(source, readonly, control)

    assert source.read_bytes() == source_before
    assert readonly.read_bytes() == readonly_before
    assert yaml.safe_load(control.read_text(encoding="utf-8"))["real_robot"]["mode"] == "control"
    assert low_speed_profile.verify_low_speed_profile(
        source, control, expected_mode="control"
    )["mode"] == "control"


def test_control_copy_never_overwrites_existing_file(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = _source(tmp_path)
    readonly = tmp_path / "slow-readonly.yaml"
    create_low_speed_profile(source, readonly)
    control = tmp_path / "slow-control.yaml"
    control.write_bytes(b"existing")
    monkeypatch.setenv("VR4ARM_REAL_ROBOT_CONFIRM", "I_UNDERSTAND_REAL_ROBOT_MOTION")

    with pytest.raises(FileExistsError):
        low_speed_profile.create_low_speed_control_copy(source, readonly, control)

    assert control.read_bytes() == b"existing"


def test_control_copy_cli_creates_separate_verified_config(
    tmp_path: Path,
) -> None:
    source = _source(tmp_path)
    readonly = tmp_path / "slow-readonly.yaml"
    control = tmp_path / "slow-control.yaml"
    create_low_speed_profile(source, readonly)
    env = {
        **os.environ,
        "PYTHONPATH": str(ROOT / "backend"),
        "VR4ARM_REAL_ROBOT_CONFIRM": "I_UNDERSTAND_REAL_ROBOT_MOTION",
    }

    completed = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts" / "prepare_vr_low_speed_control.py"),
            "--source", str(source),
            "--readonly", str(readonly),
            "--output", str(control),
        ],
        cwd=ROOT,
        env=env,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )

    assert completed.returncode == 0
    assert json.loads(completed.stdout)["mode"] == "control"
    assert completed.stderr == ""
    assert control.exists()
