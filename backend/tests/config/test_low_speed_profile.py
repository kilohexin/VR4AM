from __future__ import annotations

from pathlib import Path
from math import dist

import pytest
import yaml
from scipy.spatial.transform import Rotation

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
