from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np
from scipy.spatial.transform import Rotation

from app.config import LebaiSettings
from app.schemas.messages import Pose
from app.sim.lm3_model import LM3Model


ProfileSource = Literal["simulation", "onsite_config"]
DifferenceSeverity = Literal["info", "warning"]


@dataclass(frozen=True)
class RobotProfileSnapshot:
    source: ProfileSource
    hardware_verified: bool
    home_q: tuple[float, ...]
    joint_min_rad: tuple[float, ...]
    joint_max_rad: tuple[float, ...]
    tcp: Pose
    gripper_closed_amplitude_is_lower: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "source": self.source,
            "hardware_verified": self.hardware_verified,
            "home_q": list(self.home_q),
            "joint_min_rad": list(self.joint_min_rad),
            "joint_max_rad": list(self.joint_max_rad),
            "tcp": self.tcp.model_dump(mode="json"),
            "gripper_closed_amplitude_is_lower": (
                self.gripper_closed_amplitude_is_lower
            ),
        }


@dataclass(frozen=True)
class ProfileDifference:
    field: str
    simulation: object
    onsite: object
    severity: DifferenceSeverity

    def to_dict(self) -> dict[str, object]:
        return {
            "field": self.field,
            "simulation": self.simulation,
            "onsite": self.onsite,
            "severity": self.severity,
        }


@dataclass(frozen=True)
class ProfileComparison:
    onsite_configured: bool
    differences: tuple[ProfileDifference, ...]

    def to_dict(self) -> dict[str, object]:
        return {
            "onsite_configured": self.onsite_configured,
            "differences": [
                difference.to_dict() for difference in self.differences
            ],
        }


def _translation(values: tuple[float, ...]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(values, dtype=float)
    return transform


def _rotation(quaternion: tuple[float, ...]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    return transform


def simulation_profile(model: LM3Model | None = None) -> RobotProfileSnapshot:
    selected = model or LM3Model()
    tool_transform = (
        _translation(selected.tool_root_offset_m)
        @ _rotation(selected.tool_rotation_xyzw)
        @ _translation(selected.tcp_offset_m)
    )
    tool_rotation = Rotation.from_matrix(tool_transform[:3, :3]).as_quat()
    return RobotProfileSnapshot(
        source="simulation",
        hardware_verified=False,
        home_q=selected.home_q,
        joint_min_rad=tuple(
            value - selected.joint_window_rad
            for value in selected.home_q
        ),
        joint_max_rad=tuple(
            value + selected.joint_window_rad
            for value in selected.home_q
        ),
        tcp=Pose(
            p=tuple(float(value) for value in tool_transform[:3, 3]),
            q=tuple(float(value) for value in tool_rotation),
        ),
        gripper_closed_amplitude_is_lower=True,
    )


def onsite_profile(settings: LebaiSettings) -> RobotProfileSnapshot:
    tcp = settings.expected_tcp
    tcp_rotation = Rotation.from_euler(
        "ZYX",
        (tcp.rz, tcp.ry, tcp.rx),
    ).as_quat()
    return RobotProfileSnapshot(
        source="onsite_config",
        hardware_verified=False,
        home_q=settings.home_q,
        joint_min_rad=settings.soft_joint_min_rad,
        joint_max_rad=settings.soft_joint_max_rad,
        tcp=Pose(
            p=(tcp.x, tcp.y, tcp.z),
            q=tuple(float(value) for value in tcp_rotation),
        ),
        gripper_closed_amplitude_is_lower=(
            settings.gripper.closed_amplitude_percent
            < settings.gripper.open_amplitude_percent
        ),
    )


def compare_profiles(
    simulation: RobotProfileSnapshot,
    onsite: RobotProfileSnapshot | None,
) -> ProfileComparison:
    if onsite is None:
        return ProfileComparison(False, ())

    differences: list[ProfileDifference] = []
    for index, (sim_value, onsite_value) in enumerate(
        zip(simulation.home_q, onsite.home_q, strict=True)
    ):
        if abs(sim_value - onsite_value) > 0.01:
            differences.append(
                ProfileDifference(
                    f"home_q[{index}]",
                    sim_value,
                    onsite_value,
                    "warning",
                )
            )
    for field, sim_values, onsite_values in (
        (
            "joint_min_rad",
            simulation.joint_min_rad,
            onsite.joint_min_rad,
        ),
        (
            "joint_max_rad",
            simulation.joint_max_rad,
            onsite.joint_max_rad,
        ),
    ):
        for index, (sim_value, onsite_value) in enumerate(
            zip(sim_values, onsite_values, strict=True)
        ):
            if not np.isclose(sim_value, onsite_value, atol=1e-12, rtol=0.0):
                differences.append(
                    ProfileDifference(
                        f"{field}[{index}]",
                        sim_value,
                        onsite_value,
                        "info",
                    )
                )

    position_error = float(
        np.linalg.norm(
            np.asarray(simulation.tcp.p) - np.asarray(onsite.tcp.p)
        )
    )
    if position_error > 0.001:
        differences.append(
            ProfileDifference(
                "tcp.position",
                list(simulation.tcp.p),
                list(onsite.tcp.p),
                "warning",
            )
        )
    orientation_error = float(
        (
            Rotation.from_quat(simulation.tcp.q)
            * Rotation.from_quat(onsite.tcp.q).inv()
        ).magnitude()
    )
    if orientation_error > np.deg2rad(0.5):
        differences.append(
            ProfileDifference(
                "tcp.orientation",
                list(simulation.tcp.q),
                list(onsite.tcp.q),
                "warning",
            )
        )
    if (
        simulation.gripper_closed_amplitude_is_lower
        != onsite.gripper_closed_amplitude_is_lower
    ):
        differences.append(
            ProfileDifference(
                "gripper_direction",
                simulation.gripper_closed_amplitude_is_lower,
                onsite.gripper_closed_amplitude_is_lower,
                "warning",
            )
        )
    return ProfileComparison(True, tuple(differences))
