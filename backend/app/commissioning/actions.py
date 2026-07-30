from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from app.schemas.messages import RobotStateMessage

TranslationAxis = Literal["x", "y", "z"]
RotationAxis = Literal["roll", "pitch", "yaw"]
GripperTarget = Literal["open", "close"]


@dataclass(frozen=True)
class TranslationAction:
    axis: TranslationAxis
    distance_m: float


@dataclass(frozen=True)
class RotationAction:
    axis: RotationAxis
    angle_deg: float


@dataclass(frozen=True)
class GripperAction:
    target: GripperTarget


@dataclass(frozen=True)
class HomeAction:
    pass


@dataclass(frozen=True)
class StopAction:
    pass


SmokeAction = (
    TranslationAction | RotationAction | GripperAction | HomeAction | StopAction
)


@dataclass(frozen=True)
class SmokeOptions:
    config_path: Path
    action: SmokeAction
    confirmation: str


@dataclass(frozen=True)
class SmokeResult:
    action: str
    before: RobotStateMessage
    after: RobotStateMessage
    stable: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "action": self.action,
            "before": self.before.model_dump(mode="json"),
            "after": self.after.model_dump(mode="json"),
            "stable": self.stable,
        }
