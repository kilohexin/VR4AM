from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "default.yaml"


@dataclass(frozen=True)
class Settings:
    backend: str

    @classmethod
    def load(cls) -> "Settings":
        path = Path(os.environ.get("VR4ARM_CONFIG", DEFAULT_CONFIG))
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        backend = payload.get("backend") if isinstance(payload, dict) else None
        if backend != "simulator":
            raise RuntimeError("real_robot_disabled")
        return cls(backend=backend)
