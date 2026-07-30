from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from fastapi import FastAPI

from app.config import Settings
from app.digital_twin.lebai_client import DigitalTwinLebaiClient
from app.main import create_app
from app.robots.lebai_sdk_bridge import LebaiClientProtocol


DIGITAL_TWIN_CONFIG = (
    Path(__file__).resolve().parents[3] / "config" / "fake-lebai.yaml"
)


def load_digital_twin_settings(
    path: Path = DIGITAL_TWIN_CONFIG,
) -> Settings:
    settings = Settings.load(path)
    if settings.backend != "lebai" or settings.lebai is None:
        raise RuntimeError("digital_twin_requires_lebai_profile")
    if settings.lebai.mode != "readonly" or settings.lebai.ip != "digital-twin":
        raise RuntimeError("unsafe_digital_twin_source_profile")
    return replace(
        settings,
        lebai=replace(settings.lebai, mode="control"),
    )


def create_digital_twin_app(
    path: Path = DIGITAL_TWIN_CONFIG,
) -> FastAPI:
    settings = load_digital_twin_settings(path)
    assert settings.lebai is not None
    client = DigitalTwinLebaiClient.idle(settings.lebai)

    async def factory(_ip: str) -> LebaiClientProtocol:
        return client

    return create_app(
        settings=settings,
        client_factory=factory,
        backend_label="LEBAI_FAKE",
    )
