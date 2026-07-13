from __future__ import annotations

from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI

from app.api.health import router as health_router
from app.api.teleop_ws import router as teleop_router
from app.config import Settings
from app.control.robot_control import LatestVRFrame, RobotControl
from app.recording.noop import NoopRecorder
from app.robots.sim_adapter import SimRobotAdapter
from app.timebase import MonotonicClock


def create_app() -> FastAPI:
    settings = Settings.load()

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        backend = SimRobotAdapter()
        latest = LatestVRFrame()
        control = RobotControl(
            backend=backend,
            latest=latest,
            clock=MonotonicClock(),
            recorder=NoopRecorder(),
        )
        app.state.settings = settings
        app.state.backend = backend
        app.state.latest = latest
        app.state.control = control
        app.state.recorder = control.recorder
        app.state.teleop_sender_tasks = set()
        await backend.connect()
        await control.connect()
        await control.start()
        try:
            yield
        finally:
            await control.stop()
            await backend.disconnect()

    app = FastAPI(lifespan=lifespan)
    app.include_router(health_router)
    app.include_router(teleop_router)
    return app


app = create_app()
