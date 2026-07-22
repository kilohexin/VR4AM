from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI

from app.api.health import router as health_router
from app.api.teleop_ws import router as teleop_router
from app.config import Settings
from app.control.coordinate_mapper import CoordinateMapper
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.safety import SafetyLimiter
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
            mapper=CoordinateMapper(
                translation_scale=settings.translation_scale,
                rotation_scale=settings.rotation_scale,
            ),
            limiter=SafetyLimiter(
                max_linear_speed=settings.max_linear_speed_mps,
                max_angular_speed=settings.max_angular_speed_radps,
                max_linear_accel=settings.max_linear_accel_mps2,
                max_angular_accel=settings.max_angular_accel_radps2,
                envelope=settings.workspace_radius_m,
            ),
            constraint_clear_ms=settings.constraint_clear_ms,
        )
        app.state.settings = settings
        app.state.backend = backend
        app.state.latest = latest
        app.state.control = control
        app.state.recorder = control.recorder
        app.state.teleop_sender_tasks = set()
        app.state.teleop_owner = None
        app.state.teleop_owner_lock = asyncio.Lock()
        backend_connect_started = False
        backend_connected = False
        primary_error: BaseException | None = None
        cleanup_error: BaseException | None = None
        try:
            backend_connect_started = True
            await backend.connect()
            backend_connected = True
            await control.connect()
            await control.start()
            yield
        except BaseException as error:
            primary_error = error
            raise
        finally:
            if backend_connected:
                try:
                    await control.stop()
                except BaseException as error:
                    cleanup_error = error
            if backend_connect_started:
                try:
                    await backend.disconnect()
                except BaseException as error:
                    if cleanup_error is None:
                        cleanup_error = error
            if primary_error is None and cleanup_error is not None:
                raise cleanup_error

    app = FastAPI(lifespan=lifespan)
    app.include_router(health_router)
    app.include_router(teleop_router)
    return app


app = create_app()
