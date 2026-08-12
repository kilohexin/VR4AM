from __future__ import annotations

import asyncio
import math
import platform
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
from pathlib import Path

from fastapi import FastAPI

from app.api.health import router as health_router
from app.api.teleop_ws import router as teleop_router
from app.config import Settings
from app.control.coordinate_mapper import CoordinateMapper
from app.control.robot_control import LatestVRFrame, RobotControl
from app.control.safety import SafetyLimiter
from app.diagnostics.recorder import DiagnosticsRecorder
from app.diagnostics.store import DiagnosticsStore
from app.recording.base import RecorderSink
from app.recording.commissioning import CommissioningRecorder
from app.recording.noop import NoopRecorder
from app.rehearsal.report import RehearsalReportStore
from app.robots.base import BackendPreflight, HomeOptions, RobotBackend
from app.robots.lebai_adapter import ClientFactory, RealLebaiAdapter
from app.robots.lebai_sdk_bridge import connect_real_client
from app.robots.sim_adapter import SimRobotAdapter
from app.schemas.messages import RuntimeBackend
from app.timebase import MonotonicClock

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]


def build_recorder(
    settings: Settings,
    runtime_backend: RuntimeBackend | None = None,
) -> RecorderSink:
    if settings.backend == "simulator":
        return NoopRecorder()
    if settings.lebai is None:
        raise RuntimeError("missing_lebai_settings")
    runtime = runtime_backend or "LEBAI"
    metadata: dict[str, object] = {
        "backend": runtime,
        "real_robot_mode": settings.lebai.mode,
        "python_version": platform.python_version(),
        "platform": platform.platform(),
    }
    if runtime == "LEBAI_FAKE":
        metadata.update(
            {
                "runtime": "SIMULATION",
                "hardware_verified": False,
            }
        )
    return CommissioningRecorder(
        REPOSITORY_ROOT / "logs" / "commissioning",
        metadata=metadata,
    )


def build_backend(
    settings: Settings,
    recorder: RecorderSink,
    client_factory: ClientFactory = connect_real_client,
    backend_label: RuntimeBackend = "LEBAI",
) -> RobotBackend:
    if settings.backend == "simulator":
        return SimRobotAdapter()
    if settings.lebai is None:
        raise RuntimeError("missing_lebai_settings")
    if backend_label not in {"LEBAI", "LEBAI_FAKE"}:
        raise RuntimeError("invalid_lebai_backend_label")
    return RealLebaiAdapter(
        settings.lebai,
        client_factory=client_factory,
        event_callback=recorder.write_event,
        backend_label=backend_label,
    )


def _build_mapper(settings: Settings) -> CoordinateMapper:
    if settings.backend == "lebai":
        if settings.lebai is None:
            raise RuntimeError("missing_lebai_settings")
        return CoordinateMapper(
            translation_scale=settings.lebai.control.translation_scale,
            rotation_scale=1.0,
            rotation_dead_zone_deg=settings.rotation_dead_zone_deg,
        )
    return CoordinateMapper(
        translation_scale=settings.translation_scale,
        rotation_scale=settings.rotation_scale,
        rotation_dead_zone_deg=settings.rotation_dead_zone_deg,
    )


def _build_limiter(
    settings: Settings,
    runtime_backend: RuntimeBackend,
) -> SafetyLimiter:
    if settings.backend == "lebai":
        if settings.lebai is None:
            raise RuntimeError("missing_lebai_settings")
        control = settings.lebai.control
        return SafetyLimiter(
            max_linear_speed=control.max_tcp_speed_mps,
            max_angular_speed=control.max_tcp_rotation_radps,
            max_linear_accel=control.max_tcp_acceleration_mps2,
            max_angular_accel=control.max_tcp_angular_acceleration_radps2,
            workspace_half_extent_m=control.max_relative_translation_m,
            max_rotation_from_anchor_rad=math.radians(
                control.max_relative_rotation_deg
            ),
            max_linear_step_m=control.max_tcp_step_m,
            max_angular_step_rad=math.radians(
                control.max_tcp_rotation_step_deg
            ),
            workspace_boundary_mode=(
                "axis_clamp"
                if runtime_backend == "LEBAI_FAKE"
                else "hold"
            ),
        )
    return SafetyLimiter(
        max_linear_speed=settings.max_linear_speed_mps,
        max_angular_speed=settings.max_angular_speed_radps,
        max_linear_accel=settings.max_linear_accel_mps2,
        max_angular_accel=settings.max_angular_accel_radps2,
        workspace_radius=settings.workspace_radius_m,
    )


def _preflight_payload(preflight: BackendPreflight) -> dict[str, object]:
    return {
        "ready": preflight.ready,
        "reason": preflight.reason,
        "robot_state": preflight.robot_state.value,
        "tcp_matches": preflight.tcp_matches,
        "capabilities": list(preflight.capabilities),
    }


def create_app(
    *,
    settings: Settings | None = None,
    client_factory: ClientFactory = connect_real_client,
    backend_label: RuntimeBackend | None = None,
) -> FastAPI:
    runtime_settings = settings if settings is not None else Settings.load()
    runtime_backend = backend_label or (
        "SIMULATOR" if runtime_settings.backend == "simulator" else "LEBAI"
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        diagnostics = DiagnosticsStore()
        recorder = DiagnosticsRecorder(
            build_recorder(runtime_settings, runtime_backend),
            diagnostics,
        )
        backend = build_backend(
            runtime_settings,
            recorder,
            client_factory,
            runtime_backend,
        )
        latest = LatestVRFrame()
        control = RobotControl(
            backend=backend,
            latest=latest,
            clock=MonotonicClock(),
            recorder=recorder,
            mapper=_build_mapper(runtime_settings),
            limiter=_build_limiter(runtime_settings, runtime_backend),
            constraint_clear_ms=runtime_settings.constraint_clear_ms,
            home_options=HomeOptions(
                max_speed_radps=runtime_settings.home_joint_speed_radps,
                timeout_s=runtime_settings.home_timeout_s,
                position_tolerance_rad=(
                    runtime_settings.home_position_tolerance_rad
                ),
                velocity_tolerance_radps=(
                    runtime_settings.home_velocity_tolerance_radps
                ),
                stable_seconds=runtime_settings.home_stable_ms / 1000,
            ),
        )
        app.state.settings = runtime_settings
        app.state.backend = backend
        app.state.latest = latest
        app.state.control = control
        app.state.recorder = recorder
        app.state.diagnostics = diagnostics
        app.state.offline_rehearsal_store = RehearsalReportStore(
            root=(
                REPOSITORY_ROOT
                / "artifacts"
                / "acceptance"
                / "offline-rehearsal"
            ),
            runtime=runtime_backend,
        )
        app.state.runtime_backend = runtime_backend
        app.state.log_session_dir = recorder.log_session_dir
        app.state.teleop_sender_tasks = set()
        app.state.teleop_owner = None
        app.state.teleop_owner_lock = asyncio.Lock()
        app.state.backend_name = runtime_backend
        app.state.hardware_verified = False
        app.state.real_robot_mode = (
            None
            if runtime_settings.lebai is None
            else runtime_settings.lebai.mode
        )
        app.state.real_robot_enabled = False
        app.state.preflight_ready = None
        app.state.preflight_reason = None

        recorder_started = False
        backend_connect_started = False
        backend_connected = False
        primary_error: BaseException | None = None
        cleanup_error: BaseException | None = None
        startup_stage = "recorder_start"
        try:
            await recorder.start()
            recorder_started = True
            startup_stage = "backend_connect"
            backend_connect_started = True
            await backend.connect()
            backend_connected = True
            startup_stage = "backend_preflight"
            preflight = await backend.preflight()
            app.state.preflight_ready = preflight.ready
            app.state.preflight_reason = preflight.reason
            app.state.real_robot_enabled = bool(
                runtime_settings.lebai is not None
                and runtime_settings.lebai.mode == "control"
                and preflight.ready
            )
            await recorder.write_critical_event(
                "preflight_result",
                _preflight_payload(preflight),
                0,
            )
            startup_stage = "control_connect"
            await control.connect()
            startup_stage = "control_start"
            await control.start()
            startup_stage = "running"
            yield
        except BaseException as error:
            primary_error = error
            if recorder_started:
                with suppress(Exception):
                    await recorder.write_critical_event(
                        "startup_failure",
                        {"stage": startup_stage},
                        0,
                    )
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
            if recorder_started:
                try:
                    await recorder.close()
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
