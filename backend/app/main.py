from __future__ import annotations

import asyncio
import math
import platform
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager, suppress
from dataclasses import asdict
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
    *, target_strategy: str | None = None,
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
        "target_strategy": (
            'segmented_experimental' if settings.segmented_target is not None
            else 'transactional_experimental' if settings.transactional_target is not None
            else 'legacy'
        ),
    }
    if settings.transactional_target is not None:
        metadata['pursuit_settings'] = asdict(settings.transactional_target)
    if settings.segmented_target is not None:
        metadata['segment_settling'] = asdict(settings.segmented_target.settling)
        metadata['segment_hardware_verified'] = False
        metadata['prepare_on_arm'] = settings.segmented_target.prepare_on_arm
    if target_strategy is not None:
        metadata['target_strategy']=target_strategy
        metadata['finite_position_arrival_policy']='coarse_grasp'
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
    *,
    stop_diagnostic_sampling: bool = False,
    offline_unsent_expiry_hold: bool = False,
    offline_parallel_segment_fk: bool = False,
    offline_parallel_snapshot_reads: bool = False,
    offline_observer_priority: bool = False,
    offline_finite_position: bool = False,
    offline_position_model=None,
    offline_position_clock=None,
    real_finite_position: bool = False,
    real_position_authorize=None,
    real_position_model=None,
    real_position_vr: bool = False,
    real_position_pitch: bool = False,
    docker_simulation: bool = False,
) -> RobotBackend:
    if type(docker_simulation) is not bool:raise ValueError('invalid_docker_simulator_selection')
    if docker_simulation:
        if (real_finite_position or offline_finite_position or backend_label!='LEBAI_DOCKER'
                or settings.backend!='lebai' or settings.lebai is None
                or settings.segmented_target is not None or settings.transactional_target is not None
                or stop_diagnostic_sampling):raise ValueError('docker_simulator_explicit_selection_required')
        from app.robots.finite_position_docker import DockerFinitePositionAdapter
        return DockerFinitePositionAdapter(settings.lebai,
            **({'client_factory':client_factory} if client_factory is not connect_real_client else {}),
            event_callback=recorder.write_event)
    if type(real_position_pitch) is not bool or (real_position_pitch and not real_finite_position):
        raise ValueError('position_pitch_explicit_selection_required')
    if type(real_finite_position) is not bool:raise ValueError('invalid_real_finite_position_selection')
    if real_finite_position:
        if (offline_finite_position or backend_label!='LEBAI' or settings.backend!='lebai'
                or settings.lebai is None or settings.segmented_target is not None
                or settings.transactional_target is not None or stop_diagnostic_sampling
                or not callable(real_position_authorize) or real_position_model is None):
            raise ValueError('position_web_explicit_selection_required')
        from app.robots.finite_position_web import RealFinitePositionAdapter
        return RealFinitePositionAdapter(settings.lebai,client_factory=client_factory,
            position_model=real_position_model,authorize=real_position_authorize,
            allow_vr=real_position_vr,
            allow_pitch=real_position_pitch,
            event_callback=recorder.write_event)
    if type(offline_finite_position) is not bool:raise ValueError('invalid_finite_position_selection')
    if offline_finite_position:
        if (backend_label!='LEBAI_FAKE' or settings.backend!='lebai' or settings.lebai is None
                or settings.lebai.ip!='digital-twin' or settings.segmented_target is not None
                or settings.transactional_target is not None or stop_diagnostic_sampling
                or client_factory is connect_real_client):
            raise ValueError('position_offline_injection_required')
        from app.robots.finite_position_adapter import FinitePositionAdapter
        return FinitePositionAdapter(settings.lebai,client_factory=client_factory,
            backend_label=backend_label,position_model=offline_position_model,event_callback=recorder.write_event,
            **({'clock':offline_position_clock.now_ns} if offline_position_clock is not None else {}))
    if type(offline_observer_priority) is not bool:
        raise ValueError('invalid_offline_observer_priority')
    if offline_observer_priority and (backend_label!='LEBAI_FAKE'
            or settings.backend!='lebai' or settings.segmented_target is None
            or settings.lebai is None or settings.lebai.ip!='digital-twin'):
        raise ValueError('offline_observer_priority_offline_only')
    if type(offline_parallel_snapshot_reads) is not bool:
        raise ValueError('invalid_offline_parallel_snapshot_reads')
    if offline_parallel_snapshot_reads and (backend_label!='LEBAI_FAKE'
            or settings.backend!='lebai' or settings.segmented_target is None
            or settings.lebai is None or settings.lebai.ip!='digital-twin'):
        raise ValueError('offline_parallel_snapshot_reads_offline_only')
    if type(offline_parallel_segment_fk) is not bool:
        raise ValueError('invalid_offline_parallel_segment_fk')
    if offline_parallel_segment_fk and (backend_label!='LEBAI_FAKE'
            or settings.backend!='lebai' or settings.segmented_target is None
            or settings.lebai is None or settings.lebai.ip!='digital-twin'):
        raise ValueError('offline_parallel_segment_fk_offline_only')
    if type(offline_unsent_expiry_hold) is not bool:
        raise ValueError('invalid_unsent_expiry_hold')
    if offline_unsent_expiry_hold and (backend_label!='LEBAI_FAKE' or settings.segmented_target is None):
        raise ValueError('unsent_expiry_hold_offline_only')
    if settings.backend == "simulator":
        if stop_diagnostic_sampling:
            raise ValueError("stop_rpc_diagnostics_requires_lebai")
        return SimRobotAdapter()
    if settings.lebai is None:
        raise RuntimeError("missing_lebai_settings")
    if backend_label not in {"LEBAI", "LEBAI_FAKE"}:
        raise RuntimeError("invalid_lebai_backend_label")
    if settings.segmented_target is not None:
        if settings.transactional_target is not None or stop_diagnostic_sampling:
            raise ValueError('conflicting_segmented_backend')
        from app.robots.segmented_adapter import SegmentedLebaiAdapter
        return SegmentedLebaiAdapter(settings.lebai,
            allow_tool_spin=settings.segmented_target.tool_spin,
            allow_prepare=settings.segmented_target.prepare_on_arm,
            allow_unsent_expiry_hold=offline_unsent_expiry_hold,
            parallel_fk=offline_parallel_segment_fk,
            offline_parallel_snapshot_reads=offline_parallel_snapshot_reads,
            observer_wait_s=.020 if offline_observer_priority else 0.,
            settling=settings.segmented_target.settling, client_factory=client_factory,
            event_callback=recorder.write_event, backend_label=backend_label)
    return RealLebaiAdapter(
        settings.lebai,
        client_factory=client_factory,
        event_callback=recorder.write_event,
        backend_label=backend_label,
        stop_diagnostic_sampling=stop_diagnostic_sampling,
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
        # Keep rate/step and downstream IK checks, but do not add a
        # Grip-anchor workspace envelope to the real teleoperation path.
        real_robot = runtime_backend == "LEBAI"
        return SafetyLimiter(
            max_linear_speed=control.max_tcp_speed_mps,
            max_angular_speed=control.max_tcp_rotation_radps,
            max_linear_accel=control.max_tcp_acceleration_mps2,
            max_angular_accel=control.max_tcp_angular_acceleration_radps2,
            workspace_radius=math.inf if real_robot else 0.45,
            workspace_half_extent_m=(
                None if real_robot else control.max_relative_translation_m
            ),
            max_rotation_from_anchor_rad=(
                None if real_robot
                else math.radians(control.max_relative_rotation_deg)
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
    offline_unsent_expiry_hold: bool = False,
    offline_parallel_segment_fk: bool = False,
    offline_parallel_snapshot_reads: bool = False,
    offline_observer_priority: bool = False,
    offline_finite_position: bool = False,
    offline_position_model=None,
    offline_position_clock=None,
    real_finite_position: bool = False,
    real_position_authorize=None,
    real_position_model=None,
    real_position_vr: bool = False,
    real_position_pitch: bool = False,
    docker_simulation: bool = False,
) -> FastAPI:
    runtime_settings = settings if settings is not None else Settings.load()
    runtime_backend = backend_label or (
        "SIMULATOR" if runtime_settings.backend == "simulator" else "LEBAI"
    )

    @asynccontextmanager
    async def lifespan(app: FastAPI) -> AsyncIterator[None]:
        diagnostics = DiagnosticsStore()
        recorder = DiagnosticsRecorder(
            build_recorder(runtime_settings, runtime_backend,
                **({'target_strategy':'finite_position_web'} if real_finite_position else {})),
            diagnostics,
        )
        backend = build_backend(
            runtime_settings,
            recorder,
            client_factory,
            runtime_backend,
            offline_unsent_expiry_hold=offline_unsent_expiry_hold,
            offline_parallel_segment_fk=offline_parallel_segment_fk,
            offline_parallel_snapshot_reads=offline_parallel_snapshot_reads,
            offline_observer_priority=offline_observer_priority,
            offline_finite_position=offline_finite_position,
            offline_position_model=offline_position_model,
            offline_position_clock=offline_position_clock,
            real_finite_position=real_finite_position,
            real_position_authorize=real_position_authorize,
            real_position_model=real_position_model,
            real_position_vr=real_position_vr,
            real_position_pitch=real_position_pitch,
            docker_simulation=docker_simulation,
        )
        latest = LatestVRFrame()
        if docker_simulation:
            from app.digital_twin.docker_runtime import DockerClock
            control_clock=DockerClock()
        else:control_clock=offline_position_clock if offline_finite_position and offline_position_clock is not None else MonotonicClock()
        preparation_enabled = bool(
            getattr(backend, 'supports_prepare_on_arm', False)
            and runtime_settings.lebai is not None
            and runtime_settings.lebai.mode == 'control'
            and runtime_settings.transactional_target is None
        )
        control = RobotControl(
            backend=backend,
            latest=latest,
            clock=control_clock,
            recorder=recorder,
            mapper=_build_mapper(runtime_settings),
            limiter=_build_limiter(runtime_settings, runtime_backend),
            transactional_settings=runtime_settings.transactional_target,
            segmented_settings=runtime_settings.segmented_target,
            prepare_on_arm=preparation_enabled,
            explicit_preparation=preparation_enabled,
            prepare_options=HomeOptions(runtime_settings.lebai.control.max_joint_speed_radps if docker_simulation else .05,
                60.,math.radians(.1),.02,.3) if real_finite_position or docker_simulation else None,
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
                runtime_backend == "LEBAI"
                and runtime_settings.lebai is not None
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
