"""Fixed read-only FK/IK survey. Never constructs an adapter or sends a target.

PAUSED can be observed without resuming. Returned data are diagnostic evidence,
not stopping, collision, interpolation or motion authorization. Native resources
require standalone process exit; local query cancellation proves no remote exit.
"""
from __future__ import annotations

import asyncio
import copy
from dataclasses import asdict
from datetime import UTC, datetime
import json
import math
from time import monotonic_ns

import numpy as np

from app.commissioning.finite_diagnostic import _angle
from app.robots.lebai_codec import joint_vector, pose_from_lebai, pose_to_lebai
from app.robots.lebai_finite_pvat import build_finite_pvat_candidate, _controls
from app.robots.lebai_pvat import PvatLimits
from app.schemas.messages import Pose

READ_METHODS = ('is_connected', 'get_robot_state', 'get_estop_reason', 'get_running_motion',
                'get_motion_state', 'get_kin_data', 'get_tcp', 'kinematics_inverse', 'kinematics_forward')
READ_TIMEOUT_S = 1.
CONNECT_TIMEOUT_S = 5.
TOTAL_TIMEOUT_S = 15.
MAX_CALLS = 24  # One connect, two observations (at most nine each), IK and four FK.
DISTANCE_M = .0002
POSITION_TOLERANCE_M = .0001
ROTATION_TOLERANCE_DEG = .1


class SurveyError(ValueError):
    """Only locally defined reason strings, never vendor exception messages."""


def new_report(*, simulation):
    return dict(schema=1, started_utc=datetime.now(UTC).isoformat(), complete=False,
                runtime='SIMULATION' if simulation else 'LIVE_READONLY',
                motion_authorized=False, hardware_stop_verified=False,
                kinematics_consistent=False, connection_factory_entered=False,
                native_sdk_init_entered=False, native_sdk_connect_entered=False,
                connection_release='standalone_process_exit_required_not_verified',
                pending_remote_result_unknown=False, calls=[], fk_samples=[],
                assessment='reference_samples_only_not_path_or_physics',
                chain_within_vr_lease=False)


class _Queries:
    def __init__(self, admit, report):
        self.admit = admit
        self.report = report
        self.deadline = monotonic_ns() + int(TOTAL_TIMEOUT_S * 1e9)
        self.pending = None
        self.closed = False

    def guard(self):
        self.admit()
        if self.closed or monotonic_ns() >= self.deadline:
            raise SurveyError('survey_budget_expired')

    async def call(self, name, invoke):
        self.guard()
        if name not in ('connect', *READ_METHODS) or len(self.report['calls']) >= MAX_CALLS:
            raise SurveyError('survey_call_budget_or_allowlist')
        if self.pending is not None and not self.pending.done():
            raise SurveyError('survey_read_pending')
        item = dict(method=name, started_ns=monotonic_ns(), entered=False,
                    outcome='not_started', remote_result_unknown=False)
        self.report['calls'].append(item)
        timeout = CONNECT_TIMEOUT_S if name == 'connect' else READ_TIMEOUT_S
        deadline = min(self.deadline, item['started_ns'] + int(timeout * 1e9))
        if name == 'connect':
            self.report['connect_deadline_ns'] = deadline

        async def query():
            self.guard()  # Recheck in the scheduled task, before SDK method entry.
            if monotonic_ns() >= deadline:
                raise SurveyError('survey_query_expired_before_entry')
            item['entered'] = True
            item['factory_await_started_ns' if name == 'connect' else 'sdk_await_started_ns'] = monotonic_ns()
            if name == 'connect':
                self.report['connection_factory_entered'] = True
            raw = await invoke()  # Supports coroutine and Future; no generic RPC input.
            return raw, monotonic_ns()

        task = self.pending = asyncio.create_task(query(), name='readonly-kinematics-' + name)

        def finished(done):
            if not done.cancelled():
                done.exception()

        task.add_done_callback(finished)
        try:
            done, _ = await asyncio.wait({task}, timeout=max(0., (deadline - monotonic_ns()) / 1e9))
            if not done:
                task.cancel()
                item.update(outcome='timeout', remote_result_unknown=item['entered'])
                raise SurveyError('survey_query_timeout')
            raw, completed = task.result()
            item['outcome'] = 'returned'
            item['factory_await_completed_ns' if name == 'connect' else 'sdk_await_completed_ns'] = completed
            if completed >= deadline or monotonic_ns() >= deadline:
                raise SurveyError('survey_late_return')
            self.guard()
            if monotonic_ns() >= deadline:
                raise SurveyError('survey_late_return')
            if name == 'connect':
                return raw
            # Freeze nested SDK values immediately. Unknown non-JSON/nonfinite values fail closed.
            frozen = json.loads(json.dumps(raw, allow_nan=False))
            item['raw'] = frozen
            return copy.deepcopy(frozen)
        except asyncio.CancelledError:
            item.update(outcome='cancelled', remote_result_unknown=item['entered'])
            if not task.done() and not task.cancelling():
                task.cancel()
            raise
        except SurveyError:
            raise
        except Exception:
            item.update(outcome='error' if item['entered'] else 'not_started')
            raise
        finally:
            item['completed_ns'] = monotonic_ns()


def _state(raw):
    value = raw.strip().upper() if isinstance(raw, str) else raw
    if type(value) is int and value in (5, 6):
        return 'IDLE' if value == 5 else 'PAUSED'
    if type(value) is str and value in ('IDLE', 'PAUSED'):
        return value
    raise SurveyError('survey_state_not_idle_or_paused')


def _pose_error(a, b):
    return dict(position_m=math.dist(a.p, b.p), rotation_deg=_angle(a.q, b.q))


def _agree(a, b):
    error = _pose_error(a, b)
    return error['position_m'] <= POSITION_TOLERANCE_M and error['rotation_deg'] <= ROTATION_TOLERANCE_DEG


async def _observe(client, calls, settings):
    async def read(name, *args):
        return await calls.call(name, lambda: getattr(client, name)(*args))

    if await read('is_connected') is not True:
        raise SurveyError('survey_not_connected')
    state = _state(await read('get_robot_state'))
    estop = await read('get_estop_reason')
    if type(estop) is not int or estop != 0:
        raise SurveyError('survey_estop_or_unknown')
    running = await read('get_running_motion')
    if running is not None:
        if type(running) is not int or running < 0:
            raise SurveyError('survey_invalid_running_motion')
        if running != 0:
            motion_state = await read('get_motion_state', running)
            if type(motion_state) is not str or motion_state.strip().upper() != 'FINISHED':
                raise SurveyError('survey_running_motion_not_finished')
    raw = await read('get_kin_data')
    if not isinstance(raw, dict):
        raise SurveyError('survey_invalid_kinematics')
    q = joint_vector(raw.get('actual_joint_pose'), 'actual_joint_pose')
    qd = joint_vector(raw.get('actual_joint_speed'), 'actual_joint_speed')
    qdd = joint_vector(raw.get('actual_joint_acc'), 'actual_joint_acc')
    tcp = pose_from_lebai(raw.get('actual_tcp_pose'))
    setting = pose_from_lebai(await read('get_tcp'))
    if any(v != 0. for v in qd):
        raise SurveyError('survey_reported_speed_not_stationary')
    expected = pose_from_lebai(asdict(settings.expected_tcp))
    error = _pose_error(setting, expected)
    if (error['position_m'] > settings.tcp_position_tolerance_m
            or error['rotation_deg'] > settings.tcp_rotation_tolerance_deg):
        raise SurveyError('survey_tcp_setting_mismatch')
    if _state(await read('get_robot_state')) != state:
        raise SurveyError('survey_state_changed_during_observation')
    return dict(state=state, q=list(q), qd=list(qd), qdd=list(qdd),
                tcp=tcp.model_dump(mode='json'), tcp_setting=setting.model_dump(mode='json'))


async def collect(settings, factory, admit, report):
    if settings.mode != 'readonly':
        raise ValueError('kinematics_survey_requires_readonly')
    calls = _Queries(admit, report)
    try:
        client = await calls.call('connect', lambda: factory(settings.ip))
        if any(not callable(getattr(client, name, None)) for name in READ_METHODS):
            raise SurveyError('survey_capability_missing')
        initial = report['initial'] = await _observe(client, calls, settings)
        origin = Pose.model_validate(initial['tcp'])
        position = list(origin.p)
        position[2] += DISTANCE_M
        target = Pose(p=tuple(position), q=origin.q)
        report['target'] = target.model_dump(mode='json')
        chain_started = monotonic_ns()
        report['chain_started_ns'] = chain_started
        solution = joint_vector(await calls.call('kinematics_inverse',
            lambda: client.kinematics_inverse(pose_to_lebai(target), list(initial['q']))), 'survey_ik_solution')
        if max(abs(q - a) for q, a in zip(solution, initial['q'])) > settings.control.max_joint_step_rad:
            raise SurveyError('survey_ik_joint_step_limit')
        if min(abs(solution[2]), abs(solution[4])) <= math.radians(5):
            raise SurveyError('survey_ik_singular_configuration')
        margin = settings.joint_limit_margin_rad
        limits = PvatLimits(settings.control.pvat_horizon_s, settings.control.max_joint_speed_radps,
                           settings.control.max_joint_acceleration_radps2,
                           settings.control.max_joint_tracking_error_rad,
                           tuple(q + margin for q in settings.soft_joint_min_rad),
                           tuple(q - margin for q in settings.soft_joint_max_rad))
        candidate = build_finite_pvat_candidate(solution, initial['q'], initial['qd'], initial['qdd'], limits)
        point = candidate.point
        report['candidate'] = json.loads(json.dumps(asdict(point)))
        report['progress_fraction'] = candidate.progress_fraction
        base, slope = _controls(np.array(initial['qd']), np.array(initial['qdd']),
                                np.array(point.q) - np.array(initial['q']), point.horizon_s)
        middle_q = np.array(initial['q']) + np.array([math.comb(5, i) * .5 ** 5 for i in range(6)]) @ (base + slope)
        fk_origin = None
        for role, q in (('feedback', initial['q']), ('goal', solution), ('midpoint', middle_q), ('endpoint', point.q)):
            raw = await calls.call('kinematics_forward', lambda q=q: client.kinematics_forward([float(v) for v in q]))
            pose = pose_from_lebai(raw)
            row = dict(role=role, joints=[float(v) for v in q], pose=pose.model_dump(mode='json'))
            report['fk_samples'].append(row)
            if role == 'feedback':
                fk_origin = pose
                row['feedback_error'] = _pose_error(pose, origin)
                if not _agree(pose, origin):
                    raise SurveyError('survey_fk_feedback_mismatch')
            else:
                delta = np.array(pose.p) - np.array(fk_origin.p)
                progress = float(delta[2])
                cross = float(np.linalg.norm(delta[:2]))
                row.update(progress_m=progress, cross_m=cross)
                if (progress < -1e-9 or progress > DISTANCE_M + 1e-6
                        or cross > max(1e-6, max(0., progress) * math.tan(math.radians(5)))
                        or _angle(pose.q, fk_origin.q) > ROTATION_TOLERANCE_DEG):
                    raise SurveyError('survey_fk_direction_mismatch')
                if role == 'goal' and (not _agree(pose, target) or abs(progress - DISTANCE_M) > max(1e-6, DISTANCE_M * .05)):
                    raise SurveyError('survey_fk_goal_mismatch')
                if role == 'endpoint' and progress <= 1e-9:
                    raise SurveyError('survey_fk_endpoint_no_progress')
        report['chain_elapsed_ms'] = (monotonic_ns() - chain_started) / 1e6
        report['chain_within_vr_lease'] = 0 <= report['chain_elapsed_ms'] < 100
        report['chain_timing_scope'] = 'IK, construction, four FK and readonly admission checks; not controller execution'
        final = report['final'] = await _observe(client, calls, settings)
        if (initial['state'] != final['state']
                or max(abs(a - b) for a, b in zip(initial['q'], final['q'])) > .001
                or not _agree(Pose.model_validate(initial['tcp']), Pose.model_validate(final['tcp']))
                or _pose_error(Pose.model_validate(initial['tcp_setting']), Pose.model_validate(final['tcp_setting']))['position_m'] > 1e-9
                or _angle(initial['tcp_setting']['q'], final['tcp_setting']['q']) > 1e-6):
            raise SurveyError('survey_observation_changed')
        calls.guard()
        report.update(complete=True, kinematics_consistent=True)
    except asyncio.CancelledError:
        report.update(error='interrupted', error_type='CancelledError')
        raise
    except Exception as error:
        report.update(error=str(error) if isinstance(error, SurveyError) else 'survey_query_or_data_rejected',
                      error_type=type(error).__name__)
    finally:
        calls.closed = True
        pending = calls.pending
        if pending is not None and not pending.done() and not pending.cancelling():
            pending.cancel()
        report['pending_remote_result_unknown'] = any(x['remote_result_unknown'] for x in report['calls'])
        # The task owns its lifecycle locally; late data cannot rewrite saved or returned evidence.
        report['calls'] = copy.deepcopy(report['calls'])
        report['completed_utc'] = datetime.now(UTC).isoformat()
    return report
