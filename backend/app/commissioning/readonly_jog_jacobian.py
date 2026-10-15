"""Fixed six-joint FK survey; perturbations are query inputs, never targets.

One connection, 13 FK, zero IK or writes, at most 27 calls in 15 seconds.
Uses the existing bounded query owner and permanent unknown-result evidence.
No result grants motion, joint bounds, calibration or braking acceptance.
"""
import asyncio
import copy
from dataclasses import asdict
from datetime import UTC, datetime
import math

import numpy as np
from scipy.spatial.transform import Rotation

from app.commissioning import readonly_kinematics as base
from app.commissioning.native_jog_mdh import MdhTool, mdh_tcp_jacobian
from app.robots.lebai_codec import joint_vector, pose_from_lebai
from app.schemas.messages import Pose

READ_METHODS = ('is_connected', 'get_robot_state', 'get_estop_reason',
    'get_running_motion', 'get_kin_data', 'get_tcp', 'get_velocity_factor', 'kinematics_forward')
MAX_CALLS = 27
PERTURBATION_RAD = .001


class _JogQueries(base._Queries):
    _read_methods = READ_METHODS
    _call_limit = MAX_CALLS


def new_report(*, simulation):
    return dict(schema=1, started_utc=datetime.now(UTC).isoformat(),
        runtime='SIMULATION' if simulation else 'LIVE_READONLY',
        complete=False, feedback_fk_consistent=False, jacobian_collected=False,
        motion_authorized=False, hardware_stop_verified=False, hardware_verified=False,
        joint_bounds_verified=False, braking_verified=False, continuous_control_admitted=False,
        admission_error='native_jog_joint_bounds_unverified',
        connection_factory_entered=False, native_sdk_init_entered=False,
        native_sdk_connect_entered=False,
        connection_release='standalone_process_exit_required_not_verified',
        pending_remote_result_unknown=False, calls=[], fk_samples=[],
        assessment='local_derivatives_only_not_calibration_or_motion_admission',
        joint_perturbation_rad=PERTURBATION_RAD, event_clock='time.monotonic_ns')


async def _observe(client, calls, settings):
    async def read(name):
        return await calls.call(name, getattr(client, name))
    state = base._state(await read('get_robot_state'))
    if not base._explicit_clear_estop(await read('get_estop_reason')):
        raise base.SurveyError('jog_survey_estop_or_unknown')
    running = await read('get_running_motion')
    if type(running) is not int or running != 0:
        raise base.SurveyError('jog_survey_running_motion_not_zero')
    raw = await read('get_kin_data')
    if not isinstance(raw, dict):
        raise base.SurveyError('jog_survey_invalid_kinematics')
    q = joint_vector(raw.get('actual_joint_pose'), 'actual_joint_pose')
    qd = joint_vector(raw.get('actual_joint_speed'), 'actual_joint_speed')
    qdd = joint_vector(raw.get('actual_joint_acc'), 'actual_joint_acc')
    if any(v != 0 for v in qd) or max(map(abs, qdd)) > settings.control.max_joint_acceleration_radps2:
        raise base.SurveyError('jog_survey_reported_motion')
    actual = pose_from_lebai(raw.get('actual_tcp_pose'))
    tool = pose_from_lebai(await read('get_tcp'))
    expected = pose_from_lebai(asdict(settings.expected_tcp))
    error = base._pose_error(tool, expected)
    if (error['position_m'] > settings.tcp_position_tolerance_m
            or error['rotation_deg'] > settings.tcp_rotation_tolerance_deg):
        raise base.SurveyError('jog_survey_tcp_setting_mismatch')
    factor = await read('get_velocity_factor')
    if type(factor) is not int or not 0 <= factor <= 100:
        raise base.SurveyError('jog_survey_invalid_velocity_factor')
    return dict(state=state, q=list(q), qd=list(qd), qdd=list(qdd),
        tcp=actual.model_dump(mode='json'), tcp_setting=tool.model_dump(mode='json'), velocity_factor=factor)


def _query_points(initial, settings):
    origin = initial['q']
    points = [('reference', list(origin))]
    for axis in range(6):
        for sign in (-1, 1):
            q = list(origin)
            q[axis] += sign * PERTURBATION_RAD
            points.append((f'joint_{axis+1}_{"minus" if sign<0 else "plus"}', q))
    # Admit every mathematical input before issuing the first FK.
    margin = settings.joint_limit_margin_rad
    if any(not lo+margin <= v <= hi-margin for _,q in points
            for v,lo,hi in zip(q,settings.soft_joint_min_rad,settings.soft_joint_max_rad,strict=True)):
        raise base.SurveyError('jog_survey_query_outside_joint_limits')
    return points


def _finish_analysis(initial, samples, report):
    poses = [Pose.model_validate(row['pose']) for row in samples]
    jacobian = np.zeros((6, 6))
    for axis in range(6):
        minus, plus = poses[1+2*axis:3+2*axis]
        jacobian[:3,axis] = (np.array(plus.p)-np.array(minus.p))/(2*PERTURBATION_RAD)
        jacobian[3:,axis] = (Rotation.from_quat(plus.q)*Rotation.from_quat(minus.q).inv()).as_rotvec()/(2*PERTURBATION_RAD)
    setting = Pose.model_validate(initial['tcp_setting'])
    theoretical_pose, theoretical = mdh_tcp_jacobian(initial['q'], MdhTool(setting.p,setting.q))
    difference = jacobian-theoretical
    if not np.isfinite(jacobian).all() or not np.isfinite(difference).all():
        raise base.SurveyError('jog_survey_nonfinite_jacobian')
    report.update(controller_jacobian=jacobian.tolist(), theoretical_jacobian=theoretical.tolist(),
        controller_jacobian_rank=int(np.linalg.matrix_rank(jacobian)),
        max_linear_jacobian_difference_m_per_rad=float(np.max(np.abs(difference[:3]))),
        max_angular_jacobian_difference=float(np.max(np.abs(difference[3:]))),
        theoretical_reference_position_error_m=float(np.linalg.norm(theoretical_pose[:3,3]-poses[0].p)),
        model='manufacturer_theoretical_mdh_not_factory_calibration')


async def collect(settings, factory, admit, report):
    if settings.mode != 'readonly':
        raise ValueError('jog_survey_requires_readonly')
    calls = _JogQueries(admit,report)
    try:
        client = await calls.call('connect',lambda:factory(settings.ip))
        if any(not callable(getattr(client,name,None)) for name in READ_METHODS):
            raise base.SurveyError('jog_survey_capability_missing')
        if await calls.call('is_connected',client.is_connected) is not True:
            raise base.SurveyError('jog_survey_not_connected')
        initial = report['initial'] = await _observe(client,calls,settings)
        points = _query_points(initial,settings)
        for role,q in points:
            raw = await calls.call('kinematics_forward',lambda q=q:client.kinematics_forward(list(q)))
            pose = pose_from_lebai(raw)
            row = dict(role=role,joints=list(q),pose=pose.model_dump(mode='json'))
            report['fk_samples'].append(row)
            if role == 'reference':
                row['feedback_error'] = base._pose_error(pose,Pose.model_validate(initial['tcp']))
                if not base._agree(pose,Pose.model_validate(initial['tcp'])):
                    raise base.SurveyError('jog_survey_fk_feedback_mismatch')
                report['feedback_fk_consistent'] = True
        final = report['final'] = await _observe(client,calls,settings)
        tool_error = base._pose_error(Pose.model_validate(initial['tcp_setting']),Pose.model_validate(final['tcp_setting']))
        if (initial['state'] != final['state'] or initial['velocity_factor'] != final['velocity_factor']
                or max(abs(a-b) for a,b in zip(initial['q'],final['q'])) > .001
                or not base._agree(Pose.model_validate(initial['tcp']),Pose.model_validate(final['tcp']))
                or tool_error['position_m'] > 1e-9 or tool_error['rotation_deg'] > 1e-6):
            raise base.SurveyError('jog_survey_observation_changed')
        calls.guard()
        _finish_analysis(initial,report['fk_samples'],report)
        calls.guard()  # Analysis and hashing cannot renew the total budget.
        report.update(complete=True,jacobian_collected=True)
    except asyncio.CancelledError:
        report.update(error='interrupted',error_type='CancelledError')
        raise
    except Exception as error:
        report.update(error=str(error) if isinstance(error,base.SurveyError) else 'jog_survey_query_or_data_rejected',
            error_type=type(error).__name__)
    finally:
        calls.closed = True
        if calls.pending is not None and not calls.pending.done() and not calls.pending.cancelling():
            calls.pending.cancel()
        report['pending_remote_result_unknown'] = any(row['remote_result_unknown'] for row in report['calls'])
        report['calls'] = copy.deepcopy(report['calls'])
        report['completed_utc'] = datetime.now(UTC).isoformat()
    return report
