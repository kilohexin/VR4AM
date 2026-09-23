"""Single system-stop discrimination probe, not a production stop policy."""
import asyncio
import copy
import math
from dataclasses import asdict
from time import monotonic_ns

from scipy.spatial.transform import Rotation

from app.robots.lebai_codec import estop_fault, joint_vector, map_robot_state, pose_from_lebai
from app.schemas.messages import BackendState

CONFIRM = 'I_UNDERSTAND_SINGLE_SYSTEM_STOP_MAY_MOVE'


async def read_sample(client, clock):
    sample = {'started_ns': clock()}

    async def collect():
        for name, read in [('raw_state', client.get_robot_state),
                           ('kinematics', client.get_kin_data),
                           ('raw_estop', client.get_estop_reason),
                           ('tcp_setting', client.get_tcp),
                           ('raw_motion_id', client.get_running_motion)]:
            sample[name] = copy.deepcopy(await read())
        motion_id = sample['raw_motion_id']
        if isinstance(motion_id, bool) or (motion_id is not None and
                (not isinstance(motion_id, int) or motion_id < 0)):
            raise ValueError('invalid_motion_id')
        sample['motion_state'] = None
        if motion_id not in (None, 0):
            sample['motion_state'] = copy.deepcopy(await client.get_motion_state(motion_id))
        sample['raw_state_after'] = copy.deepcopy(await client.get_robot_state())

    await asyncio.wait_for(collect(), .3)
    sample['completed_ns'] = clock()
    if sample['completed_ns'] - sample['started_ns'] > 300_000_000:
        raise TimeoutError('late_snapshot')
    return sample


def values(sample):
    k = sample['kinematics']
    q = joint_vector(k['actual_joint_pose'], field='actual_joint_pose')
    qd = joint_vector(k['actual_joint_speed'], field='actual_joint_speed')
    return q, qd, pose_from_lebai(k['actual_tcp_pose'])


def validate_diagnostic_channels(sample):
    k = sample['kinematics']
    for field in ('actual_joint_pose', 'target_joint_pose', 'actual_joint_speed',
                  'target_joint_speed', 'actual_joint_torque', 'target_joint_torque'):
        joint_vector(k[field], field=field)
    pose_from_lebai(k['actual_tcp_pose'])
    pose_from_lebai(k['target_tcp_pose'])


def check_baseline(sample, settings, anchor=None):
    if any(map_robot_state(sample[key]) != BackendState.IDLE for key in
           ('raw_state', 'raw_state_after')):
        raise ValueError('baseline_not_idle')
    if estop_fault(sample['raw_estop']) is not None:
        raise ValueError('baseline_estop')
    if sample['raw_motion_id'] not in (None, 0) and sample['motion_state'] != 'FINISHED':
        raise ValueError('baseline_motion_unfinished')
    q, qd, tcp = values(sample)
    k = sample['kinematics']
    validate_diagnostic_channels(sample)
    target = joint_vector(k['target_joint_pose'], field='target_joint_pose')
    if max(abs(v) for v in qd + joint_vector(k['target_joint_speed'], field='target_joint_speed')) > .02:
        raise ValueError('baseline_speed')
    if max(abs(a-b) for a, b in zip(q, target)) > .001 or math.dist(
            tcp.p, pose_from_lebai(k['target_tcp_pose']).p) > .0005:
        raise ValueError('baseline_target_mismatch')
    for joints in (q, target):
        if not all(lo + settings.joint_limit_margin_rad <= v <= hi - settings.joint_limit_margin_rad
                   for v, lo, hi in zip(joints, settings.soft_joint_min_rad, settings.soft_joint_max_rad)):
            raise ValueError('baseline_joint_limits')
    if not all(lo <= v <= hi for v, lo, hi in zip(tcp.p, settings.startup_tcp_min_m, settings.startup_tcp_max_m)):
        raise ValueError('baseline_envelope')
    actual = pose_from_lebai(sample['tcp_setting'])
    expected = pose_from_lebai(asdict(settings.expected_tcp))
    angle = (Rotation.from_quat(actual.q) * Rotation.from_quat(expected.q).inv()).magnitude()
    if math.dist(actual.p, expected.p) > settings.tcp_position_tolerance_m or math.degrees(angle) > settings.tcp_rotation_tolerance_deg:
        raise ValueError('baseline_tcp_setting')
    if anchor:
        aq, _, ap = values(anchor)
        if max(abs(a-b) for a, b in zip(q, aq)) > .001 or math.dist(tcp.p, ap.p) > .0005:
            raise ValueError('baseline_drift')


async def run_probe(client, settings, *, confirm, emit, clock=monotonic_ns,
                    sleep=asyncio.sleep, before_write=lambda: None):
    if confirm != CONFIRM or settings.mode != 'control':
        raise ValueError('explicit_system_stop_confirmation_and_control_required')
    result = dict(complete=False, write_started=False, rpc_outcome='not_sent',
                  max_tcp_drift_m=0., max_joint_drift_rad=0., interrupted=False)
    task = None
    try:
        first = previous = None
        baseline_start = clock()
        while True:
            s = await read_sample(client, clock)
            emit({'kind': 'baseline_sample', 'sample': s})
            check_baseline(s, settings, first)
            if clock() - baseline_start > 2_000_000_000:
                raise TimeoutError('baseline_budget')
            if previous and s['completed_ns'] - previous['completed_ns'] > 250_000_000:
                raise ValueError('baseline_gap')
            first = first or s
            if s['completed_ns'] - first['completed_ns'] >= 300_000_000:
                break
            previous = s
            await sleep(.1)
        before_write()
        if clock() - s['completed_ns'] > 100_000_000:
            raise ValueError('baseline_stale_before_write')
        anchor_q, _, anchor_tcp = values(s)
        baseline_completed_ns = s['completed_ns']
        start = clock()

        async def send_once():
            before_write()
            if clock() - baseline_completed_ns > 100_000_000:
                raise ValueError('baseline_stale_at_dispatch')
            rpc_started_ns = clock()
            emit({'kind': 'system_stop_requested', 'started_ns': rpc_started_ns})
            # Journal fsync may itself be slow or config may change while it
            # runs. Check again immediately before crossing the write boundary.
            before_write()
            if clock() - baseline_completed_ns > 100_000_000:
                raise ValueError('baseline_stale_at_dispatch')
            result['write_started'] = True
            result['rpc_outcome'] = 'pending'
            try:
                await client.stop_sys()
                result['rpc_outcome'] = 'returned'
            except asyncio.CancelledError:
                result['rpc_outcome'] = 'unknown_on_local_cancel'
                raise
            except Exception as error:
                result.update(rpc_outcome='error', rpc_error_type=type(error).__name__)
            finally:
                rpc_completed_ns = clock()
                result['rpc_elapsed_ns'] = rpc_completed_ns - rpc_started_ns
                if result['rpc_elapsed_ns'] >= 5_000_000_000:
                    result['rpc_wait_exceeded'] = True
                emit({'kind': 'system_stop_rpc_end', 'completed_ns': rpc_completed_ns,
                      'outcome': result['rpc_outcome']})

        task = asyncio.create_task(send_once())
        await sleep(0)  # Dispatch before starting the observation interval.
        if task.done():
            task.result()
        pending_reported = False
        anomalous = False
        last = None
        while clock() - start < 60_000_000_000:
            s = await read_sample(client, clock)
            emit({'kind': 'observation_sample', 'sample': s})
            validate_diagnostic_channels(s)
            q, qd, tcp = values(s)
            result['max_joint_drift_rad'] = max(result['max_joint_drift_rad'], max(abs(a-b) for a, b in zip(q, anchor_q)))
            result['max_tcp_drift_m'] = max(result['max_tcp_drift_m'], math.dist(tcp.p, anchor_tcp.p))
            anomalous |= (max(abs(v) for v in qd) > .02 or estop_fault(s['raw_estop']) is not None
                          or result['max_joint_drift_rad'] > .001 or result['max_tcp_drift_m'] > .0005)
            # Transient unexpected states remain a failure even if the final
            # snapshot later looks settled. Keep raw states in the journal.
            anomalous |= any(s[key] not in ('IDLE', 'STOPPING', 'STOP', 5, 10, 12)
                             for key in ('raw_state', 'raw_state_after'))
            if (s['completed_ns'] - (last['completed_ns'] if last else baseline_completed_ns)
                    > 300_000_000):
                anomalous = True
            last = s
            if not task.done() and clock() - start >= 5_000_000_000 and not pending_reported:
                emit({'kind': 'rpc_pending', 'observed_ns': clock()})
                pending_reported = True
                result['rpc_wait_exceeded'] = True
            if task.done():
                task.result()  # Recorder errors must not be silently ignored.
            await sleep(.2)
        result['observation_completed'] = True
        result['final_sample'] = last
        stopped = all(last[key] in ('STOP', 12) for key in ('raw_state', 'raw_state_after'))
        result['complete'] = bool(not anomalous and not result.get('rpc_wait_exceeded')
                                  and stopped and result['rpc_outcome'] == 'returned')
    except asyncio.CancelledError:
        result.update(interrupted=True, complete=False)
        raise
    except Exception as error:
        result.update(error_type=type(error).__name__, error=str(error), complete=False)
    finally:
        if task is not None and not task.done():
            task.cancel()
            await asyncio.wait({task}, timeout=.05)
        if task is not None:
            task.add_done_callback(lambda t: None if t.cancelled() else t.exception())
        emit({'kind': 'probe_summary', **result, 'completed_ns': clock()})
    return result
