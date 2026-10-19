"""Offline finite-window rehearsal with an explicit synthetic controller only.

There is no native SDK import, network factory, live mode, or hardware target.
Frozen feedback seeds numeric inputs; the affine FK and instantaneous actuator
are synthetic and cannot establish physical motion or stopping performance.
"""
import argparse
import asyncio
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'backend'))
from app.commissioning.finite_diagnostic import FiniteWindowDiagnosticAdapter
from app.commissioning.idle_stop_baseline import validate_idle_sample, validate_position_continuity
from app.commissioning.stop_observation import observe_after_stop
from app.recording.commissioning import CommissioningRecorder
from app.robots.base import StopReason
from app.robots.lebai_codec import pose_to_lebai
from app.schemas.messages import Pose

spec = importlib.util.spec_from_file_location('offline_finite_idle_helpers', Path(__file__).with_name('real_robot_idle_stop.py'))
idle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(idle)

SCENARIOS = ('normal', 'fk_mismatch', 'fk_timeout', 'ik_timeout', 'expired_lease', 'stop_late', 'stop_moving')


def load_feedback(path):
    data = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    value = data.get('initial')
    if not isinstance(value, dict):
        raise ValueError('offline_feedback_missing')
    for key in ('q', 'qd', 'qdd'):
        values = value.get(key)
        if (not isinstance(values, list) or len(values) != 6
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
            raise ValueError('offline_feedback_invalid:' + key)
    # These poses are fixture inputs, never actual live calibration evidence.
    try:
        Pose.model_validate(value.get('tcp'))
        Pose.model_validate(value.get('tcp_setting'))
    except Exception:
        raise ValueError('offline_feedback_invalid_pose') from None
    return copy.deepcopy(value)


class SyntheticController:
    """Affine +Z per joint-1-radian, immediate state update; no physics engine."""
    def __init__(self, feedback, scenario):
        self.origin = copy.deepcopy(feedback)
        self.q, self.qd, self.qdd = (list(feedback[k]) for k in ('q', 'qd', 'qdd'))
        self.state = 'IDLE'  # Deliberate fixture state, never a field-state conversion.
        self.scenario = scenario
        self.writes = []
        self.ik_entries = self.fk_entries = 0

    async def is_connected(self):
        return True

    async def get_robot_state(self):
        return self.state

    async def get_estop_reason(self):
        return 'NONE'

    async def get_running_motion(self):
        return None

    async def get_motion_state(self, _motion_id):
        return 'FINISHED'

    async def get_claw(self):
        return dict(force=0, amplitude=100)

    async def get_tcp(self):
        return pose_to_lebai(Pose.model_validate(self.origin['tcp_setting']))

    def pose(self, joints):
        origin = Pose.model_validate(self.origin['tcp'])
        return Pose(p=(origin.p[0], origin.p[1], origin.p[2] + joints[0] - self.origin['q'][0]), q=origin.q)

    async def get_kin_data(self):
        pose = pose_to_lebai(self.pose(self.q))
        return dict(actual_joint_pose=list(self.q), actual_joint_speed=list(self.qd), actual_joint_acc=list(self.qdd),
            target_joint_pose=list(self.q), target_joint_speed=[0.]*6, target_joint_acc=[0.]*6,
            actual_tcp_pose=pose, target_tcp_pose=pose.copy(), actual_flange_pose=pose.copy())

    async def kinematics_inverse(self, pose, joints):
        self.ik_entries += 1
        if self.scenario == 'ik_timeout':
            await asyncio.sleep(.3)
        elif self.scenario == 'expired_lease':
            # Windows' coarse monotonic clock and asyncio's timer lookahead
            # can count a single 105 ms wait as only 93--94 ms. This synthetic
            # fault must actually exceed the adapter's 100 ms lease clock.
            deadline_ns = time.monotonic_ns() + 105_000_000
            while (remaining_ns := deadline_ns - time.monotonic_ns()) > 0:
                await asyncio.sleep(remaining_ns / 1e9)
        return [self.origin['q'][0] + pose['z'] - self.origin['tcp']['p'][2], *joints[1:]]

    async def kinematics_forward(self, joints):
        self.fk_entries += 1
        if self.scenario == 'fk_timeout':
            await asyncio.sleep(.2)
        pose = pose_to_lebai(self.pose(joints))
        if self.scenario == 'fk_mismatch':
            pose['x'] += .001
        return pose

    async def move_pvat(self, p, v, a, t):
        self.writes.append(['move_pvat', list(p), list(v), list(a), t])
        self.q, self.qd, self.qdd = list(p), list(v), list(a)
        return 1

    async def stop_move(self):
        self.writes.append(['stop_move'])
        if self.scenario == 'stop_late':
            await asyncio.sleep(.621)
        if self.scenario == 'stop_moving':
            self.state, self.qd = 'MOVING', [.01]*6

    async def stop_sys(self):
        raise ValueError('offline_action_forbidden')

    async def movej(self, *args):
        raise ValueError('offline_action_forbidden')

    async def set_claw(self, *args):
        raise ValueError('offline_action_forbidden')


async def run(config, fixture, output, *, scenario='normal'):
    if scenario not in SCENARIOS:
        raise ValueError('offline_scenario_invalid')
    if 'lebai_sdk' in sys.modules:
        raise ValueError('offline_rehearsal_native_sdk_already_loaded')
    feedback = load_feedback(fixture)
    plan = idle.prepare(config, output)  # Parse existing caps; never calls idle.execute.
    settings = plan['settings']
    if (not 0 < settings.control.max_joint_speed_radps <= .05
            or not 0 < settings.control.max_joint_acceleration_radps2 <= .2
            or not .01 <= settings.control.pvat_horizon_s <= .08):
        raise ValueError('offline_rehearsal_cap_invalid')
    output = plan['output']
    output.mkdir(parents=True, exist_ok=False)
    controller = SyntheticController(feedback, scenario)
    events = []
    recorder = CommissioningRecorder(output / 'sessions', metadata=dict(runtime='SIMULATION', workflow='offline_finite_window'))

    async def record(event, timestamp):
        events.append(copy.deepcopy(event))
        await recorder.write_event(event, timestamp)

    async def factory(_configured_ip):
        # The configured IP is never used for any network operation.
        return controller

    adapter = FiniteWindowDiagnosticAdapter(settings, client_factory=factory, event_callback=record)
    report = dict(runtime='SIMULATION', simulation_only=True, hardware_verified=False, motion_authorized=False,
        controller_state_is_synthetic=True, source_state=feedback.get('state'), scenario=scenario,
        assessment='simulation_only_not_live_acceptance', complete=False, tail_complete=False,
        stop_confirmed=False, latched_fault=None, cleanup_errors=[], native_sdk_loaded=False)
    primary_error = None
    try:
        await recorder.start()
        await adapter.connect()
        preflight = await adapter.preflight()
        if not preflight.ready:
            raise ValueError('offline_rehearsal_preflight_rejected')
        origin = adapter._snapshot.actual_tcp
        target = Pose(p=(origin.p[0], origin.p[1], origin.p[2] + .0002), q=origin.q)
        receipt = await adapter.submit_tcp(target, 1)
        outcome = await asyncio.wait_for(receipt.completion.wait(), 2.)
        report['command_sdk_outcome'] = outcome.sdk_outcome
        report['command_error'] = outcome.error
        await adapter.stop(StopReason.SHUTDOWN)
        anchor = await adapter.read_stop_observation()

        class Tail:
            async def read_stop_observation(self):
                sample = await adapter.read_stop_observation()
                validate_idle_sample(sample, time.monotonic_ns())
                validate_position_continuity(sample, anchor)
                return sample

        await observe_after_stop(Tail(), recorder, 2.)
        report['tail_complete'] = True
    except BaseException as error:
        primary_error = error
        report['error_type'] = type(error).__name__
    finally:
        try:
            await adapter.disconnect()
        except BaseException as error:
            report['cleanup_errors'].append(type(error).__name__)
            if primary_error is None or isinstance(error, (asyncio.CancelledError, KeyboardInterrupt, SystemExit)):
                primary_error = error
        # Disconnect can perform the first owned stop after an interrupted PVAT.
        # Sample the final transaction/latch after that attempt, not before it.
        if adapter._stop_transaction is not None:
            report['stop_confirmed'] = adapter._stop_transaction.confirmed
        report['latched_fault'] = adapter.read_motion_view().latched_fault
        report.update(synthetic_writes=copy.deepcopy(controller.writes), ik_entries=controller.ik_entries,
            fk_entries=controller.fk_entries, pvat_entries=sum(c[0] == 'move_pvat' for c in controller.writes),
            stop_entries=sum(c[0] == 'stop_move' for c in controller.writes),
            audits=[e for e in events if e.get('kind') == 'finite_candidate_audit'])
        report['complete'] = (primary_error is None and report.get('command_sdk_outcome') == 'returned' and report['stop_confirmed']
            and report['latched_fault'] is None and report['tail_complete'] and not report['cleanup_errors']
            and report['pvat_entries'] == report['stop_entries'] == 1)
        try:
            await recorder.close()
        except BaseException as error:
            report['cleanup_errors'].append(type(error).__name__)
            report['complete'] = False
            if primary_error is None or isinstance(error, (asyncio.CancelledError, KeyboardInterrupt, SystemExit)):
                primary_error = error
        report['interrupted'] = isinstance(primary_error, (asyncio.CancelledError, KeyboardInterrupt, SystemExit))
        (output / 'result.json').write_text(json.dumps(report, allow_nan=False, indent=2) + '\n', encoding='utf-8')
    if isinstance(primary_error, (asyncio.CancelledError, KeyboardInterrupt, SystemExit)):
        raise primary_error
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--scenario', choices=SCENARIOS, default='normal')
    args = parser.parse_args(argv)
    result = asyncio.run(run(args.config, args.fixture, args.output, scenario=args.scenario))
    print(json.dumps({k: result[k] for k in ('runtime', 'complete', 'pvat_entries', 'stop_entries', 'latched_fault')}))
    return 0 if result['complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
