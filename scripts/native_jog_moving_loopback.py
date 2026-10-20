"""Changing theoretical feedback over an owned SDK loopback, never a robot.

The trace advances in scripted 20ms reference steps, not controller time.
Its ideal hold models no braking, calibration, tracking error or physical stop.
"""
import argparse
import asyncio
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'backend'))

import numpy as np
from scipy.spatial.transform import Rotation
from app.commissioning.native_jog_mdh import _components, mdh_tcp_jacobian, mdh_tcp_jacobian_rate
from app.commissioning.native_jog_moving_guard import MovingModelJogRegionGuard
from app.commissioning.native_jog_packet import plan_jog
from app.commissioning.native_jog_stream_candidate import LoopbackJogStreamAdapter
from app.commissioning.qpc_event_loop import new_qpc_loop
from app.robots.base import BackendCommandError, StopReason
from scripts.native_jog_stream_loopback import (HOST, CONFIG_SHA, Q, StreamPeer,
    synthetic_context, expected_speed, parse_request as static_parse, _wire_pose, _sdk_identity)
from scripts.native_jog_loopback_probe import _same_wire, _unique_object, owned_port
from scripts.sdk_loopback_stop_probe import _sdk_call
from scripts.application_stop_loopback_probe import fixture_result

CASES = ('forward','reverse','input_timeout','region_rejected','angular_rejected',
    'speed_reply_late','late_effect','stop_reply_late')
REJECTION_ERRORS = dict(region_rejected='native_jog_moving_reference_leaves_region',
    angular_rejected='native_jog_moving_not_pure_translation')


def parse_request(raw):
    request = json.loads(raw, object_pairs_hook=_unique_object)
    if type(request) is dict and request.get('method') == 'speed_linear':
        # Validate the entire envelope with the old strict parser after matching
        # one of two fixed wires. The old peer whitelist itself is unchanged.
        positive, negative = expected_speed(), expected_speed()
        negative['speed']['position']['z'] = -.003
        if not any(_same_wire(request.get('params'), [wire]) for wire in (positive, negative)):
            raise ValueError('unexpected changing-trace speed parameters')
        canonical = copy.deepcopy(request); canonical['params'] = [positive]
        static_parse(json.dumps(canonical))
        return request
    return static_parse(raw)


class MovingTrace:
    """Deterministic theoretical samples; no claim to simulate speedl physics."""
    def __init__(self):
        self.tool = synthetic_context()[2].tool
        self.q, self.speed, self.model_time_s = Q, 0., 0.
        self.history = []
        self._install(0.)

    def _rates(self, acceleration):
        _, j = mdh_tcp_jacobian(self.q, self.tool)
        qd = np.linalg.solve(j, [0.,0.,self.speed,0.,0.,0.])
        transform, j, jdot = mdh_tcp_jacobian_rate(self.q, tuple(map(float,qd)), self.tool)
        qdd = np.linalg.solve(j, np.array([0.,0.,acceleration,0.,0.,0.])-jdot@qd)
        return transform, qd, qdd

    def _install(self, acceleration):
        transform, qd, qdd = self._rates(acceleration)
        p = list(map(float,transform[:3,3]))
        angles = list(map(float,Rotation.from_matrix(transform[:3,:3]).as_euler('ZYX')))
        self.pose = _wire_pose(p, angles)
        self.qd, self.qdd = list(map(float,qd)), list(map(float,qdd))
        self.history.append(dict(model_time_s=self.model_time_s,q=list(self.q),qd=self.qd.copy(),
            qdd=self.qdd.copy(),tcp_p=p,speed_mps=self.speed,acceleration_mps2=acceleration))

    def step(self, target_speed):
        dt = .02
        change = max(-.008*dt,min(.008*dt,target_speed-self.speed))
        acceleration = change/dt
        _, qd, qdd = self._rates(acceleration)
        self.q = tuple(float(p+d*dt+.5*a*dt*dt) for p,d,a in zip(self.q,qd,qdd,strict=True))
        self.speed += change
        self.model_time_s += dt
        self._install(acceleration)

    def hold(self):
        self.speed = 0.
        self._install(0.)  # An ideal scripted hold, explicitly not a brake model.

    def kinematics(self):
        result = fixture_result('get_kin_data')
        for prefix in ('actual','target'):
            result[prefix+'_joint_pose'] = list(self.q)
            result[prefix+'_joint_speed'] = self.qd.copy()
            result[prefix+'_joint_acc'] = self.qdd.copy()
            result[prefix+'_tcp_pose'] = copy.deepcopy(self.pose)
        result['actual_flange_pose'] = copy.deepcopy(self.pose)
        return result


def moving_context():
    settings, limits, guard, _ = synthetic_context()
    moving = MovingModelJogRegionGuard(settings,limits,CONFIG_SHA,guard.reference_q,guard.tool,
        guard.joint_radius_rad,guard.initial_offset_rad,guard.duration_s)
    return settings, limits, moving


class MovingPeer(StreamPeer):
    def __init__(self, case):
        if case not in CASES:
            raise ValueError('unknown moving trace case')
        super().__init__('normal')
        self.case, self.trace = case, MovingTrace()
        self.damage = False

    async def handler(self, connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    request = parse_request(raw)
                    key = (type(request['id']),request['id']); method = request['method']
                    if (key in self._ids or method == 'speed_linear' and self._speed_count >= 10
                            or method == 'stop_move' and self._stop_count >= 1):
                        raise ValueError('duplicate id or bounded write count exceeded')
                    self._ids.add(key)
                    if method == 'speed_linear': self._speed_count += 1
                    if method == 'stop_move': self._stop_count += 1
                except ValueError as error:
                    self.errors.append(str(error))
                    await connection.close(code=1008,reason='unexpected moving probe request')
                    return
                record = {**copy.deepcopy(request),'received_ns':time.perf_counter_ns()}
                if method == 'speed_linear': record['motion_id'] = self._speed_count+6
                self.records.append(record)
                task = asyncio.create_task(self._reply(connection,record))
                self._tasks.add(task); task.add_done_callback(self._reply_done)
        finally:
            self._connections.discard(connection)

    async def _reply(self, connection, record):
        method = record['method']
        if method == 'speed_linear':
            self.active, self.motion_id = True, record['motion_id']
            self.trace.step(record['params'][0]['speed']['position']['z'])
            if self.case == 'speed_reply_late' and self._speed_count == 2:
                await asyncio.sleep(.1)
            result = dict(id=record['motion_id'])
        elif method == 'stop_move':
            self.active = False; self.trace.hold(); result = {}
            if self.case == 'stop_reply_late': await asyncio.sleep(.25)
            if self.case == 'late_effect':
                async def late_motion():
                    await asyncio.sleep(.1)
                    self.active = True; self.trace.step(.003)
                task = asyncio.create_task(late_motion())
                self._tasks.add(task); task.add_done_callback(self._reply_done)
        elif method == 'get_robot_state': result = dict(state='MOVING' if self.active else 'IDLE')
        elif method == 'get_running_motion': result = dict(id=self.motion_id if self.active else 0)
        elif method == 'get_motion_state': result = dict(state='RUNNING' if self.active else 'FINISHED')
        elif method == 'get_kin_data':
            result = self.trace.kinematics()
            record['nonzero_feedback'] = any(v != 0 for v in result['actual_joint_speed'])
            if self.damage and self.active:
                if self.case == 'region_rejected': result['actual_joint_pose'][2] += .0029
                if self.case == 'angular_rejected': result['target_joint_speed'] = [0.,0.,0.,0.,0.,.00001]
            record['model_feedback'] = copy.deepcopy(result)
        else: result = fixture_result(method)
        record['send_started_ns'] = time.perf_counter_ns()
        await connection.send(json.dumps(dict(jsonrpc='2.0',id=record['id'],result=result)))
        record['send_completed_ns'] = time.perf_counter_ns()


def _transmitted_rejection(record,case,after_ns):
    if (record.get('method') != 'get_kin_data' or type(record.get('received_ns')) is not int
            or record['received_ns'] < after_ns or type(record.get('send_completed_ns')) is not int
            or record['send_completed_ns'] < record['received_ns']):
        return False
    try:
        data = record['model_feedback']
        if case == 'region_rejected':
            actual = _components(data['actual_joint_pose'],6)
            target = _components(data['target_joint_pose'],6)
            return actual[2]-target[2] >= .0028
        velocity = _components(data['target_joint_speed'],6)
        return velocity == (0.,0.,0.,0.,0.,.00001)
    except (KeyError,TypeError,ValueError):
        return False


def assert_case_outcome(result):
    counts = dict(forward=3,reverse=3,input_timeout=1,region_rejected=1,angular_rejected=1,
        speed_reply_late=2,late_effect=1,stop_reply_late=1)
    faults = dict(speed_reply_late='stop_unverified',late_effect='stop_incomplete',stop_reply_late='stop_unverified')
    case = result['case']; count = counts.get(case); fault = faults.get(case)
    accepted = 1 if case == 'speed_reply_late' else count
    if (count is None or result['native_entries'] != count or result['speed_requests'] != count
            or result['accepted_packets'] != accepted or result['stop_requests'] != 1
            or result['confirmed'] != (fault is None) or result['latched_fault'] != fault
            or result['unknown_motion_methods'] != (['speedl'] if case == 'speed_reply_late' else [])
            or not result['local_connection_closed'] or not result['watchdog_finished'] or result['peer_errors']
            or not any(any(v != 0 for v in row['qd']) for row in result['trace'])):
        raise RuntimeError('unexpected moving model outcome: '+case)
    if case in REJECTION_ERRORS:
        calls = result.get('calls',[])
        if (not result['continuation_rejected'] or len(calls) != 2
                or calls[0].get('outcome') != 'returned' or calls[0].get('value') != 7
                or calls[1].get('outcome') != 'error' or calls[1].get('error') != REJECTION_ERRORS[case]
                or type(calls[1].get('started_ns')) is not int
                or not any(_transmitted_rejection(r,case,calls[1]['started_ns'])
                    for r in result.get('rpc_records',[]))):
            raise RuntimeError('missing exact moving-guard rejection evidence: '+case)
    if case in ('forward','reverse') and not any(r.get('nonzero_feedback') for r in result['rpc_records']):
        raise RuntimeError('no changing feedback traversed the SDK')


async def one_case(sdk,case):
    from websockets.asyncio.server import serve
    settings,limits,guard = moving_context()
    peer = MovingPeer(case); adapter = client = None; calls = []; events = []
    async def event(value,stamp): events.append(dict(event=copy.deepcopy(value),stamp_ns=stamp))
    async with serve(peer.handler,HOST,0) as server:
        port = owned_port([sock.getsockname() for sock in server.sockets])
        async def factory(host):
            nonlocal client
            if host != HOST: raise ValueError('owned literal loopback required')
            client = await asyncio.wait_for(sdk.connect(HOST,port,{'request_timeout':2}),3)
            return client
        try:
            adapter = LoopbackJogStreamAdapter(settings,limits=limits,config_sha256=CONFIG_SHA,
                client_factory=factory,clock=time.perf_counter_ns,sleep=asyncio.sleep,
                event_callback=event,model_region_guard=guard)
            await adapter.connect()
            signs = (1,-1,-1) if case == 'reverse' else ((1,1,1) if case == 'forward'
                else (1,1) if case in ('region_rejected','angular_rejected','speed_reply_late') else (1,))
            for seq,sign in enumerate(signs,1):
                if seq == 2 and case in ('region_rejected','angular_rejected'): peer.damage = True
                now = time.perf_counter_ns()
                adapter.offer_jog(plan_jog((0,0,sign),1,limits=limits,actual_p=adapter._snapshot.actual_tcp.p,
                    snapshot_ns=adapter._snapshot.captured_ns,now_ns=now,generation=adapter._pump.generation,
                    sequence=seq,input_ns=now,input_expires_ns=now+100_000_000,duration_s=.04,config_sha256=CONFIG_SHA))
                calls.append(await _sdk_call(adapter.dispatch_latest))
                if calls[-1]['outcome'] != 'returned': break
            if case == 'input_timeout': await asyncio.wait_for(asyncio.shield(adapter._watchdog),1.)
            stop = await _sdk_call(lambda:adapter.revoke_stream(StopReason.GRIP_RELEASED))
            if peer._tasks: await asyncio.wait(tuple(peer._tasks),timeout=1.)
            transaction = adapter._stop_transaction
            result = dict(case=case,calls=calls,stop=stop,events=events,
                accepted_packets=sum(type(c.get('value')) is int and c['outcome']=='returned' for c in calls),
                native_entries=adapter.native_jog_entries,confirmed=bool(transaction and transaction.confirmed),
                latched_fault=adapter._latched_fault,unknown_motion_methods=sorted(adapter._motion_calls.unknown_methods),
                continuation_rejected=(case in REJECTION_ERRORS and calls[-1]['outcome']=='error'
                    and calls[-1].get('error') == REJECTION_ERRORS[case]),
                trace=peer.trace.history,rpc_records=peer.records,peer_errors=peer.errors,listener=dict(host=HOST,port=port))
            try: await adapter.disconnect()
            except BackendCommandError: pass
            result['watchdog_finished'] = adapter._watchdog.done(); adapter = None
            await peer.close(); await asyncio.wait_for(client.wait_disconnect(),2.)
            result.update(local_connection_closed=not await client.is_connected(),
                speed_requests=sum(r['method']=='speed_linear' for r in peer.records),
                stop_requests=sum(r['method']=='stop_move' for r in peer.records))
            return result
        finally:
            if adapter is not None:
                try: await adapter.disconnect()
                except BackendCommandError: pass
            await peer.close()
            if client is not None: await asyncio.wait_for(client.wait_disconnect(),2.)


async def run_matrix():
    if getattr(asyncio.get_running_loop(),'vr4arm_timing_clock',None) != 'time.perf_counter_ns':
        raise ValueError('isolated QPC loop required')
    sdk,identity = _sdk_identity(); sdk.init()
    cases = [await one_case(sdk,case) for case in CASES]
    sources = sorted((ROOT/'backend/app').rglob('*.py')) + [Path(__file__).resolve()] + [ROOT/p for p in (
        'scripts/native_jog_stream_loopback.py','scripts/native_jog_loopback_probe.py',
        'scripts/application_stop_loopback_probe.py','scripts/sdk_loopback_stop_probe.py')]
    return dict(offline_only=True,real_robot_connections=0,hardware_verified=False,joint_bounds_verified=False,
        braking_verified=False,continuous_control_admitted=False,controller_tracking_verified=False,
        assessment='changing_theoretical_trace_not_controller_or_braking_simulation',
        model_step_s=.02,ideal_hold_is_not_braking=True,sdk=identity,cases=cases,
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True); args = parser.parse_args(argv)
    try:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        with args.output.open('x',encoding='utf-8') as stream:
            try:
                with asyncio.Runner(loop_factory=new_qpc_loop) as runner: result = runner.run(run_matrix())
            except Exception as error:
                json.dump(dict(offline_only=True,error_type=type(error).__name__,error=str(error)),stream,indent=2)
                raise
            json.dump(result,stream,indent=2,allow_nan=False)
        for case in result['cases']: assert_case_outcome(case)
        print(json.dumps(dict(output=str(args.output),scenarios=len(result['cases']),hardware_verified=False)))
        return 0
    except (OSError,ValueError,RuntimeError) as error: parser.error(str(error))


if __name__ == '__main__': raise SystemExit(main())
