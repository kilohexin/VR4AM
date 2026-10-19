"""Bounded held-input protocol matrix, exclusively against an owned loopback.

Synthetic zero-rate telemetry tests continuation ownership, not moving-state
model admissibility, actual tracking, braking, or a real-robot stop.
"""
import argparse
import asyncio
import copy
from dataclasses import replace
import hashlib
import importlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'backend'))

from scipy.spatial.transform import Rotation
from app.commissioning.native_jog_mdh import MdhTool, mdh_tcp_jacobian
from app.commissioning.native_jog_model_guard import ModelJogRegionGuard
from app.commissioning.native_jog_packet import NativeJogLimits, plan_jog
from app.commissioning.native_jog_stream_candidate import LoopbackJogStreamAdapter
from app.commissioning.qpc_event_loop import new_qpc_loop
from app.robots.base import BackendCommandError, StopReason
from scripts.application_stop_loopback_probe import fixture_result, loopback_settings
from scripts.sdk_loopback_stop_probe import LoopbackPeer, _sdk_call
from scripts.native_jog_loopback_probe import owned_port, _same_wire, _unique_object

HOST = '127.0.0.1'
SDK_BINARY = ROOT/'.venv/Lib/site-packages/lebai_sdk/lebai_sdk.pyd'
SDK_SHA256 = 'ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b'
CONFIG_SHA = hashlib.sha256(b'held-input-synthetic-loopback-only-v1').hexdigest()
CASES = ('normal', 'replace_unsent', 'input_timeout', 'speed_reply_late', 'late_effect',
    'foreign_id', 'tool_changed', 'region_changed', 'stop_reply_late')
READS = frozenset(('get_robot_state', 'get_estop_reason', 'get_kin_data', 'get_tcp',
    'get_claw', 'get_running_motion'))
Q = (0., -1., 1., 0., 1.57, 0.)


def _wire_pose(p, angles=(0., 0., 0.)):
    return dict(position=dict(zip(('x', 'y', 'z'), p)),
        rotation=dict(euler_zyx=dict(zip(('z', 'y', 'x'), angles))))


def expected_speed():
    return dict(speed=_wire_pose((0., 0., .003)),
        param=dict(acc=.01, time=.04, constrained=True),
        frame=dict(position_kind='CUSTOM', rotation_kind='CUSTOM', **_wire_pose((0., 0., 0.))))


def parse_request(raw):
    request = json.loads(raw, object_pairs_hook=_unique_object)
    if (type(request) is not dict or set(request) != {'jsonrpc', 'id', 'method', 'params'}
            or request['jsonrpc'] != '2.0' or type(request['id']) not in (int, str)
            or type(request['method']) is not str):
        raise ValueError('unexpected loopback RPC envelope')
    method, params = request['method'], request['params']
    if method == 'speed_linear':
        valid = _same_wire(params, [expected_speed()]) if type(params) is not list else (
            len(params) == 1 and _same_wire(params[0], expected_speed()))
    elif method == 'get_motion_state':
        valid = (type(params) is list and len(params) == 1 and type(params[0]) is dict
            and set(params[0]) == {'id'} and type(params[0]['id']) is int and 7 <= params[0]['id'] <= 16)
    else:
        valid = method in READS | {'stop_move'} and params == [{}]
    if not valid:
        raise ValueError('unexpected loopback RPC method/parameters')
    return request


def synthetic_context():
    tool = MdhTool((0., 0., .175), (0., 0., 0., 1.))
    transform, _ = mdh_tcp_jacobian(Q, tool)
    position = tuple(float(x) for x in transform[:3, 3])
    angles = tuple(float(x) for x in Rotation.from_matrix(transform[:3, :3]).as_euler('ZYX'))
    lo, hi = tuple(v-.1 for v in position), tuple(v+.1 for v in position)
    base = loopback_settings()
    settings = replace(base, teleop_ready_q=Q, startup_tcp_min_m=lo, startup_tcp_max_m=hi,
        control=replace(base.control, max_joint_speed_radps=.05, max_joint_acceleration_radps2=.2))
    limits = NativeJogLimits(.003, .01, lo, hi, (.001,)*3)
    guard = ModelJogRegionGuard(settings, limits, CONFIG_SHA, Q, tool,
        (.003,)*6, (.0002,)*6, .04)
    return settings, limits, guard, _wire_pose(position, angles)


class StreamPeer(LoopbackPeer):
    def __init__(self, case):
        if case not in CASES:
            raise ValueError('unknown stream case')
        super().__init__()
        self.case = case
        self.active = False
        self.motion_id = 0
        self.foreign = self.tool_changed = self.region_changed = False
        self._speed_count = self._stop_count = 0
        self.pose = synthetic_context()[3]

    async def handler(self, connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    request = parse_request(raw)
                    key = (type(request['id']), request['id'])
                    if key in self._ids:
                        raise ValueError('duplicate RPC id')
                    method = request['method']
                    if (method == 'speed_linear' and self._speed_count >= 10
                            or method == 'stop_move' and self._stop_count >= 1):
                        raise ValueError('write limit exceeded')
                    self._ids.add(key)
                    if method == 'speed_linear':
                        self._speed_count += 1
                    if method == 'stop_move':
                        self._stop_count += 1
                except ValueError as error:
                    self.errors.append(str(error))
                    await connection.close(code=1008, reason='unexpected stream probe request')
                    return
                record = {**copy.deepcopy(request), 'received_ns': time.perf_counter_ns()}
                if method == 'speed_linear':
                    record['motion_id'] = self._speed_count+6
                self.records.append(record)
                task = asyncio.create_task(self._reply(connection, record))
                self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        finally:
            self._connections.discard(connection)

    async def _reply(self, connection, record):
        method = record['method']
        if method == 'speed_linear':
            self.active, self.motion_id = True, record['motion_id']
            if self.case == 'speed_reply_late':
                await asyncio.sleep(.1)
            result = dict(id=record['motion_id'])
        elif method == 'stop_move':
            self.active = False
            result = {}
            if self.case == 'stop_reply_late':
                await asyncio.sleep(.25)
            if self.case == 'late_effect':
                async def late_motion():
                    await asyncio.sleep(.1)
                    self.active = True
                task = asyncio.create_task(late_motion())
                self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        elif method == 'get_robot_state':
            result = dict(state='MOVING' if self.active else 'IDLE')
        elif method == 'get_running_motion':
            result = dict(id=(999 if self.foreign else self.motion_id) if self.active else 0)
        elif method == 'get_motion_state':
            result = dict(state='RUNNING' if self.active else 'FINISHED')
        else:
            result = fixture_result(method)
            if method == 'get_kin_data':
                for prefix in ('actual', 'target'):
                    result[prefix+'_tcp_pose'] = copy.deepcopy(self.pose)
                result['actual_flange_pose'] = copy.deepcopy(self.pose)
                if self.region_changed:
                    result['actual_joint_pose'][0] += .001
            if method == 'get_tcp' and self.tool_changed:
                result['position']['z'] = .18
        record['send_started_ns'] = time.perf_counter_ns()
        await connection.send(json.dumps(dict(jsonrpc='2.0', id=record['id'], result=result)))
        record['send_completed_ns'] = time.perf_counter_ns()


def _sdk_identity():
    binary = SDK_BINARY.resolve(strict=True)
    digest = hashlib.sha256(binary.read_bytes()).hexdigest()
    if digest != SDK_SHA256:
        raise ValueError('SDK binary identity mismatch before import')
    sdk = importlib.import_module('lebai_sdk')
    loaded = Path(importlib.import_module('lebai_sdk.lebai_sdk').__file__).resolve(strict=True)
    if loaded != binary or hashlib.sha256(loaded.read_bytes()).hexdigest() != digest:
        raise ValueError('SDK loaded identity mismatch')
    return sdk, dict(binary_sha256=digest, native_version=sdk.version())


def assert_case_outcome(result):
    case = result['case']
    counts = dict(normal=3, replace_unsent=1, input_timeout=1, speed_reply_late=1,
        late_effect=1, foreign_id=1, tool_changed=1, region_changed=1, stop_reply_late=1)
    faults = dict(speed_reply_late='stop_unverified', late_effect='stop_incomplete',
        stop_reply_late='stop_unverified')
    if case not in counts:
        raise RuntimeError('unknown stream result')
    fault = faults.get(case)
    expected_accepted = 0 if case == 'speed_reply_late' else counts[case]
    if (result['native_entries'] != counts[case] or result['speed_requests'] != counts[case]
            or result['stop_requests'] != 1 or result['accepted_packets'] != expected_accepted
            or result['confirmed'] != (fault is None) or result['latched_fault'] != fault
            or result['unknown_motion_methods'] != (['speedl'] if case == 'speed_reply_late' else [])
            or not result['local_connection_closed'] or result['peer_errors']):
        raise RuntimeError('unexpected stream outcome: '+case)
    if case in ('foreign_id', 'tool_changed', 'region_changed') and not result.get('continuation_rejected'):
        raise RuntimeError('unsafe continuation was not rejected: '+case)


async def _one_case(sdk, case):
    from websockets.asyncio.server import serve
    settings, limits, guard, _ = synthetic_context()
    peer = StreamPeer(case)
    adapter = client = None
    events, calls = [], []
    async def record(event, stamp):
        events.append(dict(event=copy.deepcopy(event), stamp_ns=stamp))
    async with serve(peer.handler, HOST, 0) as server:
        port = owned_port([sock.getsockname() for sock in server.sockets])
        async def factory(host):
            nonlocal client
            if host != HOST:
                raise ValueError('owned loopback required')
            client = await asyncio.wait_for(sdk.connect(HOST, port, {'request_timeout': 2}), 3)
            return client
        try:
            adapter = LoopbackJogStreamAdapter(settings, limits=limits, config_sha256=CONFIG_SHA,
                client_factory=factory, clock=time.perf_counter_ns, sleep=asyncio.sleep,
                event_callback=record, model_region_guard=guard)
            await adapter.connect()
            def offer(seq):
                now = time.perf_counter_ns()
                packet = plan_jog((0, 0, 1), 1, limits=limits, actual_p=adapter._snapshot.actual_tcp.p,
                    snapshot_ns=adapter._snapshot.captured_ns, now_ns=now, generation=adapter._pump.generation,
                    sequence=seq, input_ns=now, input_expires_ns=now+100_000_000,
                    duration_s=.04, config_sha256=CONFIG_SHA)
                adapter.offer_jog(packet)
            if case == 'replace_unsent':
                for seq in (1, 2, 3):
                    offer(seq)
                calls.append(await _sdk_call(adapter.dispatch_latest))
            else:
                for seq in range(1, 4 if case == 'normal' else 2):
                    offer(seq)
                    calls.append(await _sdk_call(adapter.dispatch_latest))
                    if calls[-1]['outcome'] != 'returned':
                        break
            continuation_rejected = False
            if case in ('foreign_id', 'tool_changed', 'region_changed') and calls[-1]['outcome'] == 'returned':
                setattr(peer, dict(foreign_id='foreign', tool_changed='tool_changed', region_changed='region_changed')[case], True)
                offer(2)
                call = await _sdk_call(adapter.dispatch_latest)
                calls.append(call)
                continuation_rejected = call['outcome'] == 'error'
            if case == 'input_timeout':
                # No new adapter calls or offers while awaiting autonomous expiry.
                await asyncio.wait_for(asyncio.shield(adapter._watchdog), 1.)
            stop = await _sdk_call(lambda: adapter.revoke_stream(StopReason.GRIP_RELEASED))
            if peer._tasks:
                await asyncio.wait(tuple(peer._tasks), timeout=1.)
            transaction = adapter._stop_transaction
            result = dict(case=case, accepted_packets=sum(c.get('outcome') == 'returned' and
                type(c.get('value')) is int for c in calls), calls=calls, stop=stop,
                native_entries=adapter.native_jog_entries, confirmed=bool(transaction and transaction.confirmed),
                latched_fault=adapter._latched_fault, unknown_motion_methods=sorted(adapter._motion_calls.unknown_methods),
                continuation_rejected=continuation_rejected, rpc_records=peer.records, events=events,
                listener=dict(host=HOST, port=port), peer_errors=peer.errors)
            try:
                await adapter.disconnect()
            except BackendCommandError:
                pass
            result['watchdog_finished'] = adapter._watchdog.done()
            adapter = None
            await peer.close()
            await asyncio.wait_for(client.wait_disconnect(), 2.)
            result.update(local_connection_closed=not await client.is_connected(),
                speed_requests=sum(r['method'] == 'speed_linear' for r in peer.records),
                stop_requests=sum(r['method'] == 'stop_move' for r in peer.records))
            return result
        finally:
            if adapter is not None:
                try:
                    await adapter.disconnect()
                except BackendCommandError:
                    pass
            await peer.close()
            if client is not None:
                await asyncio.wait_for(client.wait_disconnect(), 2.)


async def run_matrix():
    if getattr(asyncio.get_running_loop(), 'vr4arm_timing_clock', None) != 'time.perf_counter_ns':
        raise ValueError('isolated QPC loop required')
    sdk, identity = _sdk_identity()
    sdk.init()
    cases = [await _one_case(sdk, case) for case in CASES]
    return dict(offline_only=True, real_robot_connections=0, hardware_verified=False,
        joint_bounds_verified=False, braking_verified=False, continuous_control_admitted=False,
        assessment='protocol_continuation_only_synthetic_zero_rate_telemetry',
        sdk=identity, clock='time.perf_counter_ns', cases=cases,
        source_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
            'backend/app/commissioning/native_jog_stream_gate.py',
            'backend/app/commissioning/native_jog_stream_candidate.py',
            'backend/app/commissioning/native_jog_model_guard.py', 'scripts/native_jog_stream_loopback.py')})


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open('x', encoding='utf-8') as stream:
            try:
                with asyncio.Runner(loop_factory=new_qpc_loop) as runner:
                    result = runner.run(run_matrix())
            except Exception as error:
                json.dump(dict(offline_only=True, error_type=type(error).__name__, error=str(error)), stream, indent=2)
                raise
            json.dump(result, stream, indent=2, allow_nan=False)
        # Save every case before checking, preserving evidence of unexpected results.
        for case in result['cases']:
            assert_case_outcome(case)
        print(json.dumps(dict(output=str(args.output), scenarios=len(result['cases']), hardware_verified=False)))
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
