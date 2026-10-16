"""Compare shared and independent stop clients on an owned IPv4 fake peer.

This transport experiment does not change the production stop transaction.
No address, port, configuration, or arbitrary command can be supplied by CLI.
"""
import argparse
import asyncio
import copy
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'backend'))

from app.commissioning.qpc_event_loop import new_qpc_loop
from scripts.native_jog_loopback_probe import (
    _sdk_identity, owned_port, parse_request as parse_jog_request,
)
from scripts.sdk_loopback_stop_probe import HOST, LoopbackPeer, _sdk_call

CASES = ('multiplexed_motion_reply', 'application_wait_lock',
         'controller_stop_delay', 'late_motion_effect')


def parse_request(raw):
    request = parse_jog_request(raw)
    if request['method'] not in {'speed_linear', 'stop_move'}:
        raise ValueError('only fixed speedl and stop_move permitted')
    return request


async def connect_owned(sdk, host, port, listeners):
    if host != HOST or type(port) is not int or (host, port) not in listeners:
        raise ValueError('unowned or nonlocal target refused')
    return await asyncio.wait_for(sdk.connect(HOST, port, {'request_timeout': 2}), 3)


class ComparisonPeer(LoopbackPeer):
    def __init__(self, case):
        if case not in CASES:
            raise ValueError('unknown comparison case')
        super().__init__()
        self.case = case
        self.motion_received = asyncio.Event()
        self.connection_count = 0
        self.motion_effect_ns = None
        self.stop_effect_ns = None

    async def handler(self, connection):
        self.connection_count += 1
        connection_id = self.connection_count
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    request = parse_request(raw)
                    # JSON-RPC IDs are scoped to their connection, not global.
                    key = (connection_id, type(request['id']), request['id'])
                    if key in self._ids:
                        raise ValueError('duplicate connection-scoped RPC id')
                    self._ids.add(key)
                except ValueError as error:
                    self.errors.append(str(error))
                    await connection.close(code=1008, reason='unexpected request')
                    return
                record = {**copy.deepcopy(request), 'connection_id': connection_id,
                          'received_ns': time.perf_counter_ns()}
                self.records.append(record)
                if request['method'] == 'speed_linear':
                    self.motion_received.set()
                else:
                    self.stop_received.set()
                task = asyncio.create_task(self._reply(connection, record))
                self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        finally:
            self._connections.discard(connection)

    async def _reply(self, connection, record):
        motion = record['method'] == 'speed_linear'
        if motion:
            # A held reply models an outstanding motion RPC. Only the late
            # effect case also models delayed application of the command.
            if self.case != 'late_motion_effect':
                self.motion_effect_ns = time.perf_counter_ns()
            await asyncio.sleep(.6)
            if self.case == 'late_motion_effect':
                self.motion_effect_ns = time.perf_counter_ns()
        elif self.case == 'controller_stop_delay':
            await asyncio.sleep(.6)  # Global controller delay, either client.
        if not motion:
            self.stop_effect_ns = time.perf_counter_ns()
        record['send_started_ns'] = time.perf_counter_ns()
        await connection.send(json.dumps(dict(jsonrpc='2.0', id=record['id'],
            result={'id': 7} if motion else {})))
        record['send_completed_ns'] = time.perf_counter_ns()


async def one_case(sdk, case, independent):
    from websockets.asyncio.server import serve
    peer = ComparisonPeer(case)
    clients, tasks = [], []
    async with serve(peer.handler, HOST, 0) as server:
        port = owned_port([s.getsockname() for s in server.sockets])
        listeners = {(HOST, port)}
        try:
            client = await connect_owned(sdk, HOST, port, listeners)
            clients.append(client)
            lock = asyncio.Lock()

            async def motion():
                async with lock:
                    return await client.speedl(.02, dict(x=0., y=0., z=.005,
                        rx=0., ry=0., rz=0.), .08,
                        dict.fromkeys(('x','y','z','rx','ry','rz'), 0.))

            motion_task = asyncio.create_task(_sdk_call(motion))
            tasks.append(motion_task)
            await asyncio.wait_for(peer.motion_received.wait(), 2)
            intent_ns = time.perf_counter_ns()
            pending_at_intent = not motion_task.done()
            connect_started_ns = connect_completed_ns = None
            stop_client = client
            if independent:
                connect_started_ns = time.perf_counter_ns()
                stop_client = await connect_owned(sdk, HOST, port, listeners)
                connect_completed_ns = time.perf_counter_ns()
                clients.append(stop_client)

            async def stop():
                if case == 'application_wait_lock' and not independent:
                    async with lock:
                        return await stop_client.stop_move()
                return await stop_client.stop_move()

            stop_result = await asyncio.wait_for(_sdk_call(stop), 3)
            motion_result = await asyncio.wait_for(motion_task, 3)
            if motion_result.get('value') != 7 or stop_result['outcome'] != 'returned':
                raise RuntimeError('unexpected native result')
            motion_row, = [r for r in peer.records if r['method'] == 'speed_linear']
            stop_row, = [r for r in peer.records if r['method'] == 'stop_move']
            result = dict(case=case, independent=independent,
                connection_count=peer.connection_count,
                listener=dict(host=HOST, port=port),
                stop_intent_to_peer_ms=(stop_row['received_ns']-intent_ns)/1e6,
                stop_intent_to_reply_ms=(stop_result['completed_ns']-intent_ns)/1e6,
                separate_connect_ms=None if connect_started_ns is None else
                    (connect_completed_ns-connect_started_ns)/1e6,
                motion_pending_at_stop_intent=pending_at_intent,
                motion_reply_after_stop_reply=motion_result['completed_ns']>stop_result['completed_ns'],
                simulated_motion_effect_after_stop=peer.motion_effect_ns>peer.stop_effect_ns,
                # This is a requirement for a future candidate, not a claim
                # that this isolated script exercised the production latch.
                candidate_requires_unknown_result_latch=pending_at_intent,
                production_stop_path_exercised=False,
                motion=motion_result, stop=stop_result, rpc_records=peer.records)
            await peer.close()
            for c in clients:
                await asyncio.wait_for(c.wait_disconnect(), 2)
            result['local_connections_closed'] = all([not await c.is_connected() for c in clients])
            if not result['local_connections_closed'] or peer.errors:
                raise RuntimeError('connection cleanup/protocol failed')
            if peer.connection_count != (2 if independent else 1):
                raise RuntimeError('unexpected client count')
            if result['simulated_motion_effect_after_stop'] != (case == 'late_motion_effect'):
                raise RuntimeError('late effect fixture not exercised')
            return result
        finally:
            for task in tasks:
                if not task.done(): task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            await peer.close()
            for c in clients:
                await asyncio.wait_for(c.wait_disconnect(), 2)


async def run_matrix():
    if getattr(asyncio.get_running_loop(), 'vr4arm_timing_clock', None) != 'time.perf_counter_ns':
        raise ValueError('isolated QPC event loop required')
    sdk, identity = _sdk_identity()
    sdk.init()
    rows = []
    for case in CASES:
        for independent in (False, True):
            rows.append(await asyncio.wait_for(one_case(sdk, case, independent), 10))
    return dict(offline_only=True, hardware_stop_verified=False,
        real_robot_connections=0, sdk=identity, results=rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--output', type=Path, required=True)
    options = parser.parse_args(argv)
    try:
        options.output.parent.mkdir(parents=True, exist_ok=True)
        with options.output.open('x', encoding='utf-8') as stream:
            try:
                with asyncio.Runner(loop_factory=new_qpc_loop) as runner:
                    result = runner.run(run_matrix())
            except Exception as error:
                json.dump(dict(offline_only=True, hardware_stop_verified=False,
                    error=str(error)), stream, indent=2)
                raise
            json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(dict(cases=len(result['results']), output=str(options.output),
            hardware_stop_verified=False)))
        return 0
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    raise SystemExit(main())
