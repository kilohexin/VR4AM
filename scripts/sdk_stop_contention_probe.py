"""Owned loopback only: native SDK stop reply with 24 pending full-state reads."""
import argparse
import asyncio
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'backend'))

from scripts.application_stop_loopback_probe import ApplicationPeer, connect_owned, fixture_result
from scripts.sdk_loopback_stop_probe import HOST, owned_port, _sdk_identity, _sdk_call

READ_METHODS = ('get_robot_state', 'get_estop_reason', 'get_kin_data',
                'get_tcp', 'get_claw', 'get_running_motion')
CASES = (
    ('immediate_no_reads', 0, 0.0, 0.0, False),
    ('immediate_24_fast_reads', 24, 0.0, 0.0, False),
    ('immediate_24_delayed_reads', 24, 0.0, 0.1, False),
    ('delayed_stop_no_reads', 0, 0.6, 0.0, False),
    ('delayed_stop_24_fast_reads', 24, 0.6, 0.0, False),
    ('delayed_stop_24_delayed_reads', 24, 0.6, 0.1, False),
    ('rpc_error_24_delayed_reads', 24, 0.0, 0.1, True),
)


def native_pose(wire):
    position, rotation = wire['position'], wire['rotation']['euler_zyx']
    return {**position, **{f'r{axis}': rotation[axis] for axis in ('x', 'y', 'z')}}


def validate_read_results(rows):
    for row in rows:
        method = row.get('method')
        if method not in READ_METHODS or row.get('outcome') != 'returned' or 'value' not in row:
            raise RuntimeError('native read failed or has an invalid method/result')
        expected = fixture_result(method)
        if method == 'get_robot_state':
            expected = expected['state']
        elif method == 'get_estop_reason':
            expected = expected['reason']
        elif method == 'get_running_motion':
            expected = expected['id']
        elif method == 'get_tcp':
            expected = native_pose(expected)
        elif method == 'get_claw':
            expected = {**expected, 'weight': 0.0, 'hold_on': False}
        elif method == 'get_kin_data':
            expected = {key: native_pose(value) if isinstance(value, dict) else value
                        for key, value in expected.items()}
        if row['value'] != expected:
            raise RuntimeError('native read does not match the synthetic fixture')


class ContentionPeer(ApplicationPeer):
    def __init__(self, *, read_count=0, delay_s=0.0, read_delay_s=0.0, reply_error=False):
        if type(read_count) is not int or read_count not in (0, 24):
            raise ValueError('read_count must be fixed at 0 or 24')
        for value, maximum in ((delay_s, .6), (read_delay_s, .1)):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
                raise ValueError('simulated delay must be finite and bounded')
        super().__init__(delay_s=delay_s, reply_error=reply_error)
        self.read_count = read_count
        self.read_delay_s = read_delay_s
        self.reads_pending = asyncio.Event()
        self._pending_reads = 0
        if read_count == 0:
            self.reads_pending.set()

    async def _reply(self, connection, record):
        if record['method'] != 'stop_move':
            self._pending_reads += 1
            if self._pending_reads == self.read_count:
                self.reads_pending.set()
            # Controlled backlog: all reads reach the peer before stop is sent.
            # This gate is a simulator fixture, not a controller behavior claim.
            await self.stop_received.wait()
            if self.read_delay_s:
                await asyncio.sleep(self.read_delay_s)
        await super()._reply(connection, record)


async def one_case(sdk, name, read_count, delay_s, read_delay_s, reply_error):
    from websockets.asyncio.server import serve

    peer = ContentionPeer(read_count=read_count, delay_s=delay_s,
                          read_delay_s=read_delay_s, reply_error=reply_error)
    calls, client = [], None
    async with serve(peer.handler, HOST, 0) as server:
        port = owned_port([sock.getsockname() for sock in server.sockets])
        try:
            client = await connect_owned(sdk, HOST, port)
            if not await client.is_connected():
                raise RuntimeError('local SDK connection not established')
            reads = []
            for index in range(read_count):
                method = READ_METHODS[index % len(READ_METHODS)]
                task = asyncio.create_task(_sdk_call(getattr(client, method)))
                calls.append(task)
                reads.append((method, task))
            await asyncio.wait_for(peer.reads_pending.wait(), 2)
            if len(peer.records) != read_count or any('send_started_ns' in row for row in peer.records):
                raise RuntimeError('controlled pending read boundary was not established')
            stop_task = asyncio.create_task(_sdk_call(client.stop_move))
            calls.append(stop_task)
            stop = await asyncio.wait_for(asyncio.shield(stop_task), 3)
            read_results = [{'method': method, **await asyncio.wait_for(asyncio.shield(task), 3)}
                            for method, task in reads]
            validate_read_results(read_results)
            if stop['outcome'] != ('error' if reply_error else 'returned'):
                raise RuntimeError('unexpected stop outcome')
            if reply_error and ('-32001' not in stop.get('error', '') or
                                'simulated stop failure' not in stop.get('error', '')):
                raise RuntimeError('native RPC error not propagated')
            await peer.close()
            await asyncio.wait_for(client.wait_disconnect(), 2)
            if await client.is_connected() or peer.errors or peer._tasks or peer._connections:
                raise RuntimeError('local cleanup/protocol verification failed')
            stops = [row for row in peer.records if row['method'] == 'stop_move']
            if len(stops) != 1 or len(peer.records) != read_count + 1:
                raise RuntimeError('unexpected RPC count')
            rpc = stops[0]
            read_rpc = [row for row in peer.records if row['method'] != 'stop_move']
            return {
                'case': name, 'read_count': read_count, 'listener': {'host': HOST, 'port': port},
                'requested_stop_delay_ms': delay_s * 1000,
                'requested_read_delay_ms': read_delay_s * 1000,
                'sdk_stop': stop, 'sdk_reads': read_results, 'server_rpc_records': peer.records,
                'stop_sdk_elapsed_ms': (stop['completed_ns'] - stop['started_ns']) / 1e6,
                'stop_call_to_peer_receipt_ms': (rpc['received_ns'] - stop['started_ns']) / 1e6,
                'peer_stop_reply_wait_ms': (rpc['send_started_ns'] - rpc['received_ns']) / 1e6,
                'peer_send_start_to_sdk_completion_ms': (stop['completed_ns'] - rpc['send_started_ns']) / 1e6,
                'read_replies_sent_before_stop_reply': sum(row['send_started_ns'] < rpc['send_started_ns'] for row in read_rpc),
                'sdk_reads_completed_before_stop': sum(row['completed_ns'] < stop['completed_ns'] for row in read_results),
                'local_connection_closed': True,
            }
        finally:
            await peer.close()
            for task in calls:
                if not task.done():
                    task.cancel()
            await asyncio.gather(*calls, return_exceptions=True)


async def run_matrix(repeats):
    if type(repeats) is not int or not 1 <= repeats <= 5:
        raise ValueError('repeats must be 1..5')
    sdk, identity = _sdk_identity()
    if not identity['matches_field_binary']:
        raise RuntimeError('native binary does not match the field SDK')
    sdk.init()
    results = []
    for _ in range(repeats):
        for case in CASES:
            results.append(await asyncio.wait_for(one_case(sdk, *case), 8))
    summary = {}
    for name, *_ in CASES:
        rows = [row for row in results if row['case'] == name]
        summary[name] = {'runs': len(rows)}
        for key in ('stop_sdk_elapsed_ms', 'stop_call_to_peer_receipt_ms',
                    'peer_stop_reply_wait_ms', 'peer_send_start_to_sdk_completion_ms'):
            values = [row[key] for row in rows]
            summary[name][key] = {'min': min(values), 'median': statistics.median(values), 'max': max(values)}
        for key in ('read_replies_sent_before_stop_reply', 'sdk_reads_completed_before_stop'):
            summary[name][key] = [row[key] for row in rows]
    return {'schema': 1, 'offline_only': True, 'hardware_stop_verified': False,
            'application_stop_path_exercised': False, 'real_robot_address_used': False,
            'clock': 'time.perf_counter_ns', 'clock_scope': 'one local process',
            'controlled_backlog': True, 'sdk_identity': identity,
            'repeats': repeats, 'results': results, 'summary': summary}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, choices=range(1, 6), default=3)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error('output exists; choose a new evidence path')
    result = asyncio.run(run_matrix(args.repeats))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'sdk_identity': result['sdk_identity'], 'summary': result['summary']}, indent=2))


if __name__ == '__main__':
    main()
