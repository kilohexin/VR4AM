"""Finite speedl SDK protocol probe, exclusively against its owned IPv4 peer.

No real target, port, configuration or arbitrary RPC can be supplied by CLI.
Protocol: SDK v0.3.13 rpc/motion.rs + serde/posture.rs, lebai-proto 3.4.3.
"""
import argparse
import asyncio
import copy
import hashlib
import importlib
import importlib.metadata
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'backend'))

from app.commissioning.native_jog_candidate import NativeJogCandidateAdapter
from app.commissioning.native_jog_packet import NativeJogLimits, plan_jog
from app.commissioning.qpc_event_loop import new_qpc_loop
from app.robots.base import BackendCommandError, StopReason
from scripts.application_stop_loopback_probe import fixture_result, loopback_settings
from scripts.sdk_loopback_stop_probe import LoopbackPeer, _sdk_call

HOST = '127.0.0.1'
SDK_BINARY = ROOT/'.venv/Lib/site-packages/lebai_sdk/lebai_sdk.pyd'
SDK_SHA256 = 'ad2a68619b25d627593bd5f13f83f17784e7a2c10aa9615db8bd1b681f9fd03b'
CASES = ('immediate','speedl_late_100ms','invalid_id','rpc_error','stop_late_250ms')
READS = frozenset({'get_robot_state','get_estop_reason','get_kin_data','get_tcp',
    'get_claw','get_running_motion'})
LIMITS = NativeJogLimits(.005,.02,(.1,-.3,.1),(.5,.3,.6),(.001,)*3)
CONFIG_SHA = hashlib.sha256(b'native-jog-loopback-synthetic-v1').hexdigest()


def owned_port(addresses):
    if len(addresses) != 1 or len(addresses[0]) != 2:
        raise ValueError('one owned IPv4 listener required')
    host,port = addresses[0]
    if host != HOST or type(port) is not int or not 0 < port <= 65535:
        raise ValueError('nonlocal or invalid listener')
    return port


def _expected_speed_request():
    def pose(z):
        return dict(position=dict(x=0.,y=0.,z=z),
            rotation=dict(euler_zyx=dict(x=0.,y=0.,z=0.)))
    return dict(speed=pose(.005),param=dict(acc=.02,time=.08,constrained=True),
        frame=dict(position_kind='CUSTOM',rotation_kind='CUSTOM',**pose(0.)))


def _same_wire(value, expected):
    if isinstance(expected,dict):
        return (type(value) is dict and value.keys()==expected.keys()
            and all(_same_wire(value[k],v) for k,v in expected.items()))
    if type(expected) is bool:
        return type(value) is bool and value==expected
    if type(expected) in (int,float):
        return type(value) in (int,float) and math.isfinite(value) and value==expected
    return type(value) is type(expected) and value==expected


def _unique_object(pairs):
    result={}
    for key,value in pairs:
        if key in result:raise ValueError('duplicate JSON key')
        result[key]=value
    return result


def parse_request(raw):
    r=json.loads(raw,object_pairs_hook=_unique_object)
    if (type(r) is not dict or set(r)!={'jsonrpc','id','method','params'}
            or r['jsonrpc']!='2.0' or type(r['id']) not in (int,str)
            or type(r['method']) is not str or type(r['params']) is not list or len(r['params'])!=1):
        raise ValueError('unexpected loopback RPC request')
    if r['method']=='speed_linear':
        valid=_same_wire(r['params'][0],_expected_speed_request())
    else:
        valid=r['method'] in READS|{'stop_move'} and r['params']==[{}]
    if not valid:raise ValueError('unexpected loopback RPC method/parameters')
    return r


class NativeJogPeer(LoopbackPeer):
    def __init__(self,case):
        if case not in CASES:raise ValueError('unknown case')
        super().__init__()
        self.case=case

    async def handler(self,connection):
        self._connections.add(connection)
        try:
            async for raw in connection:
                try:
                    r=parse_request(raw)
                    key=(type(r['id']),r['id'])
                    if key in self._ids:raise ValueError('duplicate RPC id')
                    self._ids.add(key)
                except ValueError as error:
                    self.errors.append(str(error))
                    await connection.close(code=1008,reason='unexpected probe request')
                    return
                record={**copy.deepcopy(r),'received_ns':time.perf_counter_ns()}
                self.records.append(record)
                task=asyncio.create_task(self._reply(connection,record))
                self._tasks.add(task)
                task.add_done_callback(self._reply_done)
        finally:
            self._connections.discard(connection)

    async def _reply(self,connection,record):
        method=record['method']
        if method=='speed_linear' and self.case=='speedl_late_100ms':await asyncio.sleep(.1)
        if method=='stop_move' and self.case=='stop_late_250ms':await asyncio.sleep(.25)
        reply=dict(jsonrpc='2.0',id=record['id'])
        if method=='speed_linear' and self.case=='rpc_error':
            reply['error']=dict(code=-32001,message='synthetic speedl failure')
        else:
            reply['result']=(dict(id=0 if self.case=='invalid_id' else 7)
                if method=='speed_linear' else fixture_result(method))
        record['send_started_ns']=time.perf_counter_ns()
        await connection.send(json.dumps(reply))
        record['send_completed_ns']=time.perf_counter_ns()


def _sdk_identity():
    binary=SDK_BINARY.resolve(strict=True)
    digest=hashlib.sha256(binary.read_bytes()).hexdigest()
    if digest!=SDK_SHA256:raise ValueError('SDK binary identity mismatch before import')
    sdk=importlib.import_module('lebai_sdk')
    loaded=Path(importlib.import_module('lebai_sdk.lebai_sdk').__file__).resolve(strict=True)
    if loaded!=binary or hashlib.sha256(loaded.read_bytes()).hexdigest()!=digest:
        raise ValueError('SDK loaded path/hash mismatch')
    try:distribution=importlib.metadata.version('lebai-sdk-asyncio')
    except importlib.metadata.PackageNotFoundError:distribution=None
    return sdk,dict(binary_path=str(binary),binary_sha256=digest,
        distribution_version=distribution,native_version=sdk.version())


async def _one_case(sdk,case):
    from websockets.asyncio.server import serve
    peer=NativeJogPeer(case)
    adapter=client=None
    events=[]
    async def record(event,stamp):events.append(dict(event=copy.deepcopy(event),stamp_ns=stamp))
    async with serve(peer.handler,HOST,0) as server:
        port=owned_port([s.getsockname() for s in server.sockets])
        async def factory(host):
            nonlocal client
            if host!=HOST:raise ValueError('nonlocal target')
            # The closure captures only the verified listener's ephemeral port.
            client=await asyncio.wait_for(sdk.connect(HOST,port,{'request_timeout':2}),3)
            return client
        try:
            adapter=NativeJogCandidateAdapter(loopback_settings(),limits=LIMITS,
                config_sha256=CONFIG_SHA,client_factory=factory,clock=time.perf_counter_ns,
                sleep=asyncio.sleep,event_callback=record)
            await adapter.connect()
            if not (await adapter.preflight()).ready:raise RuntimeError('synthetic preflight failed')
            now=time.perf_counter_ns()
            packet=plan_jog((0,0,1),1,limits=LIMITS,actual_p=adapter._snapshot.actual_tcp.p,
                snapshot_ns=adapter._snapshot.captured_ns,now_ns=now,generation=adapter._pump.generation,
                sequence=1,input_ns=now,input_expires_ns=now+100_000_000,duration_s=.08,config_sha256=CONFIG_SHA)
            jog=await _sdk_call(lambda:adapter.submit_jog(packet,authorize=lambda:None))
            await asyncio.sleep(max(0,.08-(time.perf_counter_ns()-now)/1e9))
            stop=await _sdk_call(lambda:adapter.stop(StopReason.GRIP_RELEASED))
            transaction=adapter._stop_transaction
            # Observe late completion locally without restoring authority.
            for req in transaction.requests.values():
                try:await asyncio.wait_for(asyncio.shield(req.task),2)
                except Exception:pass
            if peer._tasks:await asyncio.wait(tuple(peer._tasks),timeout=2)
            before=adapter._motion_calls.snapshot()
            preflight=await adapter.preflight()
            result=dict(case=case,jog=jog,stop=stop,confirmed=transaction.confirmed,
                latched_fault=adapter._latched_fault,preflight_ready=preflight.ready,
                ownership=before,native_entries=adapter.native_jog_entries,
                listener=dict(host=HOST,port=port),rpc_records=peer.records,events=events)
            expected_ok=case=='immediate'
            if (result['confirmed']!=expected_ok or (expected_ok and
                    (jog.get('value')!=7 or stop['outcome']!='returned' or result['latched_fault'] is not None))
                    or (not expected_ok and (result['latched_fault']!='stop_unverified' or preflight.ready))):
                raise RuntimeError('unexpected case ownership/stop outcome: '+case)
            if (case in {'speedl_late_100ms','invalid_id','rpc_error'}
                    and before['unknown_methods']!=['speedl']):
                raise RuntimeError('unknown speedl result was cleared')
            try:await adapter.disconnect()
            except BackendCommandError:pass
            result['ownership_after_disconnect']=adapter._motion_calls.snapshot()
            adapter=None
            await peer.close()
            await asyncio.wait_for(client.wait_disconnect(),2)
            result['local_connection_closed']=not await client.is_connected()
            if (not result['local_connection_closed'] or peer.errors
                    or sum(r['method']=='speed_linear' for r in peer.records)!=1
                    or sum(r['method']=='stop_move' for r in peer.records)!=1):
                raise RuntimeError('protocol, close or single-write count failed')
            return result
        finally:
            if adapter is not None:
                try:await adapter.disconnect()
                except BackendCommandError:pass
            await peer.close()
            if client is not None:
                await asyncio.wait_for(client.wait_disconnect(),2)


async def run_matrix(repeats):
    if type(repeats) is not int or not 1<=repeats<=3:raise ValueError('repeats must be 1..3')
    if getattr(asyncio.get_running_loop(),'vr4arm_timing_clock',None)!='time.perf_counter_ns':
        raise ValueError('isolated QPC event loop required')
    sdk,identity=_sdk_identity()
    sdk.init()
    cases=[]
    for _ in range(repeats):
        for name in CASES:cases.append(await _one_case(sdk,name))
    return dict(offline_only=True,hardware_verified=False,real_robot_connections=0,
        continuous_mode='native_jog_joint_bounds_unverified',sdk=identity,cases=cases,
        clock='time.perf_counter_ns',config_sha256=CONFIG_SHA,
        limits=vars(LIMITS),nominal_box_only=True)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--repeats',type=int,choices=(1,2,3),default=1)
    options=parser.parse_args(argv)
    try:
        # Reserve before any import/init/connection; exclusive creation prevents
        # evidence replacement even if another process races this entry.
        options.output.parent.mkdir(parents=True,exist_ok=True)
        with options.output.open('x',encoding='utf-8') as stream:
            try:
                with asyncio.Runner(loop_factory=new_qpc_loop) as runner:
                    result=runner.run(run_matrix(options.repeats))
            except Exception as error:
                json.dump(dict(offline_only=True,hardware_verified=False,error=str(error)),stream,indent=2)
                raise
            json.dump(result,stream,indent=2,allow_nan=False)
        print(json.dumps(dict(output=str(options.output),cases=len(result['cases']),hardware_verified=False)))
        return 0
    except (OSError,ValueError,RuntimeError) as error:
        parser.error(str(error))


if __name__=='__main__':raise SystemExit(main())
