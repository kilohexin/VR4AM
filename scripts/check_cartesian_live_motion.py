"""One bounded actual virtual PVAT trial of the live host chain.

Preview default. Pinned isolated official Docker only. Requires existing
IDLE/quiet nonsingular origin: NO startup/preparation or gripper. <=128
SDK PVAT writes, one original exception stop maximum, no retries or real
address. Normal release/expiry executes the already owned finite tail.
"""
import argparse
import asyncio
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'scripts')]
from scripts import check_cartesian_live_host as live
from scripts.check_native_cartesian_motion import COMMON
from scripts.check_native_cartesian_reference import native_admission
from scripts.cartesian_signal_channel import CartesianSignalChannel
from scripts.cartesian_native_pipe import arguments
from app.control.cartesian_pvat_reference import JointReference
import hashlib
import struct
import secrets


def program(origin,session,model_hash):
    source=live.program(origin,session,model_hash)
    helpers='local report\n'+COMMON.replace('timestamp()','cartesian_now()').replace("print('FINITE_ABORT'","report('FINITE_ABORT'")
    helpers+='\nlocal sdk_result=(function()\n'+(ROOT/'scripts/pvat_sdk_result.lua').read_text(encoding='utf-8')+'\nend)()\n'
    logger='''local journal={};local journal_index=1;local journal_end=false
report=function(...) assert(#journal<2048,'live_journal_budget');journal[#journal+1]=table.pack(...) end
local journal_worker=async_task(function()
    while not journal_end or journal_index<=#journal do
        local row=journal[journal_index]
        if row then print(table.unpack(row,1,row.n));journal_index=journal_index+1 else rt_sleep(1) end
    end
end)'''
    initial=(ROOT/'scripts/native_cartesian_live_motion_support.lua').read_text(encoding='utf-8')
    precheck='''tool_check();assert(lebai:get_velocity_factor()==100,'live_actual_velocity_factor')
    assert(lebai:get_robot_state()=='IDLE','live_actual_requires_idle')
    actual_base_rotation=actual_rotation(initial.actual_tcp_pose)
    actual_envelope({start=origin,next=origin},0)'''
    emit='''if segment then
                    assert(actual_entries<128,'live_actual_point_budget')
                    actual_envelope(segment)
                    entered=true;actual_entries=actual_entries+1
                    report('CART_ACTUAL_ENTER',actual_entries,clock(),owner.head_tick,mode)
                    sdk_result.call_void(function()
                        return lebai:move_pvat(segment.next.q,segment.next.v,segment.next.a,.08)
                    end)
                    actual_returns=actual_returns+1
                    report('CART_ACTUAL_RETURNED',actual_returns,clock(),owner.head_tick,mode)
                end'''
    workers='''local actual_monitor=worker(function()
        while not finish do local _,k=read(.3);actual_observe(k);rt_sleep(20) end
    end)'''
    quiet='''local last=quiet(2500,200,.3,actual_observe)
    for i=1,6 do assert(math.abs(last.actual_joint_pose[i]-owner.head.q[i])<=.002,'live_actual_terminal') end
    report('CART_ACTUAL_QUIET',actual_entries,actual_returns,actual_samples,actual_max_drift,actual_max_error)'''
    replacements={
        '-- LIVE_EXTRA_HELPERS':helpers,
        'local ok,err=pcall(function()':'local main_task=async_task(function()\nlocal ok,err=pcall(function()',
        "local function report(tag,...)\n    print(tag,...)\nend":logger,
        '-- LIVE_EXTRA_INITIAL':initial,
        '-- LIVE_EXTRA_PRECHECK':precheck,
        '-- LIVE_EXTRA_EMIT':emit,
        '-- LIVE_EXTRA_WORKERS':workers,
        'wait_all(receiver,sender,clock_service)':'wait_all(receiver,sender,clock_service,actual_monitor)',
        '-- LIVE_EXTRA_QUIET':quiet,
        '''local after=lebai:get_kin_data()
    for i=1,6 do assert(after.actual_joint_pose[i]==initial.actual_joint_pose[i]
        and math.abs(after.actual_joint_speed[i])<1e-9,'dry_joint_changed') end''':'',
        '-- LIVE_EXTRA_ABORT':"if not ok and entered then abort() end\nreport('CART_ACTUAL_STATUS',actual_entries,actual_returns)",
        '-- LIVE_EXTRA_LOG_END':'journal_end=true\nend)\nwait_all(main_task,journal_worker)',
    }
    for marker,value in replacements.items():
        if source.count(marker)!=1:raise ValueError('live_motion_template')
        source=source.replace(marker,value)
    return source


def analyze(report):
    rows=report.get('raw_output',[])
    entered=[r for r in rows if r[0]=='CART_ACTUAL_ENTER'];returned=[r for r in rows if r[0]=='CART_ACTUAL_RETURNED']
    quiet=[r for r in rows if r[0]=='CART_ACTUAL_QUIET'];status=[r for r in rows if r[0]=='CART_ACTUAL_STATUS']
    aborts=[r for r in rows if r[0]=='FINITE_ABORT']
    unknown=bool(len(entered)!=len(returned) or len(status)!=1)
    try:
        consistent=(1<=len(entered)<=128 and len(returned)==len(entered)
            and [int(r[1]) for r in entered]==list(range(1,len(entered)+1))
            and [int(r[1]) for r in returned]==list(range(1,len(returned)+1))
            and len(status)==1 and list(map(int,status[0][1:]))==[len(entered),len(returned)])
        measured=(len(quiet)==1 and len(quiet[0])==6
            and int(quiet[0][1])==len(entered) and int(quiet[0][2])==len(returned)
            and int(quiet[0][3])>=30 and 0<=float(quiet[0][4])<=.03 and 0<=float(quiet[0][5])<=.03)
    except (ValueError,IndexError):consistent=measured=False;unknown=True
    confirmed=measured or len(aborts)==1 and aborts[0][1:]==['true','true']
    stop_unverified=bool(unknown or report.get('unknown') or len(aborts)>1
        or any(row[1:]!=['true','true'] for row in aborts) or entered and not confirmed)
    report.update(passed=bool(report.get('passed') and consistent and measured and not aborts and not stop_unverified),
        motion_calls=len(entered),motion_returns=len(returned),unknown_motion_write=unknown,
        exception_stops=len(aborts),stop_unverified=stop_unverified,native_interpolation_verified=False,
        hardware_verified=False,product_admission_complete=False)
    return report


async def preflight(admission,*,source_path=None):
    """Same actual executable, ZERO goal packets, explicit final zero ledger."""
    channel=CartesianSignalChannel(admission);proc=None;reader=None;raw=[]
    try:
        await channel.open()
        if await channel.call('get_robot_state')!='IDLE':raise ValueError('live_preflight_requires_idle')
        data=await channel.call('get_kin_data');origin=JointReference(tuple(data['actual_joint_pose']))
        session=secrets.token_hex(16);source=program(origin,session,live.model_fingerprint())
        if source_path is not None:
            source_path.parent.mkdir(parents=True,exist_ok=True);source_path.write_text(source,encoding='utf-8')
        proc=await asyncio.create_subprocess_exec(*arguments(admission,'CART_DONE'),
            stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.PIPE)
        proc.stdin.write(source.encode());await proc.stdin.drain();proc.stdin.close()
        ready=asyncio.get_running_loop().create_future();started=asyncio.get_running_loop().create_future()
        async def read():
            while line:=await proc.stdout.readline():
                row=line.decode('utf-8').strip().split('\t');raw.append(row)
                if row[0]=='CART_READY' and not ready.done():ready.set_result(int(row[1]))
                if row[0]=='CART_HEAD' and not started.done():started.set_result(True)
                if row[0]=='CART_DONE':return
        reader=asyncio.create_task(read());await asyncio.wait_for(ready,3)
        await channel.call('set_signal',250,1)
        async with asyncio.timeout(.2):
            while True:
                pair=await channel.call('get_signals',251,2)
                if pair[1]==1:break
                await asyncio.sleep(.001)
        words=[*struct.unpack('<4i',bytes.fromhex(session)),1]
        await channel.call('set_signals',241,words)
        await channel.call('set_signal',240,(pair[0]+1)*1024+1)
        await asyncio.wait_for(started,1)
        await channel.call('set_signal',240,0)
        await asyncio.wait_for(reader,7);code=await proc.wait()
        status=[r for r in raw if r[0]=='CART_ACTUAL_STATUS'];done=[r for r in raw if r[0]=='CART_DONE']
        calls=[r for r in raw if r[0] in ('CART_ACTUAL_ENTER','FINITE_ABORT')]
        passed=bool(code==0 and status==[['CART_ACTUAL_STATUS','0','0']]
            and done==[['CART_DONE','true','nil']] and not calls and not channel.unknown)
        return dict(passed=passed,preflight_only=True,motion_calls=0,goal_packets=0,raw_output=raw,
            unknown=channel.unknown,program_sha256=hashlib.sha256(source.encode()).hexdigest(),
            product_admission_complete=False,hardware_verified=False)
    except Exception as error:
        return dict(passed=False,preflight_only=True,error=type(error).__name__+':'+str(error),
            raw_output=raw,unknown=channel.unknown,goal_packets=0,product_admission_complete=False)
    finally:
        await channel.close()
        if reader and not reader.done():
            try:await asyncio.wait_for(reader,7)
            except Exception:pass
        if proc and proc.returncode is None:proc.terminate();await proc.wait()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true')
    p.add_argument('--expire',action='store_true');p.add_argument('--preflight',action='store_true')
    args=p.parse_args();admission=native_admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=admission,pvat_max=0 if args.preflight else 128,exception_stop_max=1,
            preparation=0,gripper=0,hardware_verified=False)));return 0
    def frozen_program(origin,session,model_hash):
        source=program(origin,session,model_hash)
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.with_suffix('.lua').write_text(source,encoding='utf-8')
        return source
    report=(asyncio.run(preflight(admission,source_path=args.output.with_suffix('.lua'))) if args.preflight else
        analyze(asyncio.run(live.execute(admission,expire=args.expire,_source_factory=frozen_program))))
    report['admission']=admission;args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    summary={k:v for k,v in report.items() if k not in ('raw_output','heads','admission','statistics')}
    summary['statistics']={k:v for k,v in report.get('statistics',{}).items() if k not in ('math_ms','publications')}
    print(json.dumps(summary));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
