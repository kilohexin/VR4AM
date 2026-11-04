"""Read-only native timestamp diagnostic in a network-none official container."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from docker_pose_transport_probe import admission

PROGRAM='''program_begin(0)
disable_auto_sync()
local begin=timestamp();local last=begin;local count=0;local failed=false
local function check(stage)
    local now=timestamp();count=count+1
    if type(now)~='number' or now~=math.floor(now) or now<last then
        print('CLOCK_FAILURE',stage,now,last,begin,count);failed=true;return false
    end
    last=now;return true
end
for i=1,1000 do
    if not check('before_read') then break end
    lebai:get_signal(240)
    if not check('after_read') then break end
    rt_sleep(5)
    if not check('after_sleep') then break end
end
print('CLOCK_DONE',not failed,count,timestamp()-begin)
program_end(0)
'''


def analyze(raw,returncode):
    rows=[x.strip().split('\t') for x in raw.splitlines()]
    done=[x for x in rows if x[0]=='CLOCK_DONE']
    failures=[x for x in rows if x[0]=='CLOCK_FAILURE']
    result=dict(completed=len(done)==1,clock_valid=False,failures=failures,raw_output=raw,returncode=returncode)
    try:
        if len(done)==1 and len(done[0])==4:
            result.update(observed_calls=int(done[0][2]),elapsed_ms=int(done[0][3]))
            result['clock_valid']=(returncode==0 and done[0][1]=='true' and not failures
                and result['observed_calls']==3000 and result['elapsed_ms']>=5000)
    except ValueError:pass
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--container',required=True);p.add_argument('--run',action='store_true')
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    target=admission(args.container)
    if not args.run:
        print(json.dumps(dict(execute=False,admission=target,motion_calls=0,signal_writes=0,maximum_reads=1000)));return 0
    bridge='exec 3<>/dev/tcp/127.0.0.1/3010 || exit 2; cat >&3; while IFS= read -r -t 30 line <&3; do printf "%s\\n" "$line"; case "$line" in *CLOCK_DONE*) break;; esac; done'
    try:
        r=subprocess.run(['docker','exec','-i',target['container_id'],'bash','-c',bridge],input=PROGRAM,
            text=True,encoding='utf-8',capture_output=True,timeout=35)
        result=analyze(r.stdout,r.returncode)
        result['stderr']=r.stderr
    except subprocess.TimeoutExpired as e:
        raw=e.stdout or b''
        result=dict(completed=False,error='read_result_unknown',raw_output=raw.decode('utf-8') if isinstance(raw,bytes) else raw)
    result.update(admission=target,motion_calls=0,signal_writes=0)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('raw_output','admission')}))
    return 0 if result.get('clock_valid') else 1


if __name__=='__main__':raise SystemExit(main())
