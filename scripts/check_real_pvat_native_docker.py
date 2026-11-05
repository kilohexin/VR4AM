"""Explicit isolated-Docker trial of the finite native source; no real selector."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'backend'),str(ROOT/'scripts')]
from scripts.check_native_cartesian_reference import native_admission
from scripts.check_native_cartesian_motion import preparation_program,transport,finished
from scripts.real_pvat_native import build_program,analyze
from app.commissioning.native_jog_mdh import MdhTool
from app.commissioning.real_cartesian_pvat import build_contract
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel


def run(container,output):
    admitted=native_admission(container);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    result=dict(hardware_verified=False,simulated_only=True,sampled_passed=False)
    try:
        prep=transport(admitted,preparation_program(),'FINITE_PREP_DONE');result['preparation']=prep
        if not finished(prep,'FINITE_PREP_DONE') or 'FINITE_ABORT' in prep['raw_output']:
            return result
        rows=[r.strip().split('\t') for r in prep['raw_output'].splitlines() if r.startswith('FINITE_ORIGIN\t')]
        if len(rows)!=1 or len(rows[0])!=7:raise ValueError('docker_origin')
        start=JointReference(tuple(map(float,rows[0][1:])))
        model=MdhReferenceModel(MdhTool((0.,0.,0.),(0.,0.,0.,1.)),(-3.,)*6,(3.,)*6)
        p=model.pose(start.q).p
        contract=build_contract(model,start,workspace_lower=tuple(x-.01 for x in p),
            workspace_upper=tuple(x+.01 for x in p),config_sha256='0'*64,
            source_sha256=sha256(Path(__file__).read_bytes()).hexdigest(),tool_report_sha256='0'*64)
        source=build_program(contract);(output/'program.lua').write_bytes(source.encode())
        response=transport(admitted,source,'RP_DONE');result['motion']=response
        rows=[r.strip().split('\t') for r in response['raw_output'].splitlines()]
        result.update(analyze(contract,rows))
        if response.get('transport_unknown') or response.get('transport_returncode')!=0:
            result.update(sampled_passed=False,motion_unknown=True,stop_unverified=True)
        result['contract_sha256']=contract.sha256
        return result
    finally:(output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--container',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--run',action='store_true');a=p.parse_args()
    if not a.run:
        print(json.dumps(dict(admission=native_admission(a.container),execute=False,real_selector=False)));return 0
    result=run(a.container,a.output)
    print(json.dumps({k:v for k,v in result.items() if k not in ('preparation','motion')}))
    return 0 if result['sampled_passed'] else 2


if __name__=='__main__':raise SystemExit(main())
