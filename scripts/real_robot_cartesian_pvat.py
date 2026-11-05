"""Offline finite PVAT scope preview. NO transport or --execute option yet.

Local manifest pins saved readonly evidence and a private configuration.
Workspace bounds are explicit, never derived by recentering on the robot.
This entry cannot authorize a physical connection or create approval files.
"""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'backend')]
from app.commissioning.real_cartesian_pvat import build_contract, _vector
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.control.cartesian_tool_binding import load_tool_report


def source_identity():
    # Content and relative paths, including this entry, not a git revision
    # that would miss uncommitted edits. Excludes all private field material.
    files=sorted(p for p in (ROOT/'backend/app').rglob('*') if p.suffix in ('.py','.lua'))
    files.append(Path(__file__).resolve())
    manifest=[(p.relative_to(ROOT).as_posix(),sha256(p.read_bytes()).hexdigest()) for p in files]
    return sha256(json.dumps(manifest,separators=(',',':')).encode()).hexdigest()


def _read(path,digest):
    raw=Path(path).read_bytes()
    if sha256(raw).hexdigest()!=digest:raise ValueError('pvat_saved_source_changed')
    return raw


def preview_manifest(path):
    raw=Path(path).read_bytes();m=json.loads(raw)
    if m.get('schema')!=1:raise ValueError('pvat_manifest_schema')
    source_before=source_identity()
    report_raw=_read(m['tool_report'],m['tool_report_sha256'])
    tool=load_tool_report(m['tool_report'],m['tool_report_sha256'])
    config_raw=_read(m['config'],m['config_sha256'])
    cfg=yaml.safe_load(config_raw)['real_robot']
    margin=cfg['joint_limit_margin_rad']
    if type(margin) not in (int,float) or not math.isfinite(margin) or margin<=0:
        raise ValueError('pvat_joint_margin')
    lower=tuple(x+margin for x in _vector(cfg['soft_joint_min_rad'],6))
    upper=tuple(x-margin for x in _vector(cfg['soft_joint_max_rad'],6))
    report=json.loads(report_raw);index=m['kin_read_index']
    if type(index) is not int or not 0<=index<len(report['reads']):raise ValueError('pvat_saved_index')
    row=report['reads'][index]
    if row['method']!='get_kin_data' or row['outcome']!='returned':raise ValueError('pvat_saved_origin')
    k=row['raw']
    actual=JointReference(*(_vector(k['actual_joint_'+f],6) for f in ('pose','speed','acc')))
    target=JointReference(*(_vector(k['target_joint_'+f],6) for f in ('pose','speed','acc')))
    if actual!=target or any(actual.v+actual.a):raise ValueError('pvat_saved_origin_not_quiet')
    contract=build_contract(MdhReferenceModel(tool,lower,upper),actual,
        workspace_lower=m['workspace_lower'],workspace_upper=m['workspace_upper'],
        config_sha256=m['config_sha256'],source_sha256=source_before,tool_report_sha256=m['tool_report_sha256'])
    # Reject concurrent local edits during relatively expensive planning.
    _read(m['tool_report'],m['tool_report_sha256']);_read(m['config'],m['config_sha256'])
    if source_identity()!=source_before or Path(path).read_bytes()!=raw:raise ValueError('pvat_preview_identity_changed')
    return dict(schema=1,contract_sha256=contract.sha256,manifest_sha256=sha256(raw).hexdigest(),
        source_sha256=source_before,config_sha256=m['config_sha256'],tool_report_sha256=m['tool_report_sha256'],
        tool_sha256=contract.tool_sha256,points=len(contract.points),axis='base_z',distance_m=.002,duration_s=1.6,
        joint_lower=list(contract.joint_lower),joint_upper=list(contract.joint_upper),
        workspace_lower=list(contract.workspace_lower),workspace_upper=list(contract.workspace_upper),
        execute_available=False,motion_admitted=False,origin_is_saved_not_fresh=True,
        scope_requires_onsite_approval=True,sdk_connections=0,robot_writes=0,
        remaining=['native_bounded_sender_and_live_feedback','arrival_and_original_stop_verification',
            'isolated_native_runtime_trial','fresh_physical_scope_and_onsite_approval'])


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,help='hash-pinned local inputs; never an authorization')
    args=p.parse_args(argv)
    result=preview_manifest(args.manifest) if args.manifest else dict(execute_available=False,
        motion_admitted=False,sdk_connections=0,robot_writes=0,manifest_required=True)
    print(json.dumps(result,indent=2,allow_nan=False));return 0


if __name__=='__main__':raise SystemExit(main())
