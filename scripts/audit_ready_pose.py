"""Compare a saved ready-pose capture with readonly configuration, entirely offline.

No SDK, connection table, sockets, robot adapter, FK/IK or configuration writes.
A compatible historical capture never authorizes or certifies hardware motion.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'backend'))
from app.config import Settings, TELEOP_SINGULARITY_GUARD_RAD
from app.robots.lebai_codec import pose_from_lebai

FIELDS = ('x','y','z','rz','ry','rx')
METHODS = ['connect']+['is_connected','get_robot_state','get_estop_reason',
    'get_kin_data','get_tcp','get_robot_state']*2


def _vector(value):
    if (not isinstance(value,list) or len(value)!=6
            or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
        raise ValueError('invalid_capture_joint_vector')
    return value


def _pose(value):
    if (not isinstance(value,dict) or any(k not in value or type(value[k]) not in (int,float)
            or not math.isfinite(value[k]) for k in FIELDS)):
        raise ValueError('invalid_capture_pose')
    return value


def _error(actual,expected):
    a,b=pose_from_lebai(actual),pose_from_lebai(expected)
    return dict(position_m=math.dist(a.p,b.p),rotation_deg=math.degrees(
        (Rotation.from_quat(a.q)*Rotation.from_quat(b.q).inv()).magnitude()))


def assess(record,settings):
    if settings.mode!='readonly':raise ValueError('readonly_config_required')
    try:
        if (record['complete'] is not True or record['runtime']!='LIVE_READONLY'
                or record['motion_authorized'] is not False or record.get('error') is not None
                or record['pending_remote_result_unknown'] is not False):
            raise ValueError('invalid_capture_status')
        calls=record['calls']
        if ([c['method'] for c in calls]!=METHODS
                or any(c['outcome']!='returned' or c['entered'] is not True
                    or c['remote_result_unknown'] is not False for c in calls)):
            raise ValueError('invalid_capture_calls')
        initial,final=record['initial'],record['final']
        for sample in (initial,final):
            _vector(sample['q_rad'])
            if (sample['state'] not in ('IDLE','PAUSED')
                    or any(v!=0. for key in ('qd_radps','qdd_radps2') for v in _vector(sample[key]))):
                raise ValueError('invalid_capture_observation')
            for key in ('actual_tcp_pose','tool_tcp_setting'):_pose(sample[key])
        if (initial['state']!=final['state']
                or max(abs(a-b) for a,b in zip(initial['q_rad'],final['q_rad']))>1e-6
                or any(abs(initial[key][axis]-final[key][axis])>1e-6
                    for key in ('actual_tcp_pose','tool_tcp_setting') for axis in FIELDS)):
            raise ValueError('capture_observation_changed')
        selected=record['operator_selected_ready_pose'];source=selected['source']
        digest=hashlib.sha256(json.dumps(dict(initial=initial,final=final),sort_keys=True,
            allow_nan=False,separators=(',',':')).encode()).hexdigest()
        if (source['target_ip']!=settings.ip or source['telemetry_sha256']!=digest
                or selected['raw_state']!=final['state']
                or any(selected[key]!=final[key] for key in ('q_rad','actual_tcp_pose','tool_tcp_setting'))):
            raise ValueError('capture_source_or_selected_pose_mismatch')
    except (KeyError,TypeError,IndexError,AttributeError) as error:
        raise ValueError('invalid_capture_structure') from error

    q=final['q_rad'];margin=settings.joint_limit_margin_rad
    tool_error=_error(final['tool_tcp_setting'],asdict(settings.expected_tcp))
    workspace={}
    for i,axis in enumerate(('x','y','z')):
        current=final['actual_tcp_pose'][axis];target=current+(.0002 if axis=='z' else 0.)
        lower,upper=settings.startup_tcp_min_m[i],settings.startup_tcp_max_m[i]
        workspace[axis]=dict(current_m=current,configured_min_m=lower,configured_max_m=upper,
            current_inside=lower<=current<=upper,computational_target_m=target,
            target_inside=lower<=target<=upper)
    checks=dict(recorded_idle=final['state']=='IDLE',
        joints_inside_limits=all(lo+margin<=v<=hi-margin
            for v,lo,hi in zip(q,settings.soft_joint_min_rad,settings.soft_joint_max_rad)),
        away_from_joint_singularity_guard=min(abs(q[2]),abs(q[4]))>TELEOP_SINGULARITY_GUARD_RAD,
        ready_pose_matches=max(abs(v-goal) for v,goal in zip(q,settings.teleop_ready_q))<=1e-6,
        tool_tcp_matches=tool_error['position_m']<=settings.tcp_position_tolerance_m
            and tool_error['rotation_deg']<=settings.tcp_rotation_tolerance_deg,
        startup_workspace=all(v['current_inside'] and v['target_inside'] for v in workspace.values()))
    return dict(schema=1,historical_capture_only=True,configuration_compatible=all(checks.values()),
        checks=checks,blocking_reasons=[k for k,v in checks.items() if not v],
        startup_workspace=workspace,tool_tcp_error=tool_error,
        computational_target=dict(axis='z',distance_m=.0002,executed=False),
        motion_authorized=False,hardware_ready=False,
        remaining_live_gates=['FK_IK_and_timing','motion_release_stop','empty_gripper'])


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--capture',type=Path,required=True)
    parser.add_argument('--capture-sha256',required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args(argv)
    config_raw=args.config.read_bytes()
    data=yaml.safe_load(config_raw)
    if (not isinstance(data,dict) or data.get('backend')!='lebai'
            or not isinstance(data.get('real_robot'),dict) or data['real_robot'].get('mode')!='readonly'):
        raise ValueError('readonly_config_required')
    raw=args.capture.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=args.capture_sha256:raise ValueError('capture_hash_mismatch')
    settings=Settings.load(args.config).lebai
    if args.config.read_bytes()!=config_raw:raise ValueError('configuration_changed_during_audit')
    report=assess(json.loads(raw),settings)
    report.update(capture_sha256=args.capture_sha256,config_sha256=hashlib.sha256(config_raw).hexdigest())
    text=json.dumps(report,allow_nan=False,indent=2)+'\n'
    if args.output:
        with args.output.open('x',encoding='utf-8') as stream:stream.write(text)
    else:print(text,end='')
    return 0  # A generated report can identify blocked configuration; inspect its explicit checks.


if __name__=='__main__':raise SystemExit(main())
