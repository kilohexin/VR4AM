"""Immutable geometry from a hash-pinned saved readonly tool report.

No SDK, network, config edits, endpoint selection or motion authorization.
The source proves file identity only, not calibration or physical accuracy.
"""
from hashlib import sha256
import json
import math
from pathlib import Path
import re
import struct

from app.commissioning.native_jog_mdh import MdhTool
from app.robots.lebai_codec import pose_from_lebai

ZERO_TOOL=MdhTool((0.,0.,0.),(0.,0.,0.,1.))


def checked_tool(tool):
    if type(tool) is not MdhTool or math.hypot(*tool.translation_m)>=1.:
        raise ValueError('cartesian_tool_geometry')
    return tool


def tool_digest(tool):
    checked_tool(tool)
    return sha256(b'cartesian-tool-v1'+struct.pack('<7d',*tool.translation_m,*tool.rotation_xyzw)).hexdigest()


def load_tool_report(path,expected_sha256):
    if type(expected_sha256) is not str or re.fullmatch('[a-f0-9]{64}',expected_sha256) is None:
        raise ValueError('cartesian_tool_source_hash_required')
    raw=Path(path).read_bytes()
    if sha256(raw).hexdigest()!=expected_sha256:raise ValueError('cartesian_tool_source_changed')
    try:
        report=json.loads(raw)
        if (report.get('mode')!='readonly' or report.get('complete') is not True
                or report.get('config_unchanged') is not True or report.get('error_type')):
            raise ValueError
        reads=report['reads']
        if not isinstance(reads,list) or any(r.get('outcome')!='returned' for r in reads):raise ValueError
        tools=[r['raw'] for r in reads if r.get('method')=='get_tcp']
        if len(tools)!=1:raise ValueError
        tool=tools[0]
        for key in ('x','y','z','rx','ry','rz'):
            v=tool[key]
            if type(v) not in (int,float) or not math.isfinite(v):raise ValueError
        pose=pose_from_lebai(tool)
        return checked_tool(MdhTool(pose.p,pose.q))
    except (ValueError,TypeError,KeyError,AttributeError) as error:
        raise ValueError('cartesian_tool_report_rejected') from error


def tool_arguments(parser,args):
    if bool(args.tool_report)!=bool(args.tool_report_sha256):
        parser.error('--tool-report and --tool-report-sha256 are required together')
    return ZERO_TOOL if args.tool_report is None else load_tool_report(args.tool_report,args.tool_report_sha256)
