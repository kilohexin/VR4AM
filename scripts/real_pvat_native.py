"""Pure finite Lua program builder and independent trace replay. No transport."""
from dataclasses import replace
from hashlib import sha256
from pathlib import Path
import math
from scipy.spatial.transform import Rotation

from app.commissioning.real_cartesian_pvat import FinitePvatContract,FinitePvatWindow,FinitePvatFeedback
from app.control.cartesian_pvat_reference import JointReference,MdhReferenceModel
from app.robots.lebai_codec import pose_from_lebai
from app.robots.base import BackendCommandError
from app.schemas.messages import Pose

ROOT=Path(__file__).resolve().parents[1]


def lua(value):
    if isinstance(value,(tuple,list)):return '{'+','.join(lua(v) for v in value)+'}'
    if type(value) is not float:raise ValueError('native_finite_literal')
    return repr(value)


def build_program(contract):
    if type(contract) is not FinitePvatContract:raise ValueError('native_finite_contract')
    contract=replace(contract)  # Full validation even for a caller-supplied instance.
    geometry=(ROOT/'scripts/cartesian_lm3_geometry.lua').read_text(encoding='utf-8')
    body=(ROOT/'scripts/real_pvat_finite.lua').read_text(encoding='utf-8')
    pose=Pose(p=contract.model_tool.translation_m,q=contract.model_tool.rotation_xyzw)
    from scipy.spatial.transform import Rotation
    rotation=tuple(float(x) for x in Rotation.from_quat(pose.q).as_matrix().ravel())
    sender=('local function send_pvat(item)\n lebai:move_pvat(item.p,item.v,item.a,.08)\nend\n')
    if contract.pvat_api=='lua':
        sender=('local function joint_pose(values)\n local q={}\n for i=1,6 do q["j"..i]=values[i] end\n return q\nend\n'
            'local function send_pvat(item)\n assert(move_pvat(joint_pose(item.p),joint_pose(item.v),joint_pose(item.a),.08)==true,"pvat_api_rejected")\nend\n')
    return ('program_begin(0)\ndisable_auto_sync()\n-- contract '+contract.sha256+'\n'
        +'local geometry=(function()\n'+geometry+'\nend)()\n'
        +'local model=geometry.with_tool('+lua(pose.p)+','+lua(pose.q)+')\n'
        +'local tool_pose={p='+lua(pose.p)+',r='+lua(rotation)+'}\n'
        +'local origin='+lua(contract.origin.q)+'\n'
        +'local first_reply_ms='+str(contract.first_reply_deadline_ns//1_000_000)+'\n'
        +'local lower='+lua(contract.joint_lower)+';local upper='+lua(contract.joint_upper)+'\n'
        +'local box_lower='+lua(contract.workspace_lower)+';local box_upper='+lua(contract.workspace_upper)+'\n'
        +'local points={'+','.join('{p='+lua(s.end.q)+',v='+lua(s.end.v)+',a='+lua(s.end.a)+'}' for s in contract.points)+'}\n'
        +sender+body+'\nprogram_end(0)\n')


def analyze(contract,rows):
    """Replay capture/entry/return order; native DONE alone cannot pass."""
    result=dict(sampled_passed=False,hardware_verified=False,motion_unknown=True,stop_unverified=True,
        program_sha256=sha256(build_program(contract).encode()).hexdigest())
    entries=[r for r in rows if r and r[0]=='RP_ENTER'];returns=[r for r in rows if r and r[0]=='RP_RETURN']
    stops=[r for r in rows if r and r[0]=='RP_STOP_ENTER']
    done=[r for r in rows if r and r[0]=='RP_DONE']
    # Failed/incomplete native traces cannot reconstruct when a monitor
    # revoked a pending write. Conservatively retain unknown for every
    # entered failed trial; only full successful replay can clear it.
    # Events are buffered in Lua: an empty/truncated stream does not prove
    # zero writes. Transport failure may occur before the buffer is flushed.
    result['motion_unknown']=True
    # Without exact complete valid evidence, preserve uncertainty regardless
    # of an exception stop return or apparent success marker.
    try:
        if len(done)!=1 or rows[-1] is not done[0] or len(done[0])!=4 or done[0][1:3]!=['true','nil'] or int(done[0][3])!=20:return result
        if len(entries)!=20 or len(returns)!=20 or stops:return result
        connection=object();window=FinitePvatWindow(contract,connection,0);feedback=None;initial=None
        model=MdhReferenceModel(contract.model_tool,contract.joint_lower,contract.joint_upper)
        tail=False;arrival=False
        for row in rows:
            if not row:raise ValueError('empty_row')
            kind=row[0]
            if kind=='RP_SAMPLE':
                if len(row)!=46:raise ValueError('sample_shape')
                capture,now=(int(row[i])*1_000_000 for i in (1,2));state=row[3]
                v=list(map(float,row[4:]));actual=JointReference(tuple(v[:6]),tuple(v[6:12]),tuple(v[12:18]))
                target=JointReference(tuple(v[18:24]),tuple(v[24:30]),tuple(v[30:36]))
                tcp=pose_from_lebai(dict(zip(('x','y','z','rx','ry','rz'),v[36:])))
                if feedback is None:
                    if initial is not None:raise ValueError('duplicate_initial')
                    expected=model.pose(actual.q)
                    if (not 0<=capture<=now or now-capture>=100_000_000
                            or any(x<lo or x>hi for x,lo,hi in zip(tcp.p,contract.workspace_lower,contract.workspace_upper))
                            or math.dist(tcp.p,expected.p)>.001
                            or (Rotation.from_quat(tcp.q).inv()*Rotation.from_quat(expected.q)).magnitude()>math.radians(.2)):
                        raise ValueError('initial_feedback_invalid')
                    initial=(actual,target,state,capture,now)
                else:feedback.observe(connection,0,actual,target,tcp,state,capture,now)
            elif kind=='RP_ADMIT':
                if feedback or initial is None or len(row)!=3:raise ValueError('admission_shape')
                actual,target,state,capture,received=initial
                if capture!=int(row[2])*1_000_000:raise ValueError('capture_changed')
                if received>int(row[1])*1_000_000:raise ValueError('initial_receipt_in_future')
                window.admit(connection,0,contract.sha256,contract.tool_sha256,actual,target,state,capture,int(row[1])*1_000_000)
                feedback=FinitePvatFeedback(window,model)
            elif kind=='RP_ENTER':window.enter(connection,0,int(row[1]),int(row[2])*1_000_000)
            elif kind=='RP_RETURN':window.returned(int(row[1]),int(row[2])*1_000_000)
            elif kind=='RP_ARRIVAL':
                if arrival or not feedback or feedback.arrival_ns!=int(row[1])*1_000_000:raise ValueError('arrival_mismatch')
                arrival=True
            elif kind=='RP_TAIL':
                if tail or not feedback or not feedback.sampled_passed or int(row[1])*1_000_000!=feedback.previous_ns:raise ValueError('tail_mismatch')
                tail=True
            elif kind!='RP_DONE':raise ValueError('unexpected_event')
        result.update(sampled_passed=bool(feedback and feedback.sampled_passed and tail and arrival),
            motion_unknown=window.motion_unknown,stop_unverified=window.stop_unverified,
            entered=window.entered,returned=window.returned_count,feedback_samples=feedback.samples if feedback else 0)
    except (ValueError,TypeError,IndexError,OverflowError,BackendCommandError) as error:result['invalid_evidence']=type(error).__name__
    return result
