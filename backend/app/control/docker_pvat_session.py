"""Opt-in Docker XR executor; normal release drains the bounded Lua stream."""
import asyncio
from pathlib import Path
from app.control.docker_pvat_input import DockerPvatInput,ExecutorLease,encode_pose
from app.control.finite_position_gate import PositionGate
from app.control.finite_position_types import PositionView,INPUT_NS


class DockerPvatSession:
    def __init__(self,adapter,binding,*,clock,recorder,authorize):
        from app.robots.finite_position_docker import DockerFinitePositionAdapter
        if type(adapter) is not DockerFinitePositionAdapter or not adapter._docker_native:
            raise ValueError('pvat_native_docker_required')
        self.adapter,self.binding,self.clock=adapter,binding,clock
        self.recorder,self.authorize=recorder,authorize
        self.gate=PositionGate(binding,clock=clock,diagnostic_limits=False)
        self.worker=None;self.pump=None;self.closed=False;self.phase='ready';self.reason=None
        self.steps=0;self.hold=False;self._withdraw=asyncio.Event()

    @property
    def busy(self):return self.worker is not None

    def check(self):
        self.authorize();self.adapter._position_health(self.binding)
        if self.closed:raise ValueError('pvat_session_closed')
        if getattr(self.recorder,'fatal_error',None) or getattr(self.recorder,'dropped_normal_events',0):
            raise ValueError('position_recording_unavailable')

    def offer(self,intent):
        self.check()
        if not intent.session_id.startswith('xr-finite-'):
            raise ValueError('pvat_xr_source_required')
        self.gate.offer(intent)
        if intent.kind=='release':
            self._withdraw.set()
            if not self.busy:self.hold=False;self.phase='ready';self.reason=None
        elif not self.hold and self.pump and not self.pump.closed and not self._withdraw.is_set():
            self._forward(intent)

    def _forward(self,intent):
        self.pump.offer(seq=intent.seq,captured_ns=intent.received_ns,action=(*intent.vector,intent.pitch,intent.spin))

    async def _run(self):
        a=self.adapter;proc=None;done=False;quiet=False;okay=False
        boundary=None;reference_rejected=False;abort_confirmed=False;known_failure=None
        try:
            from app.digital_twin.docker_runtime import verify_local_docker
            admitted=await asyncio.to_thread(verify_local_docker)
            self.check()
            if admitted!=a._docker_admission or a.position_busy or a._docker_entry_owner is not None:
                raise ValueError('pvat_admission_changed_or_busy')
            # Use only the native local simulator client admitted by the adapter.
            client=a._docker_ready_client
            if await asyncio.wait_for(client.get_signal(240),.1)!=0:raise ValueError('pvat_mailbox_occupied')
            self.check()
            if self.gate.released:
                self.phase='ready';self.reason=None;self.hold=False;return
            if not 0<=self.clock()-self.gate.latest.received_ns<INPUT_NS:
                self.phase='hold';self.reason='position_input_expired';self.hold=True;return
            a.position_busy=True;a._motion_accepted=a._motion_write_stop_required=True
            bridge='exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; while IFS= read -r -t 12 line <&3; do printf "%s\\n" "$line"; case "$line" in *LIVE_DONE*) break;; esac; done'
            program=(Path(__file__).resolve().parents[3]/'scripts/docker_pvat_live.lua').read_text(encoding='utf-8')
            program=program.replace('local vector_mailbox = false','local vector_mailbox = true')
            program=program.replace('local pose_mailbox = false','local pose_mailbox = true')
            program=program.replace('local tcp_speed = 0.005','local tcp_speed = 0.010')
            # Add the product's fixed workspace, in addition to the local box.
            lo,hi=a.settings.startup_tcp_min_m,a.settings.startup_tcp_max_m
            bounds='\n'.join(f"assert(k.actual_tcp_pose.{axis}>={lo[i]!r} and k.actual_tcp_pose.{axis}<={hi[i]!r},'product_workspace')" for i,axis in enumerate(('x','y','z')))
            joint_lo=tuple(x+a.settings.joint_limit_margin_rad for x in a.settings.soft_joint_min_rad)
            joint_hi=tuple(x-a.settings.joint_limit_margin_rad for x in a.settings.soft_joint_max_rad)
            bounds+='\n'+'\n'.join(f"assert(k.actual_joint_pose[{i+1}]>={joint_lo[i]!r} and k.actual_joint_pose[{i+1}]<={joint_hi[i]!r},'product_joint_limit')" for i in range(6))
            if program.count('-- PRODUCT_HARD_BOUNDS')!=1:raise ValueError('pvat_program_marker')
            soft=[]
            for i,name in enumerate(('x','y','z')):
                soft.append(f"if (k.actual_tcp_pose.{name}<={lo[i]+.006!r} and direction[{i+1}]<0) or (k.actual_tcp_pose.{name}>={hi[i]-.006!r} and direction[{i+1}]>0) then return 'workspace_boundary' end")
            for i in range(6):
                soft.append(f"if (k.actual_joint_pose[{i+1}]<={joint_lo[i]+.025!r} and goal[{i+1}]<0) or (k.actual_joint_pose[{i+1}]>={joint_hi[i]-.025!r} and goal[{i+1}]>0) then return 'joint_boundary' end")
            from scipy.spatial.transform import Rotation
            reference=Rotation.from_quat(a._position_pitch_reference).as_euler('xyz')
            fixed='{rx='+repr(float(reference[0]))+',ry='+repr(float(reference[1]))+',rz='+repr(float(reference[2]))+'}'
            soft.append('local reference='+fixed+'\nlocal angular=rotation_distance(k.actual_tcp_pose,reference)')
            for axis in (4,5):
                soft.append(f"if direction[{axis}] and direction[{axis}]~=0 and angular>=math.pi/2-0.1 and rotation_distance(tool_offset(k.actual_tcp_pose,{axis},direction[{axis}]*0.001),reference)>angular then return 'rotation_boundary' end")
            bounds+="\nassert(rotation_distance(k.actual_tcp_pose,"+fixed+")<=math.pi/2,'product_orientation')"
            program=program.replace('-- PRODUCT_HARD_BOUNDS','if not for_stop then\n'+bounds+'\n end')
            program=program.replace('local function product_boundary(k,goal,direction) return nil end',
                'local function product_boundary(k,goal,direction)\n'+'\n'.join(soft)+'\n return nil end')
            absolute='\n'.join(f"assert(np[{i+1}]>={joint_lo[i]!r} and np[{i+1}]<={joint_hi[i]!r},'product_joint_reference')" for i in range(6))
            program=program.replace('entered=true;last_send=timestamp()',absolute+'\n            entered=true;last_send=timestamp()')
            proc=await asyncio.create_subprocess_exec('docker','exec','-i',admitted['container_id'],'bash','-c',bridge,
                stdin=asyncio.subprocess.PIPE,stdout=asyncio.subprocess.PIPE,stderr=asyncio.subprocess.PIPE)
            proc.stdin.write(program.encode());await proc.stdin.drain();proc.stdin.close()
            async with asyncio.timeout(15):
                while line:=await proc.stdout.readline():
                    fields=line.decode('utf-8',errors='replace').strip().split('\t')
                    await self.recorder.write_event(dict(kind='docker_pvat_controller',fields=fields),self.clock())
                    if fields[0]=='LIVE_READY':
                        if self.pump:raise ValueError('pvat_duplicate_ready')
                        self.pump=DockerPvatInput(ExecutorLease(),clock=self.clock,authorize=self.check,encoder=encode_pose)
                        self.pump.start(client,int(fields[1]))
                        # Wait for a frame captured after the controller handshake.
                        self.phase='executing'
                    elif fields[0]=='LIVE_POINT':
                        self.steps+=1
                        a.position_pvat_points=getattr(a,'position_pvat_points',0)+1
                    elif fields[0]=='LIVE_QUIET':quiet=True
                    elif fields[0]=='LIVE_BRAKE':self.phase='draining'
                    elif fields[0]=='LIVE_BOUNDARY':boundary=fields[2]
                    elif fields[0]=='LIVE_ABORT':abort_confirmed=fields[1:3]==['true','true']
                    elif fields[0]=='LIVE_DONE':
                        done=True;okay=fields[1]=='true'
                        reference_rejected=not okay and fields[2].endswith(': reference_limit')
                        if not okay:
                            for code in ('reference_limit','tcp_region','joint_region'):
                                if fields[2].endswith(': '+code):known_failure=code
                        if self.pump:await self.pump.withdraw()
                        break
                    if self.pump and self._withdraw.is_set():await self.pump.withdraw()
                transport_ok=await proc.wait()==0
                okay=okay and transport_ok
            known_rejection=known_failure is not None and abort_confirmed and transport_ok
            if self.pump:
                await self.pump.complete(client,controller_ok=done and (okay or known_rejection),quiet_observed=quiet)
            elif not (done and okay and quiet):raise ValueError('pvat_completion_unverified')
            a.position_busy=False;a._motion_accepted=a._motion_write_stop_required=False
            await self.recorder.write_event(dict(kind='docker_pvat_outcome',controller_passed=okay,
                reference_rejected=reference_rejected and known_rejection,known_rejection=known_rejection,
                failure_code=known_failure,boundary=boundary,
                input_exit_reason=self.pump.reason if self.pump else None),self.clock())
            if known_rejection:
                self.phase='fault';self.reason='pvat_'+known_failure
            else:
                # A verified time-window completion may roll over using a
                # CURRENT held input. Boundary, expiry and revocation may not.
                latest=self.gate.latest
                continue_held=(boundary in (None,'reference_boundary') and self.pump is not None and self.pump.reason is None
                    and not self.closed and not self._withdraw.is_set() and latest is not None
                    and latest.kind=='held' and 0<=self.clock()-latest.received_ns<INPUT_NS)
                self.hold=not self.gate.released and not continue_held
                if continue_held and boundary=='reference_boundary':a.position_pvat_reference_rollovers=getattr(a,'position_pvat_reference_rollovers',0)+1
                self.phase='hold' if self.hold else 'ready'
                self.reason=('pvat_'+boundary if boundary else
                    'position_input_expired' if self.pump and self.pump.reason=='pvat_input_expired' else
                    'pvat_gesture_complete') if self.hold else None
        except BaseException as error:
            self.phase='fault';self.reason='stop_unverified' if a.position_busy else str(error)
            if a.position_busy:a._latched_fault='stop_unverified'
        finally:
            if self.pump:await self.pump.withdraw()

    async def advance(self):
        if self.worker and self.worker.done():
            self.worker.result();self.worker=None;self.pump=None
        if self.closed or self.phase=='fault':return
        try:self.check()
        except Exception as error:await self.abort(str(error));return
        if not self.busy and not self.hold and self.gate.latest and not self.gate.released:
            if not 0<=self.clock()-self.gate.latest.received_ns<INPUT_NS:
                self.hold=True;self.phase='hold';self.reason='position_input_expired';return
            self._withdraw=asyncio.Event();self.phase='planning'
            self.worker=asyncio.create_task(self._run(),name='docker-pvat-session')

    async def abort(self,reason):
        self.closed=True;self._withdraw.set();self.gate.revoke(str(reason))
        if self.pump:await self.pump.withdraw()
        if self.worker:await asyncio.shield(self.worker)
        self.phase='fault';self.reason=self.adapter._latched_fault or str(reason)

    def view(self):
        return PositionView(self.phase,self.binding.generation,self.steps,reason=self.reason,
            blocked_input_seq=self.gate.latest.seq if self.hold and self.gate.latest else None)


class DockerPvatOrWebSession:
    """Select once per preparation; never replace an executor with work pending."""
    def __init__(self,adapter,binding,**kwargs):
        from app.control.docker_motion_session import DockerMotionSession
        self.active=DockerMotionSession(adapter,binding,**kwargs)
        self.adapter,self.binding,self.kwargs=adapter,binding,kwargs
        self.source=None

    def offer(self,intent):
        if self.source is None:
            if intent.session_id.startswith('xr-finite-'):
                self.active=DockerPvatSession(self.adapter,self.binding,**self.kwargs)
            self.source=intent.session_id
        if self.source!=intent.session_id:raise ValueError('position_source_requires_preparation')
        self.active.offer(intent)

    @property
    def busy(self):return self.active.busy
    @property
    def worker(self):return self.active.worker
    @property
    def gate(self):return self.active.gate
    @property
    def phase(self):return self.active.phase
    async def advance(self):await self.active.advance()
    async def abort(self,reason):await self.active.abort(reason)
    def view(self):return self.active.view()
