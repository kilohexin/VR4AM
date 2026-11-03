"""Opt-in native Docker pose-follow bridge. No physical or Fake selector."""
import asyncio
from pathlib import Path

from app.control.docker_pvat_input import ExecutorLease
from app.control.docker_pvat_session import DockerPvatSession
from app.control.vr_pose_input import VRPoseInput


def build_pose_program(adapter):
    from scipy.spatial.transform import Rotation
    root = Path(__file__).resolve().parents[3] / 'scripts'
    source = (root / 'docker_pose_live.lua').read_text(encoding='utf-8')
    for marker, name in (('__POSE_MAILBOX_MODULE__', 'vr_pose_mailbox.lua'),
                         ('__POSE_TRACKER_MODULE__', 'vr_pose_tracker.lua')):
        if source.count(marker) != 1:
            raise ValueError('pose_program_marker')
        source = source.replace(marker, (root / name).read_text(encoding='utf-8'))
    s = adapter.settings
    lower = tuple(q + s.joint_limit_margin_rad for q in s.soft_joint_min_rad)
    upper = tuple(q - s.joint_limit_margin_rad for q in s.soft_joint_max_rad)
    bounds = ['local function product_bounds(p,q)']
    for i, axis in enumerate(('x', 'y', 'z')):
        bounds.append(f"if p and (p.{axis}<{s.startup_tcp_min_m[i]!r} or p.{axis}>{s.startup_tcp_max_m[i]!r}) then return 'workspace_boundary' end")
    for i in range(6):
        bounds.append(f"if q and (q[{i+1}]<{lower[i]!r} or q[{i+1}]>{upper[i]!r}) then return 'joint_boundary' end")
    r = Rotation.from_quat(adapter._position_pitch_reference).as_euler('xyz')
    fixed = '{rx=' + repr(float(r[0])) + ',ry=' + repr(float(r[1])) + ',rz=' + repr(float(r[2])) + '}'
    bounds.extend(["if p and angle(p," + fixed + ")>math.pi/2 then return 'rotation_boundary' end", 'return nil end'])
    if source.count('-- PRODUCT_POSE_BOUNDS') != 1:
        raise ValueError('pose_program_marker')
    return source.replace('-- PRODUCT_POSE_BOUNDS', '\n'.join(bounds))


class DockerPoseSession(DockerPvatSession):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if getattr(self.adapter, 'position_pose_follow_vr', False) is not True:
            raise ValueError('pose_follow_not_enabled')
        self._pose_ready = False
        self._reference = None
        self._anchor_tcp = None

    def offer(self, intent):
        if intent.kind == 'held' and intent.hand_pose is None:
            raise ValueError('pose_follow_requires_pose')
        first = self.gate.released and not self.busy and intent.kind == 'held'
        if first:
            from app.control.vr_pose_reference import VRPoseReference
            self.check()
            snapshot = self.adapter._snapshot
            if snapshot is None or snapshot.robot_state.value != 'IDLE':
                raise ValueError('pose_capture_requires_idle')
            reference = VRPoseReference(clock=self.clock)
            reference.capture(intent.hand_pose, snapshot.actual_tcp, generation=self.binding.generation,
                hand_ns=intent.received_ns, tcp_ns=snapshot.captured_ns)
            self._reference, self._anchor_tcp = reference, snapshot.actual_tcp
        super().offer(intent)
        if intent.kind == 'release' and self.pump:
            self.pump.revoke('vr_pose_released')

    def _forward(self, intent):
        if self._pose_ready:
            self.pump.offer(intent)

    def _input_authority(self):
        # Private inactive-bank writes cannot publish a goal. Keep immediate
        # control/connection/revocation checks on each call; full file/config
        # identity is checked once immediately before its commit header.
        self.authorize()
        if self.closed or self.adapter.position_binding is not self.binding:
            raise ValueError('pose_input_revoked')
        if getattr(self.recorder,'fatal_error',None) or getattr(self.recorder,'dropped_normal_events',0):
            raise ValueError('position_recording_unavailable')

    async def _run(self):
        a = self.adapter
        try:
            from app.digital_twin.docker_runtime import verify_local_docker
            admitted = await asyncio.to_thread(verify_local_docker)
            self.check()
            if admitted != a._docker_admission or a.position_busy or a._docker_entry_owner is not None:
                raise ValueError('pose_admission_changed_or_busy')
            client = a._docker_ready_client
            anchor = [None]

            def stamp(ns):
                if anchor[0] is None or ns < anchor[0][0]:
                    raise ValueError('pose_input_before_handshake')
                return anchor[0][1] + (ns - anchor[0][0]) // 1_000_000

            self.pump = VRPoseInput(client, ExecutorLease(), self.binding, clock=self.clock,
                stamp_for_capture=stamp, authorize=self._input_authority, authorize_publish=self.check,
                authorize_cleanup=lambda: a._position_health(self.binding))
            try:
                await self.pump.open()
            except Exception:
                if (self._withdraw.is_set() and not self.pump.writer.read_unknown
                        and not self.pump.writer.unknown):
                    self.pump.lease.release(self.pump.writer.token)
                    self.phase = 'ready'
                    return
                raise
            self.check()
            if self.gate.released or self._withdraw.is_set():
                await self.pump.withdraw()
                self.pump.lease.release(self.pump.writer.token)  # No actuator program launched.
                self.phase = 'ready'
                return
            source = build_pose_program(a)
            a.position_busy = True
            a._motion_accepted = a._motion_write_stop_required = True
            bridge = ('exec 3<>/dev/tcp/127.0.0.1/3010; cat >&3; '
                      'while IFS= read -r -t 12 line <&3; do printf "%s\\n" "$line"; '
                      'case "$line" in *LIVE_DONE*) break;; esac; done')
            proc = await asyncio.create_subprocess_exec('docker', 'exec', '-i', admitted['container_id'], 'bash', '-c', bridge,
                stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
            proc.stdin.write(source.encode())
            await proc.stdin.drain()
            proc.stdin.close()
            done = okay = quiet = False
            boundary = None
            expired = False
            async with asyncio.timeout(15):
                while line := await proc.stdout.readline():
                    fields = line.decode('utf-8', errors='replace').strip().split('\t')
                    await self.recorder.write_event(dict(kind='docker_pose_controller', fields=fields), self.clock())
                    if fields[0] == 'LIVE_READY':
                        if anchor[0] is not None:
                            raise ValueError('pose_duplicate_ready')
                        anchor[0] = (self.clock(), int(fields[1]))
                        snapshot = await a._read_snapshot(read_purpose='position_progress')
                        self.check()
                        intent = self.gate.latest
                        deadline = self.clock() + 100_000_000
                        while (not self._withdraw.is_set() and intent is not None and intent.kind == 'held'
                                and intent.received_ns < anchor[0][0] and self.clock() < deadline):
                            await asyncio.sleep(.005)
                            self.check()
                            intent = self.gate.latest
                        if not self._withdraw.is_set() and intent is not None and intent.kind == 'held':
                            import math
                            from scipy.spatial.transform import Rotation
                            if (self._anchor_tcp is None or math.dist(snapshot.actual_tcp.p,self._anchor_tcp.p)>.0001
                                    or (Rotation.from_quat(snapshot.actual_tcp.q).inv()
                                        *Rotation.from_quat(self._anchor_tcp.q)).magnitude()>.001):
                                raise ValueError('pose_capture_robot_changed')
                            self.pump.inherit_capture(self._reference, latest_ns=intent.received_ns)
                            self.pump.offer(intent)
                            self._pose_ready = True
                            self.pump.start()
                            self.phase = 'planning'
                    elif fields[0] == 'LIVE_POINT':
                        self.phase = 'executing'
                        self.steps += 1
                        a.position_pvat_points = getattr(a, 'position_pvat_points', 0) + 1
                    elif fields[0] == 'LIVE_QUIET':
                        quiet = True
                    elif fields[0] == 'LIVE_EXPIRED':
                        expired = True
                    elif fields[0] == 'LIVE_BOUNDARY':
                        boundary = fields[2]
                    elif fields[0] == 'LIVE_DONE':
                        done, okay = True, fields[1] == 'true'
                        break
                exited = await proc.wait() == 0
            await self.pump.complete(controller_ok=done and okay, quiet_observed=quiet, program_exited=exited)
            a.position_busy = False
            a._motion_accepted = a._motion_write_stop_required = False
            self.hold = not self.gate.released
            self.phase = 'hold' if self.hold else 'ready'
            self.reason = ('pvat_' + boundary if boundary else
                'position_input_expired' if expired else 'pvat_gesture_complete') if self.hold else None
            await self.recorder.write_event(dict(kind='docker_pose_outcome', controller_passed=okay,
                input_exit_reason=self.pump.reason, boundary=boundary), self.clock())
        except BaseException:
            self.phase = 'fault'
            unknown = self.pump is not None and (self.pump.writer.read_unknown
                or self.pump.writer.unknown or self.pump.unknown_cleanup)
            self.reason = 'stop_unverified' if a.position_busy or unknown else 'pose_startup_rejected'
            if a.position_busy or unknown:
                a._latched_fault = 'stop_unverified'
        finally:
            self._pose_ready = False
            if self.pump:
                await self.pump.withdraw()
                await self.recorder.write_event(dict(kind='docker_pose_input_end',reason=self.pump.reason,
                    written_seq=self.pump.writer.written_seq,max_publish_ms=self.pump.max_publish_ns/1e6,
                    unknown_write=self.pump.writer.unknown,unknown_read=self.pump.writer.read_unknown),self.clock())
