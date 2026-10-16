"""Single diagnostic proxy with a shared native read-entry budget.

The inherited parallel snapshot owners are used only for reads. Every inherited
motion route is denied; speedl and the reserved stop have dedicated entries.
"""
import asyncio
import copy
from contextlib import contextmanager

from app.commissioning.finite_window_run import FiniteScopeClient
from app.commissioning.idle_stop_baseline import AdmissionRejected
from app.commissioning.native_jog_site_scope import SiteJogScope, permission


class _ReadSurface:
    def __init__(self, native, proxy):
        self.__native, self.__proxy = native, proxy

    def __getattr__(self, name):
        if name.startswith('_'): raise AttributeError(name)
        method = getattr(self.__native, name)
        if not callable(method): raise AttributeError(name)
        async def invoke(*args, **kwargs):
            p = self.__proxy
            p.read_guard()
            if not p.speedl_entries: p._jog_scope.read()
            p.read_guard()  # Local authorization work cannot extend the total deadline.
            group=p._snapshot_context.get()
            active=p._active_snapshot_group
            if active is not None:
                if group is not active or not active['active']:
                    p.read_gate_closed=True
                    raise AdmissionRejected('native_jog_snapshot_native_foreign')
                used=active.setdefault('native_used',set())
                if name in used or name not in {'is_connected','get_robot_state','get_estop_reason',
                        'get_kin_data','get_tcp','get_claw','get_running_motion','get_motion_state'}:
                    p.read_gate_closed=True
                    raise AdmissionRejected('native_jog_snapshot_native_duplicate_or_method')
                used.add(name)
            if name not in p.READS:
                raise AdmissionRejected('native_jog_write_forbidden')
            if p.read_entries >= 256:
                p.read_gate_closed = True
                raise AdmissionRejected('native_jog_read_budget_exhausted')
            if name in p.special_entries and p.special_entries[name] >= 1:
                p.read_gate_closed = True
                raise AdmissionRejected('native_jog_math_or_factor_consumed')
            p.read_entries += 1
            if name in p.special_entries: p.special_entries[name] += 1
            row = dict(method=name, started_ns=p.clock(), outcome='entered', remote_result_unknown=False)
            p.native_reads.append(row)
            try:
                result = await method(*copy.deepcopy(args), **copy.deepcopy(kwargs))
            except BaseException:
                # Cancellation can abandon a remote request. Never clear this later.
                row.update(outcome='error', remote_result_unknown=True)
                raise
            else:
                row.update(outcome='returned', completed_ns=p.clock())
                return result
        return invoke


class SiteJogClient(FiniteScopeClient):
    READS = frozenset({'is_connected','get_robot_state','get_estop_reason','get_kin_data',
        'get_tcp','get_claw','get_running_motion','get_motion_state',
        'kinematics_forward','get_velocity_factor'})

    def __init__(self, native, scope, *, owner, clock, motion_permission, stop_permission):
        if type(scope) is not SiteJogScope or not all(callable(x) for x in (clock,motion_permission,stop_permission)):
            raise ValueError('native_jog_explicit_client_scope_required')
        self._native = native
        self._jog_scope = scope
        self.read_entries = 0
        self.special_entries = dict(kinematics_forward=0, get_velocity_factor=0)
        self.native_reads = []
        self.speedl_entries = 0
        self.speedl_started_ns = None
        self.speedl_call = None
        self.speedl_remote_result_unknown = False
        self.pending_speedl = None
        self.speedl_attempted = False
        def read_permission():
            # Read-only cleanup uses the reserved total budget after native entry.
            if not self.speedl_entries: scope._binding()
        super().__init__(_ReadSurface(native,self), motion_permission, stop_permission,
            read_permission, owner, clock, parallel_motion_reads=True, deadline_clock=clock)
        self.read_deadline_ns = clock()+20_000_000_000
        self._motion_permission_check = motion_permission
        self._stop_permission_check = stop_permission
        self._snapshot_owner_archive = []
        self.snapshot_groups = []

    def _snapshot_read_pending(self):
        return (super()._snapshot_read_pending() or any(child.pending_read is not None
            and not child.pending_read.done() for child in self._snapshot_owner_archive))

    def require_clear_read_gate(self):
        if any(child.read_gate_closed or child.read_remote_result_unknown
                for child in self._snapshot_owner_archive):
            self.read_gate_closed=True
            raise AdmissionRejected('native_jog_snapshot_archive_not_clear')
        return super().require_clear_read_gate()

    @contextmanager
    def _parallel_snapshot_scope(self, guard, *, purpose='motion'):
        # Exactly one preflight and one motion group. Completed ownership is
        # archived, never reset or discarded to make a failed group admissible.
        self.require_clear_read_gate()
        used=[row['purpose'] for row in self.snapshot_groups]
        if (purpose not in ('preflight','motion') or purpose in used or 'motion' in used
                or self.speedl_attempted or len(used)>=2):
            raise AdmissionRejected('native_jog_snapshot_group_not_admitted')
        if self.snapshot_read_owners:
            self._snapshot_owner_archive.extend(self.snapshot_read_owners.values())
            self.snapshot_read_owners={}
        row=dict(purpose=purpose,started_ns=self.clock(),outcome='entered')
        self.snapshot_groups.append(row)
        try:
            with super()._parallel_snapshot_scope(guard):yield
            row['outcome']='returned'
        except BaseException:
            self.read_gate_closed=True
            row['outcome']='failed'
            raise
        finally:
            row['completed_ns']=self.clock()

    def _native_motion_entry_count(self, method):
        return self.speedl_entries if method == 'speedl' else 0

    def _native_snapshot_entry_count(self, method):
        return sum(r['method'] == method for r in self.native_reads)

    def _native_snapshot_result_unknown(self, method):
        rows = [r for r in self.native_reads if r['method'] == method]
        return bool(rows and (rows[-1]['remote_result_unknown'] or rows[-1]['outcome'] == 'entered'))

    async def kinematics_forward(self, joints):
        return await self.read('kinematics_forward', lambda:self.client.kinematics_forward(joints))

    async def get_velocity_factor(self):
        return await self.read('get_velocity_factor', self.client.get_velocity_factor)

    async def speedl(self, a, v, t, frame):
        if self.speedl_attempted:
            raise AdmissionRejected('native_jog_speedl_consumed')
        self.speedl_attempted = True
        expected = dict(x=0.,y=0.,z=.005,rx=0.,ry=0.,rz=0.)
        if (type(a) not in (int,float) or a != .01 or type(t) not in (int,float) or t != .08
                or type(v) is not dict or v != expected or any(type(x) not in (int,float) for x in v.values())
                or type(frame) is not dict or frame != dict.fromkeys(expected,0.)
                or any(type(x) not in (int,float) for x in frame.values())):
            raise AdmissionRejected('native_jog_fixed_parameters_required')
        self._jog_scope.write()
        permission(self._motion_permission_check)
        self.require_clear_read_gate()
        self.speedl_entries += 1
        self.speedl_started_ns = self.clock()
        self.pending_speedl = asyncio.current_task()
        self.speedl_call = dict(native_started_ns=self.speedl_started_ns, outcome='entered')
        try:
            result = await self._native.speedl(a,copy.deepcopy(v),t,copy.deepcopy(frame))
            if type(result) is not int or result <= 0:
                raise AdmissionRejected('native_jog_invalid_motion_id')
            self.speedl_call.update(outcome='returned',native_completed_ns=self.clock())
            return result
        except BaseException:
            self.speedl_remote_result_unknown = True
            self.speedl_call['outcome'] = 'unknown'
            raise

    async def stop_move(self):
        if self.sdk_stop_entries or self.speedl_entries != 1:
            raise AdmissionRejected('native_jog_stop_not_admitted')
        permission(self._stop_permission_check)
        self.sdk_stop_entries += 1
        self.native_started_ns = self.clock()
        try:
            return await self._native.stop_move()
        finally:
            self.native_completed_ns = self.clock()

    async def _deny(self, *args, **kwargs):
        raise AdmissionRejected('native_jog_write_forbidden')

    move_pvat = kinematics_inverse = _kinematics_forward_guarded = _deny
