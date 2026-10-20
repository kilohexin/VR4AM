"""Single diagnostic proxy with a shared native read-entry budget.

The inherited parallel snapshot owners are used only for reads. Every inherited
motion route is denied; speedl and the reserved stop have dedicated entries.
"""
import asyncio
import copy
from contextlib import contextmanager
from contextvars import ContextVar

from app.commissioning.finite_window_run import FiniteScopeClient
from app.commissioning.idle_stop_baseline import AdmissionRejected
from app.commissioning.native_jog_site_scope import SiteJogScope, permission
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient
from app.commissioning.native_jog_mdh import _components

FEEDBACK_READ_POLICY = 'one_fk_paired_with_one_fresh_motion_snapshot_independent_owners'


class _PairedFKOwner(BoundedStopOnlyClient):
    READS = frozenset({'kinematics_forward'})


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
            paired_fk = p._pair_native_entry(name,args,kwargs)
            if active is not None and not paired_fk:
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
        self._feedback_pair = None
        self._feedback_context = ContextVar('native_jog_feedback_pair',default=None)

    def _pair_require(self,pair,*,authorize=False):
        BoundedStopOnlyClient.read_guard(self)
        # Actual native fields already run scope.read in _ReadSurface; the
        # snapshot group also checks its boundary. Do not repeat file/input
        # work on every ownership poll. The math owner and join still check it.
        if authorize:self._jog_scope.read()
        if (self._feedback_pair is not pair or not pair['active']
                or self._native is not pair['native'] or self.client is not pair['surface']
                or self.owner is not pair['owner'] or self.speedl_attempted):
            self.read_gate_closed=True
            raise AdmissionRejected('native_jog_feedback_pair_revoked')

    def read_guard(self):
        super().read_guard()
        pair=self._feedback_pair
        if pair is not None and pair['active']:
            context=self._feedback_context.get()
            if context is None or context[0] is not pair or context[1] not in ('fk','snapshot'):
                self.read_gate_closed=True
                raise AdmissionRejected('native_jog_feedback_pair_foreign_read')
            self._pair_require(pair)

    def _pair_native_entry(self,name,args,kwargs):
        pair=self._feedback_pair
        if pair is None or not pair['active']:return False
        self._pair_require(pair)
        context=self._feedback_context.get()
        role=context[1] if context is not None and context[0] is pair else None
        if role=='fk':
            if (name!='kinematics_forward' or asyncio.current_task() is not pair['fk_owner'].pending_read
                    or len(args)!=1 or kwargs or _components(args[0],6)!=pair['q']):
                self.read_gate_closed=True
                raise AdmissionRejected('native_jog_feedback_pair_foreign_math')
            return True
        if (role!='snapshot' or self._active_snapshot_group is None
                or self._snapshot_context.get() is not pair.get('snapshot_group')
                or name not in BoundedStopOnlyClient.READS):
            self.read_gate_closed=True
            raise AdmissionRejected('native_jog_feedback_pair_foreign_snapshot')
        return False

    async def read(self,name,invoke):
        pair=self._feedback_pair
        if pair is not None and pair['active']:
            self.read_guard()
            if self._feedback_context.get()[1]!='snapshot' or name not in BoundedStopOnlyClient.READS:
                self.read_gate_closed=True
                raise AdmissionRejected('native_jog_feedback_pair_method_not_admitted')
        return await super().read(name,invoke)

    async def feedback_pair(self,joints,snapshot_reader):
        self.require_clear_read_gate()
        if (self._feedback_pair is not None or asyncio.current_task() is not self.owner
                or not callable(snapshot_reader) or self.special_entries['kinematics_forward']
                or self.speedl_attempted):
            self.read_gate_closed=True
            raise AdmissionRejected('native_jog_feedback_pair_consumed_or_foreign')
        q=_components(joints,6)
        pair=dict(active=True,native=self._native,surface=self.client,owner=self.owner,q=q,
            report=dict(started_ns=self.clock(),outcome='entered',policy=FEEDBACK_READ_POLICY))
        self._feedback_pair=pair
        child=pair['fk_owner']=_PairedFKOwner(self.client,lambda:None,
            lambda:self._pair_require(pair,authorize=True),self.owner,deadline_clock=self.clock)
        child.read_deadline_ns=self.read_deadline_ns
        child.read_calls=self.read_calls
        async def fk():
            token=self._feedback_context.set((pair,'fk'))
            try:
                value=await child.read('kinematics_forward',lambda:self.client.kinematics_forward(list(q)))
                rows=[r for r in self.native_reads if r['method']=='kinematics_forward']
                if (len(rows)!=1 or rows[0]['outcome']!='returned'
                        or rows[0]['started_ns']<pair['report']['started_ns']
                        or type(rows[0].get('completed_ns')) is not int):
                    raise AdmissionRejected('native_jog_feedback_pair_math_completion_not_owned')
                # Local parsing/authorization after the native reply cannot
                # make an old mathematical result younger.
                pair['report']['fk_completed_ns']=rows[0]['completed_ns']
                return value
            finally:self._feedback_context.reset(token)
        async def snapshot():
            token=self._feedback_context.set((pair,'snapshot'))
            try:
                result=await snapshot_reader()
                if not pair.get('snapshot_group') or not pair.get('snapshot_returned'):
                    raise AdmissionRejected('native_jog_feedback_pair_snapshot_not_owned')
                pair['report']['snapshot_completed_ns']=self.clock()
                return result
            finally:self._feedback_context.reset(token)
        tasks=[asyncio.create_task(fk(),name='native-jog-paired-fk'),
            asyncio.create_task(snapshot(),name='native-jog-paired-snapshot')]
        pair['tasks']=tasks
        for task in tasks:task.add_done_callback(lambda done:None if done.cancelled() else done.exception())
        try:
            done,_=await asyncio.wait(tasks,timeout=min(.2,max(0.,(self.read_deadline_ns-self.clock())/1e9)),
                return_when=asyncio.FIRST_EXCEPTION)
            for task in done:task.result()
            if len(done)!=2:raise AdmissionRejected('native_jog_feedback_pair_timeout')
            self._pair_require(pair,authorize=True)
            if child.read_gate_closed or child.read_remote_result_unknown:
                raise AdmissionRejected('native_jog_feedback_pair_math_not_clear')
            pair['report']['outcome']='returned'
            return tasks[0].result(),pair['report']['fk_completed_ns'],tasks[1].result()
        except BaseException:
            self.read_gate_closed=True
            pair['report']['outcome']='failed'
            self.read_remote_result_unknown |= (child.read_remote_result_unknown
                or child.pending_read is not None and not child.pending_read.done())
            for task in tasks:
                if not task.done() and not task.cancelling():task.cancel()
            raise
        finally:
            pair['active']=False
            pair['report']['completed_ns']=self.clock()

    def feedback_pair_report(self):
        pair=self._feedback_pair
        if pair is None:return dict(policy=FEEDBACK_READ_POLICY,outcome='not_started')
        child=pair['fk_owner']
        return dict(**pair['report'],math_remote_result_unknown=bool(child.read_remote_result_unknown
            or child.pending_read is not None and not child.pending_read.done()),
            math_gate_closed=child.read_gate_closed)

    def _snapshot_read_pending(self):
        return (super()._snapshot_read_pending() or any(child.pending_read is not None
            and not child.pending_read.done() for child in self._snapshot_owner_archive))

    def require_clear_read_gate(self):
        pair=self._feedback_pair
        if pair is not None:
            startup=(pair['active'] and self._feedback_context.get() is not None
                and self._feedback_context.get()[0] is pair and self._feedback_context.get()[1]=='snapshot'
                and pair.get('snapshot_group') is None)
            if not startup and (pair['active'] or pair['report']['outcome']!='returned'
                    or pair['fk_owner'].read_gate_closed or pair['fk_owner'].read_remote_result_unknown
                    or pair['fk_owner'].pending_read is not None and not pair['fk_owner'].pending_read.done()):
                self.read_gate_closed=True
                raise AdmissionRejected('native_jog_feedback_pair_not_clear')
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
            with super()._parallel_snapshot_scope(guard):
                pair=self._feedback_pair
                if pair is not None and pair['active']:
                    if purpose!='motion':raise AdmissionRejected('native_jog_feedback_pair_wrong_snapshot')
                    pair['snapshot_group']=self._active_snapshot_group
                yield
                if pair is not None and pair['active']:pair['snapshot_returned']=True
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
        if self._feedback_pair is not None and self._feedback_pair['active']:
            self.read_gate_closed=True
            raise AdmissionRejected('native_jog_feedback_pair_not_clear')
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
