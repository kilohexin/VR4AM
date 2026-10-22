"""Owned read-only groups for the separately admitted finite-position entry."""
from contextlib import contextmanager
from contextvars import ContextVar
from app.commissioning.paused_stop_cleanup import BoundedStopOnlyClient

FIELDS=frozenset(('get_robot_state','get_estop_reason','get_kin_data',
    'get_tcp','get_claw','get_running_motion'))


class PositionSnapshotGroups:
    """Mixin; no native loader, motion method or independent authorization."""
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self._position_group_context=ContextVar('position_snapshot_group',default=None)
        self._position_active_group=None
        self._position_children=[]
        self.snapshot_groups=[]

    def _position_group_live(self,group):
        if (group is not self._position_active_group or not group['active']
                or self._position_group_context.get() is not group or self.read_gate_closed
                or self.owner is None or self.owner.done() or self.owner.cancelling()
                or self.deadline_clock()>=self.read_deadline_ns):
            raise ValueError('position_snapshot_group_revoked')
        group['guard']()

    def read_guard(self):
        group=self._position_group_context.get()
        if group is not None:
            # Full approval/file checks bracket this read-only group. Children
            # retain cancellation, connection/generation and fixed-budget guards.
            self._position_group_live(group)
            return
        if self._position_active_group is not None:
            self.read_gate_closed=True
            raise ValueError('position_snapshot_foreign_read')
        super().read_guard()

    def _position_children_clear(self):
        if (self.pending_read is not None and not self.pending_read.done()
                or any(c.read_gate_closed or c.read_remote_result_unknown
                    or c.pending_read is None or not c.pending_read.done()
                    for c in self._position_children)):
            self.read_gate_closed=True
            raise ValueError('position_snapshot_children_not_clear')

    @contextmanager
    def position_snapshot_scope(self,guard,purpose):
        super().read_guard();self._position_children_clear();guard()
        if self._position_active_group is not None:raise ValueError('position_snapshot_group_busy')
        row=dict(purpose=purpose,started_ns=self.deadline_clock(),outcome='entered',methods=[])
        self.snapshot_groups.append(row)
        group=dict(active=True,guard=guard,used=set(),verified=False,row=row)
        self._position_active_group=group;token=self._position_group_context.set(group)
        try:
            yield
            if not group['verified']:raise ValueError('position_snapshot_group_not_verified')
            row['outcome']='returned'
        except BaseException:
            self.read_gate_closed=True;row['outcome']='failed'
            raise
        finally:
            row['completed_ns']=self.deadline_clock();group['active']=False
            self._position_active_group=None;self._position_group_context.reset(token)

    def verify_position_snapshot_group(self):
        group=self._position_group_context.get();self._position_group_live(group)
        self._position_children_clear()
        if group['used']!=FIELDS:raise ValueError('position_snapshot_group_incomplete')
        # Before snapshot installation, re-read and verify the complete approval.
        super().read_guard();group['guard']();group['verified']=True

    async def read(self,name,invoke):
        group=self._position_group_context.get()
        if group is None or name not in FIELDS:
            if group is not None and name not in ('is_connected','get_motion_state'):
                self.read_gate_closed=True;raise ValueError('position_snapshot_group_method')
            return await super().read(name,invoke)
        self._position_group_live(group)
        if name in group['used']:
            self.read_gate_closed=True;raise ValueError('position_snapshot_group_duplicate')
        group['used'].add(name);group['row']['methods'].append(name)
        def authorize():self._position_group_live(group)
        child_type=getattr(type(self),'snapshot_read_client_type',BoundedStopOnlyClient)
        child=child_type(self.client,lambda:None,authorize,self.owner,deadline_clock=self.deadline_clock)
        child.method=name;child.native_item=None
        child.read_deadline_ns=self.read_deadline_ns;child.read_calls=self.read_calls
        self._position_children.append(child)
        async def native():
            child.native_item=next(r for r in reversed(self.read_calls) if r['method']==name)
            return await invoke()
        try:
            value=await child.read(name,native)
            if name=='get_estop_reason':self.raw_estop_clear=child.raw_estop_clear
            return value
        except BaseException:
            self.read_gate_closed=True
            raise
        finally:
            self.read_gate_closed |= child.read_gate_closed
            self.read_remote_result_unknown |= child.read_remote_result_unknown

    def require_clear_read_gate(self):
        if self._position_active_group is not None:raise ValueError('position_snapshot_group_still_active')
        self._position_children_clear()
        return super().require_clear_read_gate()

    def _native_snapshot_result_unknown(self,method):
        rows=[r for r in self.read_calls if r['method']==method]
        children=[c for c in self._position_children if c.method==method]
        if rows and children and children[-1].native_item is rows[-1]:
            row=rows[-1];child=children[-1]
            if row['remote_result_unknown'] or child.read_remote_result_unknown:return True
            if not row['entered'] or 'native_completed_ns' in row:return False
            task=child.pending_read
            return task is None or not task.done() or task.cancelled()
        return super()._native_snapshot_result_unknown(method)
