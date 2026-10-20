"""Explicit experimental scope. No production registration or default factory."""
import asyncio
import copy
import inspect
import ipaddress
from collections.abc import Mapping
from contextlib import nullcontext
from dataclasses import dataclass, field

from app.commissioning.native_jog_core import _SingleJogSubmission
from app.commissioning.native_jog_model_guard import _ModelRegionGuardCore
from app.commissioning.native_jog_packet import NativeJogLimits, _sha
from app.commissioning.native_jog_mdh import MdhTool
from app.config import LebaiSettings
from app.robots.base import BackendCommandError

RUNTIME_PRECHECK_POLICY = 'single_pre_pair_scan_original_native_write_scan'


@dataclass(frozen=True)
class _RuntimePrecheck:
    adapter: object
    scope: object
    settings: object
    limits: object
    config_sha256: str
    client: object
    owner: object
    generation: int
    snapshot_owner: object
    read_sequence: int
    native_reads: object
    read_entries: int


def permission(callback):
    value = callback()
    if value is not None:
        if inspect.iscoroutine(value):
            value.close()
        raise ValueError('native_jog_invalid_authorization_result')


def site_address(value):
    try:
        ip = ipaddress.IPv4Address(value)
    except Exception:
        raise ValueError('native_jog_literal_private_ipv4_required') from None
    if str(ip) != value or not (value == '127.0.0.1' or any(ip in ipaddress.IPv4Network(net)
            for net in ('10.0.0.0/8', '172.16.0.0/12', '192.168.0.0/16'))):
        raise ValueError('native_jog_literal_private_ipv4_required')


@dataclass(frozen=True)
class _SiteRegionGuard(_ModelRegionGuardCore):
    pass


@dataclass(frozen=True)
class SiteJogScope:
    settings: LebaiSettings
    limits: NativeJogLimits
    config_sha256: str
    reference_tool: MdhTool
    authorize: object
    hot_check: object
    write_check: object
    read_check: object = None
    _guard: object = field(init=False, repr=False, compare=False)

    def __post_init__(self):
        s, l = self.settings, self.limits
        if type(s) is not LebaiSettings or type(l) is not NativeJogLimits:
            raise ValueError('native_jog_explicit_scope_required')
        site_address(s.ip)
        _sha(self.config_sha256)
        if (s.mode != 'control' or l.max_speed_mps != .005 or l.max_acceleration_mps2 != .010
                or not 0 < s.control.max_joint_speed_radps <= .05
                or not 0 < s.control.max_joint_acceleration_radps2 <= .2
                or any(lo < a or hi > b for lo, hi, a, b in zip(l.tcp_min_m, l.tcp_max_m,
                    s.startup_tcp_min_m, s.startup_tcp_max_m))
                or not all(callable(c) for c in (self.authorize, self.hot_check, self.write_check))
                or self.read_check is not None and not callable(self.read_check)):
            raise ValueError('native_jog_scope_limits_or_callbacks_invalid')
        object.__setattr__(self, '_guard', _SiteRegionGuard(s, l, self.config_sha256,
            s.teleop_ready_q, self.reference_tool, (.005,)*6, (.001,)*6, .08))

    def _binding(self):
        self._guard.require_binding(self.settings, self.limits, self.config_sha256)

    def check(self):
        self._binding(); permission(self.authorize)

    def hot(self):
        self._binding(); permission(self.hot_check)

    def read(self):
        self._binding()
        permission(self.hot_check if self.read_check is None else self.read_check)

    def write(self):
        self._binding(); permission(self.write_check)

    def model_guard(self):
        self._binding()
        return self._guard


class SiteJogDiagnosticAdapter(_SingleJogSubmission):
    def __init__(self, scope, *, client_factory, clock, sleep, event_callback):
        if type(scope) is not SiteJogScope or not callable(client_factory):
            raise ValueError('native_jog_explicit_scope_and_factory_required')
        scope.check()
        self._site_scope = scope
        self._site_factory_used = False
        self._site_snapshot_tasks = set()
        self._site_authority_error = None
        self._runtime_precheck_requested = False
        self._runtime_precheck_issued = False
        self._runtime_precheck_consumed = False
        self._runtime_precheck = None

        async def factory(ip):
            scope.check()
            if self._site_factory_used or ip != scope.settings.ip:
                raise ValueError('native_jog_connection_consumed_or_changed')
            self._site_factory_used = True
            client = await client_factory(ip)
            if (getattr(client, '_jog_scope', None) is not scope
                    or not callable(getattr(client, '_native_motion_entry_count', None))):
                raise ValueError('native_jog_untrusted_client')
            return client

        super().__init__(scope.settings, client_factory=factory, clock=clock, pvat_clock=clock,
            sleep=sleep, event_callback=event_callback, backend_label='LEBAI_FAKE')
        self._initialize_jog(scope.settings, scope.limits, scope.config_sha256, scope.model_guard())

    async def connect(self):
        self._site_scope.check()
        self._require_model_binding()
        await super().connect()

    def _require_model_binding(self):
        # Submission authority checks the complete runtime once per boundary;
        # binding validation must not repeat the same filesystem scan.
        self._site_scope._binding()
        if self.settings is not self._site_scope.settings:
            raise ValueError('native_jog_configuration_changed')
        super()._require_model_binding()

    def _runtime_binding(self):
        return _RuntimePrecheck(self,self._site_scope,self.settings,self._jog_limits,
            self._jog_sha,self._client,asyncio.current_task(),self._pump.generation,
            self._snapshot_calls,self._snapshot_read_sequence,
            self._client.native_reads,self._client.read_entries)

    def precheck_jog_runtime(self):
        """One metadata scan before the unique FK/fresh-snapshot pair.

        This is no motion permission and cannot replace the proxy's actual
        native write check. Unused/failed proofs never permit fallback.
        """
        from app.commissioning.native_jog_site_client import SiteJogClient
        if self._runtime_precheck_requested or self._jog_consumed:
            self._runtime_precheck = None
            raise ValueError('native_jog_precheck_consumed')
        self._runtime_precheck_requested = True
        if (type(self._client) is not SiteJogClient or self._client._feedback_pair is not None
                or asyncio.current_task() is not self._client.owner
                or not self._pump.is_current(self._pump.generation)
                or self._snapshot_calls.closed):
            raise ValueError('native_jog_precheck_not_admitted')
        self._client.require_clear_read_gate()
        before = self._runtime_binding()
        self._site_scope.hot()
        if self._runtime_binding() != before:
            raise ValueError('native_jog_precheck_binding_changed')
        self._client.require_clear_read_gate()
        self._runtime_precheck = before
        self._runtime_precheck_issued = True

    def _consume_runtime_precheck(self):
        proof,self._runtime_precheck = self._runtime_precheck,None
        self._runtime_precheck_consumed = True
        if (type(proof) is not _RuntimePrecheck
                or proof.adapter is not self
                or proof.scope is not self._site_scope or proof.settings is not self.settings
                or proof.limits is not self._jog_limits or proof.config_sha256 != self._jog_sha
                or proof.client is not self._client or proof.owner is not asyncio.current_task()
                or self._client.owner is not proof.owner
                or proof.generation != self._pump.generation
                or not self._pump.is_current(proof.generation)
                or proof.snapshot_owner is not self._snapshot_calls or self._snapshot_calls.closed
                or proof.read_sequence+1 != self._snapshot_read_sequence):
            raise ValueError('native_jog_precheck_binding_changed')
        client = self._client
        client.require_clear_read_gate()
        pair = client._feedback_pair
        read = self._latest_snapshot_read
        group = pair.get('snapshot_group') if pair is not None else None
        if (pair is None or pair['owner'] is not proof.owner
                or pair['active'] or pair['report']['outcome'] != 'returned'
                or group is None or not pair.get('snapshot_returned')
                or read is None or read.sequence != self._snapshot_read_sequence
                or self._snapshot is None or self._snapshot.read_purpose != 'motion'
                or read.snapshot is not self._snapshot
                or client.native_reads is not proof.native_reads):
            raise ValueError('native_jog_precheck_pair_not_matching')
        # Exactly the paired FK plus that group's actual native fields. This
        # also fences direct raw reads which do not change adapter sequence.
        methods = set(group.get('native_used',())) | {'kinematics_forward'}
        rows = client.native_reads[proof.read_entries:]
        if (client.read_entries != proof.read_entries+len(methods)
                or len(rows) != len(methods) or {r['method'] for r in rows} != methods
                or any(r['outcome'] != 'returned' or r['remote_result_unknown'] for r in rows)):
            raise ValueError('native_jog_precheck_extra_read')

    def runtime_precheck_report(self):
        return dict(policy=RUNTIME_PRECHECK_POLICY,requested=self._runtime_precheck_requested,
            issued=self._runtime_precheck_issued,consumed=self._runtime_precheck_consumed)

    async def submit_jog(self, packet, *, authorize):
        if self._jog_consumed:
            raise BackendCommandError('native_jog_submission_consumed')
        if (packet.velocity_mps != (0., 0., .005) or packet.acceleration_mps2 != .01
                or packet.duration_s != .08):
            self._jog_consumed = True
            raise ValueError('native_jog_fixed_packet_required')
        def check():
            try:
                # Polling ownership is not a new file-entry boundary. Keep
                # input/expiry checks here; the proxy checks metadata again
                # immediately before native speedl, even after lock waiting.
                self._site_scope.read()
                permission(authorize)
            except Exception as error:
                if self._site_authority_error is None:self._site_authority_error=error
                raise
        try:
            try:
                if self._runtime_precheck_requested:
                    self._consume_runtime_precheck()
                else:
                    self._site_scope.hot()  # Legacy direct callers retain their scan.
            except Exception as error:
                self._jog_consumed=True
                if self._site_authority_error is None:self._site_authority_error=error
                raise
            result=await super().submit_jog(packet, authorize=check)
            self._site_scope.hot()
            return result
        except BackendCommandError:
            # The owner deliberately collapses revoked permission to a bool.
            # Keep its local reason when trusted evidence proves no SDK entry.
            if (self._client is not None and self._site_authority_error is not None
                    and self._client._native_motion_entry_count('speedl') == 0):
                raise self._site_authority_error from None
            raise
        finally:
            # Only trusted native entry creates a stop obligation.
            if self._client is not None:
                self.native_jog_entries = self._client._native_motion_entry_count('speedl')
                self._motion_write_stop_required = self.native_jog_entries > 0

    async def _read_snapshot_impl(self, read, **kwargs):
        if kwargs['read_purpose'] not in ('preflight','motion'):
            return await super()._read_snapshot_impl(read, **kwargs)
        self._site_scope.hot()
        group = getattr(self._client, '_parallel_snapshot_scope', None)
        with group(self._site_scope._binding,purpose=kwargs['read_purpose']) if callable(group) else nullcontext():
            return await super()._read_snapshot_impl(read, **kwargs)

    def _before_snapshot_install(self, snapshot, read):
        if snapshot.read_purpose in ('preflight','motion'):
            self._site_scope.hot()
            self._ensure_command_snapshot_fresh(snapshot)

    async def _read_snapshot_fields(self, client, latencies, deadline_ns, **kwargs):
        if kwargs['read_purpose'] not in ('preflight','motion'):
            return await super()._read_snapshot_fields(client, latencies, deadline_ns, **kwargs)
        async def query(name):
            self._site_scope._binding()
            value = await self._timed(name, getattr(client, name), latencies, deadline_ns=deadline_ns)
            return value, (copy.deepcopy(dict(value)) if name == 'get_kin_data'
                and kwargs['include_raw_kinematics'] and isinstance(value, Mapping) else None)
        tasks = [asyncio.create_task(query(n)) for n in
            ('get_robot_state','get_estop_reason','get_kin_data','get_tcp','get_claw','get_running_motion')]
        self._site_snapshot_tasks.update(tasks)
        def complete(task):
            self._site_snapshot_tasks.discard(task)
            if not task.cancelled(): task.exception()
        for task in tasks: task.add_done_callback(complete)
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_EXCEPTION)
            values = [t.result() for t in tasks]
            self._site_scope._binding()
            return dict(values=[v[0] for v in values], raw_kinematics=values[2][1])
        except BaseException:
            for task in tasks:
                if not task.done() and not task.cancelling(): task.cancel()
            raise
