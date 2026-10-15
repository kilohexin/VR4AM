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
                or not all(callable(c) for c in (self.authorize, self.hot_check, self.write_check))):
            raise ValueError('native_jog_scope_limits_or_callbacks_invalid')
        object.__setattr__(self, '_guard', _SiteRegionGuard(s, l, self.config_sha256,
            s.teleop_ready_q, self.reference_tool, (.005,)*6, (.001,)*6, .08))

    def _binding(self):
        self._guard.require_binding(self.settings, self.limits, self.config_sha256)

    def check(self):
        self._binding(); permission(self.authorize)

    def hot(self):
        self._binding(); permission(self.hot_check)

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
        self._site_scope.hot()
        if self.settings is not self._site_scope.settings:
            raise ValueError('native_jog_configuration_changed')
        super()._require_model_binding()

    async def submit_jog(self, packet, *, authorize):
        if (packet.velocity_mps != (0., 0., .005) or packet.acceleration_mps2 != .01
                or packet.duration_s != .08):
            self._jog_consumed = True
            raise ValueError('native_jog_fixed_packet_required')
        def check():
            self._site_scope.hot()
            permission(authorize)
        try:
            return await super().submit_jog(packet, authorize=check)
        finally:
            # Only trusted native entry creates a stop obligation.
            if self._client is not None:
                self.native_jog_entries = self._client._native_motion_entry_count('speedl')
                self._motion_write_stop_required = self.native_jog_entries > 0

    async def _read_snapshot_impl(self, read, **kwargs):
        if kwargs['read_purpose'] != 'motion':
            return await super()._read_snapshot_impl(read, **kwargs)
        self._site_scope.hot()
        group = getattr(self._client, '_parallel_snapshot_scope', None)
        with group(self._site_scope.hot) if callable(group) else nullcontext():
            return await super()._read_snapshot_impl(read, **kwargs)

    def _before_snapshot_install(self, snapshot, read):
        if snapshot.read_purpose == 'motion':
            self._site_scope.hot()
            self._ensure_command_snapshot_fresh(snapshot)

    async def _read_snapshot_fields(self, client, latencies, deadline_ns, **kwargs):
        if kwargs['read_purpose'] != 'motion':
            return await super()._read_snapshot_fields(client, latencies, deadline_ns, **kwargs)
        async def query(name):
            self._site_scope.hot()
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
            self._site_scope.hot()
            return dict(values=[v[0] for v in values], raw_kinematics=values[2][1])
        except BaseException:
            for task in tasks:
                if not task.done() and not task.cancelling(): task.cancel()
            raise
