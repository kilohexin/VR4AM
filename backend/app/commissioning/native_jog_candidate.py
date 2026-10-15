"""One injected, IPv4-loopback-only speedl candidate; no production selector.

No continuous mode: native_jog_joint_bounds_unverified. This adapter supplies
neither physical stopping evidence nor a real robot execution entry.
"""
from collections.abc import Callable

from app.commissioning.native_jog_packet import NativeJogLimits, validate_packet, _sha
from app.robots.base import BackendCommandError
from app.commissioning.native_jog_core import _SingleJogSubmission
from app.robots.lebai_motion_calls import OwnedMotionCalls
from app.commissioning.native_jog_model_guard import ModelJogRegionGuard


class NativeJogCandidateAdapter(_SingleJogSubmission):
    def __init__(self, settings, *, limits: NativeJogLimits, config_sha256: str,
                 client_factory, clock, sleep, event_callback, model_region_guard=None):
        if settings.ip != '127.0.0.1' or not callable(client_factory):
            raise ValueError('explicit injected IPv4 loopback factory required')
        _sha(config_sha256)
        if model_region_guard is not None:
            if type(model_region_guard) is not ModelJogRegionGuard:
                raise ValueError('explicit model region guard required')
            model_region_guard.require_binding(settings, limits, config_sha256)
        if (type(limits) is not NativeJogLimits
                or limits.max_speed_mps > settings.control.max_tcp_speed_mps
                or limits.max_acceleration_mps2 > settings.control.max_tcp_acceleration_mps2
                or any(lo < allowed_lo or hi > allowed_hi for lo,hi,allowed_lo,allowed_hi in zip(
                    limits.tcp_min_m, limits.tcp_max_m,
                    settings.startup_tcp_min_m, settings.startup_tcp_max_m))):
            raise ValueError('candidate limits exceed configured limits')
        super().__init__(settings, client_factory=client_factory, clock=clock,
            pvat_clock=clock, sleep=sleep, event_callback=event_callback, backend_label='LEBAI_FAKE')
        self._initialize_jog(settings, limits, config_sha256, model_region_guard)

    async def connect(self):
        if self.settings is not self._jog_settings or self.settings.ip != '127.0.0.1':
            raise BackendCommandError('loopback_configuration_changed')
        self._require_model_binding()
        await super().connect()

