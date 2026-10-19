"""Separately admitted single 50 um pause diagnostic; no normal API selector.

The standalone CLI binds the approval before connecting. The explicit callback
and factory are trusted dependencies, not a network or authorization sandbox.
Pause can retain a trajectory; this module grants no resume or second target.
"""
import ipaddress
import math

from app.commissioning.finite_window_run import _run_window
from app.commissioning.readonly_kinematics import computational_distance

DISTANCE_M = .00005


def validate_settings(settings):
    if settings.mode != 'control' or computational_distance(settings) != DISTANCE_M:
        raise ValueError('finite_pause_requires_control_50um_window')
    ip = ipaddress.IPv4Address(settings.ip)
    if str(ip) != settings.ip or not ip.is_private or ip.is_loopback or ip.is_multicast or ip.is_unspecified:
        raise ValueError('finite_pause_requires_literal_private_ipv4')
    for name, cap in (('max_joint_speed_radps', .05), ('max_joint_acceleration_radps2', .2),
                      ('pvat_horizon_s', .08)):
        value = getattr(settings.control, name)
        if not math.isfinite(value) or not 0 < value <= cap:
            raise ValueError('finite_pause_cap_invalid:' + name)
    if settings.control.pvat_horizon_s < .01:
        raise ValueError('finite_pause_horizon_invalid')


async def run_pause_window(settings, recorder, client_factory, authorization_check, hot_check,
                           *, clock, write_check, **kwargs):
    if '_site_parallel_stop_reads' in kwargs:
        raise ValueError('finite_pause_parallel_scope_required')
    if '_site_pause_start_period' in kwargs:
        raise ValueError('finite_pause_sampling_scope_required')
    validate_settings(settings)
    if not all(callable(v) for v in (clock, write_check, client_factory, authorization_check, hot_check)):
        raise TypeError('finite_pause_explicit_checks_required')
    if kwargs.get('deadline_clock') != clock:
        raise ValueError('finite_pause_requires_one_clock')
    return await _run_window(settings, recorder, client_factory, authorization_check, hot_check,
        clock=clock, write_check=write_check, pause_comparison=True, offline_pause_comparison=False, **kwargs)
