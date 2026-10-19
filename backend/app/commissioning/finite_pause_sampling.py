"""Separate single-window pause scope with start-period stop sampling.

This runner requires the standalone caller's bound approval and trusted
dependencies. It is not selected by the original pause CLI or normal API.
"""
from app.commissioning.finite_pause_window import validate_settings
from app.commissioning.finite_window_run import _run_window


async def run_sampling_pause_window(settings, recorder, client_factory, authorization_check, hot_check,
                                    *, clock, write_check, **kwargs):
    if '_site_parallel_stop_reads' in kwargs:
        raise ValueError('finite_pause_parallel_scope_required')
    validate_settings(settings)
    if not all(callable(v) for v in (clock, write_check, client_factory, authorization_check, hot_check)):
        raise TypeError('finite_pause_sampling_explicit_checks_required')
    if kwargs.get('deadline_clock') != clock:
        raise ValueError('finite_pause_sampling_requires_one_clock')
    return await _run_window(settings, recorder, client_factory, authorization_check, hot_check,
        clock=clock, write_check=write_check, pause_comparison=True, offline_pause_comparison=False,
        _site_pause_start_period=True, **kwargs)
