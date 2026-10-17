"""Offline Windows timer request is bounded by the server lifetime."""
import asyncio
from types import SimpleNamespace
import pytest


@pytest.mark.parametrize('error',[None,ValueError('serve failed'),asyncio.CancelledError(),KeyboardInterrupt()])
def test_period_is_released_on_every_server_exit(monkeypatch,error):
    from app.digital_twin import offline_timing as timing
    calls=[]
    monkeypatch.setattr(timing.sys,'platform','win32')
    def begin(value):calls.append(('begin',value));return 0
    def end(value):calls.append(('end',value));return 0
    monkeypatch.setattr(timing,'_windows_timer_api',lambda:(begin,end))
    def run():
        with timing.offline_timer_period():
            calls.append(('serve',None))
            if error is not None:raise error
    if error is None:run()
    else:
        with pytest.raises(type(error)):run()
    assert calls==[('begin',1),('serve',None),('end',1)]


def test_non_windows_never_loads_or_requests_a_windows_timer(monkeypatch):
    from app.digital_twin import offline_timing as timing
    monkeypatch.setattr(timing.sys,'platform','linux')
    def forbidden():raise AssertionError('Windows API loaded')
    monkeypatch.setattr(timing,'_windows_timer_api',forbidden)
    with timing.offline_timer_period():pass


def test_failed_acquisition_never_starts_server_or_releases_an_unowned_period(monkeypatch):
    from app.digital_twin import offline_timing as timing
    calls=[]
    monkeypatch.setattr(timing.sys,'platform','win32')
    monkeypatch.setattr(timing,'_windows_timer_api',lambda:(lambda n:1,lambda n:calls.append(n)))
    with pytest.raises(RuntimeError,match='offline_timer_request_failed'):
        with timing.offline_timer_period():pytest.fail('server started')
    assert calls==[]


def test_release_failure_is_reported(monkeypatch):
    from app.digital_twin import offline_timing as timing
    monkeypatch.setattr(timing.sys,'platform','win32')
    monkeypatch.setattr(timing,'_windows_timer_api',lambda:(lambda n:0,lambda n:1))
    with pytest.raises(RuntimeError,match='offline_timer_release_failed'):
        with timing.offline_timer_period():pass


def test_older_windows_global_timer_semantics_are_not_selected(monkeypatch):
    from app.digital_twin import offline_timing as timing
    monkeypatch.setattr(timing.sys,'getwindowsversion',lambda:SimpleNamespace(major=10,build=18363),raising=False)
    with pytest.raises(RuntimeError,match='offline_timer_requires_windows_10_2004'):
        timing._windows_timer_api()
