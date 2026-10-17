"""Process-scoped Windows wait resolution for the offline demo entry only.

QPC measures time precisely but does not shorten the OS wait quantum. This
request is balanced on all exits and does not make Windows a real-time OS.
https://learn.microsoft.com/windows/win32/api/timeapi/nf-timeapi-timebeginperiod
"""
from contextlib import contextmanager
import sys


def _windows_timer_api():
    version=sys.getwindowsversion()
    # Older releases apply this API globally rather than to this process.
    if version.major<10 or version.build<19041:
        raise RuntimeError('offline_timer_requires_windows_10_2004')
    import ctypes
    from ctypes import wintypes
    library=ctypes.WinDLL('winmm')
    begin,end=library.timeBeginPeriod,library.timeEndPeriod
    for method in (begin,end):
        method.argtypes=[wintypes.UINT]
        method.restype=wintypes.UINT
    return begin,end


@contextmanager
def offline_timer_period():
    if sys.platform!='win32':
        yield
        return
    begin,end=_windows_timer_api()
    if begin(1)!=0:
        raise RuntimeError('offline_timer_request_failed')
    try:
        yield
    finally:
        if end(1)!=0:
            raise RuntimeError('offline_timer_release_failed')
