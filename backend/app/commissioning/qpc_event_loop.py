"""Explicit local high-resolution loop; never installs a global policy.

CPython 3.12 on Windows normally schedules against GetTickCount64. Its
15.625-ms resolution permits callbacks to run early by that amount. This
isolated diagnostic loop uses QPC for both deadlines and scheduler resolution.
This does not make Windows real time or prove a robot stopped.
"""
import asyncio
import os
import platform
import time


_Base = asyncio.ProactorEventLoop if os.name == 'nt' else asyncio.SelectorEventLoop


class QpcEventLoop(_Base):
    vr4arm_timing_clock = 'time.perf_counter_ns'

    def __init__(self):
        if platform.python_implementation()!='CPython':
            raise RuntimeError('qpc_loop_requires_cpython')
        info=time.get_clock_info('perf_counter')
        if not info.monotonic or info.adjustable or not 0<info.resolution<=.000001:
            raise RuntimeError('qpc_clock_unavailable')
        super().__init__()
        # CPython implementation detail, isolated to this newly created loop.
        self._clock_resolution=info.resolution

    def time(self):
        return time.perf_counter()


def new_qpc_loop():
    return QpcEventLoop()
