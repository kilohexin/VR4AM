"""Host-timed load comparison with a hardcoded fake SDK; never a live entry."""
import argparse
import asyncio
import json
import math
from pathlib import Path
import statistics
from time import perf_counter_ns

from app.robots.lebai_observer_window import ObserverWindow
from app.robots.lebai_pump import PvatRequest
from tests.robots.test_lebai_adapter_control import _connected_control_adapter, _target


# Artificial per-call delays, not a fit of controller/network behavior.
PROFILES = {'baseline': (0, 0, 0), 'moderate': (.003, .006, .002),
            'read_heavy': (.007, .003, .002), 'ik_heavy': (.001, .025, .002)}


class HostClock:
    now_ns = staticmethod(perf_counter_ns)
    sleep = staticmethod(asyncio.sleep)


def summarize(values):
    values = sorted(values)
    return {'count': len(values), 'median_ms': statistics.median(values) if values else None,
            'p95_ms': values[math.ceil(len(values) * .95) - 1] if values else None,
            'max_ms': max(values) if values else None,
            'over_80ms': sum(x > 80 for x in values)}


async def run_case(profile, budget_s, *, samples=12, warmup=2):
    if (profile not in PROFILES or type(samples) is not int or not 1 <= samples <= 100
            or type(warmup) is not int or not 0 <= warmup <= 10):
        raise ValueError('invalid_probe_inputs')
    window = ObserverWindow(budget_s)  # Validate before creating any fake client.
    adapter, client, clock = await _connected_control_adapter(clock=HostClock())
    adapter._observer_window = window
    read_delay, ik_delay, send_delay = PROFILES[profile]
    originals = {}
    starts, handler_times, observation_records, events = [], [], [], []
    observation_errors = []
    stop_observer = asyncio.Event()
    observer = None
    sampling_error = cleanup_error = None
    measurement_start = measurement_end = None

    def delayed(operation, delay, mark_send=False):
        async def call(*args):
            if mark_send:
                starts.append(perf_counter_ns())
            if delay:
                await asyncio.sleep(delay)
            return await operation(*args)
        return call

    async def record(event, timestamp):
        events.append(event)

    async def observe():
        while not stop_observer.is_set():
            started = perf_counter_ns()
            try:
                await adapter.get_state()
            except Exception as exc:
                observation_errors.append(str(exc))
                return
            observation_records.append((started, (perf_counter_ns() - started) / 1e6))
            await asyncio.sleep(.040)

    try:
        for name in ['get_robot_state', 'get_estop_reason', 'get_kin_data',
                     'get_tcp', 'get_claw', 'get_running_motion']:
            originals[name] = getattr(client, name)
            setattr(client, name, delayed(originals[name], read_delay))
        for name, delay in [('kinematics_inverse', ik_delay), ('move_pvat', send_delay)]:
            originals[name] = getattr(client, name)
            setattr(client, name, delayed(originals[name], delay, name == 'move_pvat'))
        adapter._event_callback = record
        observer = asyncio.create_task(observe())
        loop = asyncio.get_running_loop()
        deadline = loop.time()
        for command_id in range(samples + warmup):
            client.ik_results.append(list(client.kin_data['actual_joint_pose']))
            started = perf_counter_ns()
            if command_id == warmup:
                measurement_start = started
            try:
                await adapter._send_target(PvatRequest(_target(), command_id, adapter._pump.generation))
            except Exception as exc:
                sampling_error = str(exc)
                break  # Do not retry failures or count absent sends as success.
            handler_times.append((perf_counter_ns() - started) / 1e6)
            deadline = max(deadline + .040, loop.time())
            await asyncio.sleep(max(0, deadline - loop.time()))
    finally:
        measurement_end = perf_counter_ns()
        stop_observer.set()
        if observer is not None:
            try:
                await asyncio.wait_for(observer, 2)
            except Exception as exc:
                observation_errors.append('observer_teardown:' + str(exc))
        # Restore delay wrappers before the fake SDK's ordinary stop/teardown.
        for name, operation in originals.items():
            setattr(client, name, operation)
        adapter._event_callback = None
        try:
            await adapter.disconnect()
        except Exception as exc:
            cleanup_error = str(exc)

    measured_starts = starts[warmup:]
    gaps = [(b - a) / 1e6 for a, b in zip(measured_starts, measured_starts[1:])]
    waits = []
    def in_measurement(timestamp):
        return (measurement_start is not None and measurement_end is not None
                and measurement_start <= timestamp <= measurement_end)

    observation_times = [elapsed for started, elapsed in observation_records
                         if in_measurement(started)]
    for event in events:
        timing = event.get('diagnostic_timing', {}).get('observer_window')
        if (timing and timing['requested_ns'] is not None and timing['completed_ns'] is not None
                and in_measurement(timing['requested_ns'])):
            waits.append((timing['completed_ns'] - timing['requested_ns']) / 1e6)
    sampling_complete = sampling_error is None and len(measured_starts) == samples
    return {'offline_only': True, 'hardware_verified': False, 'profile': profile,
            'budget_s': budget_s, 'artificial_delay_s': PROFILES[profile],
            'requested_samples': samples, 'warmup_samples': warmup,
            'motion_samples': len(measured_starts), 'send_gaps': summarize(gaps),
            'handler_times': summarize(handler_times[warmup:]),
            'observation_count': len(observation_times),
            'observation_total_count': len(observation_records),
            'observation_times': summarize(observation_times), 'window_wait': summarize(waits),
            'sampling_error': sampling_error, 'observation_errors': observation_errors,
            'cleanup_error': cleanup_error, 'sampling_complete': sampling_complete,
            'comparison_eligible': (sampling_complete and not observation_errors
                                    and cleanup_error is None and bool(observation_times)
                                    and bool(gaps)),
            'send_gaps_ms': gaps, 'observation_times_ms': observation_times,
            'window_wait_ms': waits}


async def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=12)
    parser.add_argument('--repetitions', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.samples <= 100 or not 1 <= args.repetitions <= 10:
        parser.error('samples must be 1..100 and repetitions 1..10')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    runs = []
    budgets = [0.0, .005, .020]
    for repeat in range(args.repetitions):
        for profile in PROFILES:
            order = budgets[repeat % 3:] + budgets[:repeat % 3]
            for budget in order:
                result = await run_case(profile, budget, samples=args.samples)
                result['repetition'] = repeat
                runs.append(result)
                (args.output_dir / 'results.json').write_text(json.dumps({
                    'offline_only': True, 'hardware_verified': False,
                    'host_timing_not_controller_timing': True, 'runs': runs,
                }, indent=2) + '\n', encoding='utf-8')
                print(f"{profile} wait={budget * 1000:g}ms repeat={repeat} "
                      f"samples={result['motion_samples']} error={result['sampling_error']}", flush=True)


if __name__ == '__main__':
    asyncio.run(main())
