"""Extract a public TCP replay from a complete, locally verified comparison."""

import argparse
import json
from pathlib import Path

import numpy as np


def select(rows, stride, extra=()):
    indices = sorted(set(range(0, len(rows), stride)) | {len(rows) - 1, *extra})
    return [rows[i] for i in indices]


def summarize(replay, demo):
    if (replay['schema'] != 'lm3-physical-replay-v1' or replay['source'] != 'physical'
            or replay['pathHash'] != demo['pathHash'] or replay['goalError'] is None):
        raise ValueError('A complete, same-trajectory physical replay is required')
    actual, simulated = replay['frames'], demo['mujoco']
    actual_t, sim_t = [np.array([frame['t'] for frame in rows]) for rows in (actual, simulated)]
    duration = demo['planDuration']
    if min(actual_t[-1], sim_t[-1]) < duration or max(actual_t[0], sim_t[0]) > 0:
        raise ValueError('The physical and simulated records must cover the full trajectory')
    times = np.union1d(actual_t, sim_t)
    times = np.unique(np.r_[0, times[(0 < times) & (times < duration)], duration])

    def tcp_at(rows, stamps):
        return np.column_stack([np.interp(times, stamps, axis) for axis in np.array([f['p'] for f in rows]).T])

    difference = tcp_at(actual, actual_t) - tcp_at(simulated, sim_t)
    a, b = difference[:-1], difference[1:]
    squared = np.sum(np.diff(times)[:, None] * (a*a + a*b + b*b) / 3) / duration
    comparisons = {}
    for name in ('mujoco', 'docker'):
        item = replay['comparisons'][name]
        peak_index = max(range(len(item['errors'])), key=lambda i: item['errors'][i][1])
        comparisons[name] = {key: item[key] for key in ('peak', 'rawPeak', 'settled')}
        comparisons[name]['errors'] = select(item['errors'], 4, (peak_index,))
    return dict(schema='lm3-public-replay-v1', pathHash=demo['pathHash'], recordedOn=replay['recordedAt'][:10],
        planDuration=duration, captureSeconds=float(actual_t[-1]+replay['onsetOffsetMs']/1000),
        sampleCount=len(actual), onsetOffsetMs=replay['onsetOffsetMs'], maxGapMs=replay['maxGapMs'],
        goalError=replay['goalError'], motionRmse=float(1000*np.sqrt(squared)), comparisons=comparisons,
        frames=[dict(t=round(f['t'], 6), p=[round(v, 7) for v in f['p']]) for f in select(actual, 4)])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('comparison', type=Path)
    parser.add_argument('simulation', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    js = args.simulation.read_text(encoding='utf-8').strip()
    demo = json.loads(js.removeprefix('window.LM3_DEMO = ').removesuffix(';'))
    data = summarize(json.loads(args.comparison.read_bytes()), demo)
    with args.output.open('x', encoding='utf-8') as handle:
        json.dump(data, handle, allow_nan=False, separators=(',', ':'))
        handle.write('\n')


if __name__ == '__main__':
    main()
