import {describe, expect, it} from 'vitest';
import {LatencyTracker} from '../src/ui/latency';

describe('LatencyTracker', () => {
  it('reports p95 over a bounded 512-sample window', () => {
    const tracker = new LatencyTracker(512);

    for (let sample = 1; sample <= 600; sample += 1) tracker.add(sample);

    expect(tracker.count).toBe(512);
    expect(tracker.current).toBe(600);
    expect(tracker.p95()).toBe(575);
  });

  it('sorts a copy and uses ceil(0.95*n)-1 without mutating insertion order', () => {
    const tracker = new LatencyTracker(4);
    tracker.add(40);
    tracker.add(10);
    tracker.add(30);
    tracker.add(20);

    expect(tracker.p95()).toBe(40);
    expect(tracker.current).toBe(20);
    expect(new LatencyTracker().p95()).toBeNull();
  });
});
