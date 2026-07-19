import {describe, expect, it} from 'vitest';
import {
  TABLE_HEIGHT_STORAGE_KEY,
  TableHeightController,
} from '../src/scenes/tableHeightController';

class MemoryStorage implements Storage {
  private readonly values = new Map<string, string>();

  get length(): number {
    return this.values.size;
  }

  clear(): void {
    this.values.clear();
  }

  getItem(key: string): string | null {
    return this.values.get(key) ?? null;
  }

  key(index: number): string | null {
    return [...this.values.keys()][index] ?? null;
  }

  removeItem(key: string): void {
    this.values.delete(key);
  }

  setItem(key: string, value: string): void {
    this.values.set(key, value);
  }
}

const sample = (overrides: Partial<Parameters<TableHeightController['update']>[0]> = {}) => ({
  headY: 1.67,
  axisY: 0,
  resetPressed: false,
  enabled: true,
  nowMs: 100,
  ...overrides,
});

describe('TableHeightController', () => {
  it('uses the first valid head height as its baseline and integrates at 0.35 m/s', () => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();

    expect(control.update(sample())).toBeCloseTo(0.95);

    for (let step = 1; step <= 10; step += 1) {
      control.update(sample({axisY: -1, nowMs: 100 + step * 100}));
    }

    expect(control.heightM).toBeCloseTo(1.30);
  });

  it.each([
    [0.5, 0.55],
    [3, 1.10],
  ])('clamps a head baseline from %s m to %s m', (headY, expected) => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();

    expect(control.update(sample({headY}))).toBeCloseTo(expected);
    expect(control.update(sample({headY: 1.67, nowMs: 200}))).toBeCloseTo(expected);
  });

  it('treats the inclusive 0.2 axis dead zone as neutral', () => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();
    control.update(sample());

    control.update(sample({axisY: 0.2, nowMs: 200}));
    control.update(sample({axisY: -0.2, nowMs: 300}));

    expect(control.manualOffsetM).toBe(0);
  });

  it('clamps elapsed time to 0.1 seconds', () => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();
    control.update(sample());

    expect(control.update(sample({axisY: -1, nowMs: 10_100}))).toBeCloseTo(0.985);
    expect(control.manualOffsetM).toBeCloseTo(0.035);
  });

  it('clamps manual offset to plus or minus 0.6 m', () => {
    const raise = new TableHeightController(new MemoryStorage());
    raise.beginSession();
    raise.update(sample());
    for (let step = 1; step <= 30; step += 1) {
      raise.update(sample({axisY: -1, nowMs: 100 + step * 100}));
    }

    const lower = new TableHeightController(new MemoryStorage());
    lower.beginSession();
    lower.update(sample());
    for (let step = 1; step <= 30; step += 1) {
      lower.update(sample({axisY: 1, nowMs: 100 + step * 100}));
    }

    expect(raise.manualOffsetM).toBeCloseTo(0.6);
    expect(lower.manualOffsetM).toBeCloseTo(-0.6);
  });

  it('clamps final height to the 0.30-1.40 m safety range', () => {
    const highStorage = new MemoryStorage();
    highStorage.setItem(TABLE_HEIGHT_STORAGE_KEY, '0.6');
    const high = new TableHeightController(highStorage);
    high.beginSession();

    const lowStorage = new MemoryStorage();
    lowStorage.setItem(TABLE_HEIGHT_STORAGE_KEY, '-0.6');
    const low = new TableHeightController(lowStorage);
    low.beginSession();

    expect(high.update(sample({headY: 3}))).toBeCloseTo(1.40);
    expect(low.update(sample({headY: 0.5}))).toBeCloseTo(0.30);
  });

  it('requires a neutral axis sample after disabled input before resuming', () => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();
    control.update(sample());

    control.update(sample({axisY: -1, enabled: false, nowMs: 200}));
    control.update(sample({axisY: -1, nowMs: 300}));
    expect(control.manualOffsetM).toBe(0);

    control.update(sample({axisY: 0.2, nowMs: 400}));
    control.update(sample({axisY: -1, nowMs: 500}));
    expect(control.manualOffsetM).toBeCloseTo(0.035);
  });

  it('keeps the neutral gate when disabled input arrives before a valid head baseline', () => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();

    control.update(sample({headY: null, axisY: -1, enabled: false}));
    control.update(sample({axisY: -1, nowMs: 200}));
    control.update(sample({axisY: -1, nowMs: 300}));
    expect(control.manualOffsetM).toBe(0);

    control.update(sample({axisY: 0, nowMs: 400}));
    control.update(sample({axisY: -1, nowMs: 500}));
    expect(control.manualOffsetM).toBeCloseTo(0.035);
  });

  it('resets once on a press edge only after a released sample has armed it', () => {
    const storage = new MemoryStorage();
    storage.setItem(TABLE_HEIGHT_STORAGE_KEY, '0.4');
    const control = new TableHeightController(storage);
    control.beginSession();

    control.update(sample({resetPressed: true}));
    expect(control.manualOffsetM).toBeCloseTo(0.4);

    control.update(sample({resetPressed: false, nowMs: 200}));
    control.update(sample({resetPressed: true, nowMs: 300}));
    expect(control.manualOffsetM).toBe(0);

    control.update(sample({axisY: -1, resetPressed: true, nowMs: 400}));
    expect(control.manualOffsetM).toBeCloseTo(0.035);
  });

  it('does not integrate a deflected axis on the same sample as a reset edge', () => {
    const storage = new MemoryStorage();
    storage.setItem(TABLE_HEIGHT_STORAGE_KEY, '0.4');
    const control = new TableHeightController(storage);
    control.beginSession();
    control.update(sample({resetPressed: false}));

    control.update(sample({axisY: -1, resetPressed: true, nowMs: 200}));

    expect(control.manualOffsetM).toBe(0);
  });

  it('arms reset from the neutral sample that clears the disabled gate', () => {
    const storage = new MemoryStorage();
    storage.setItem(TABLE_HEIGHT_STORAGE_KEY, '0.4');
    const control = new TableHeightController(storage);
    control.beginSession();
    control.update(sample({enabled: false}));

    control.update(sample({resetPressed: false, nowMs: 200}));
    control.update(sample({resetPressed: true, nowMs: 300}));

    expect(control.manualOffsetM).toBe(0);
  });

  it.each(['not-a-number', 'NaN', 'Infinity', '-Infinity']) (
    'falls back to zero for invalid stored offset %s',
    (stored) => {
      const storage = new MemoryStorage();
      storage.setItem(TABLE_HEIGHT_STORAGE_KEY, stored);
      const control = new TableHeightController(storage);

      control.beginSession();

      expect(control.manualOffsetM).toBe(0);
    },
  );

  it.each([
    ['0.9', 0.6],
    ['-0.9', -0.6],
  ])('clamps finite stored offset %s to %s', (stored, expected) => {
    const storage = new MemoryStorage();
    storage.setItem(TABLE_HEIGHT_STORAGE_KEY, stored);
    const control = new TableHeightController(storage);

    control.beginSession();

    expect(control.manualOffsetM).toBeCloseTo(expected);
  });

  it('ignores non-finite head, axis, and timestamp inputs without corrupting state', () => {
    const control = new TableHeightController(new MemoryStorage());
    control.beginSession();

    expect(control.update(sample({headY: Number.NaN}))).toBe(0);
    expect(control.update(sample({headY: Number.POSITIVE_INFINITY}))).toBe(0);
    expect(control.update(sample({headY: 1.67, nowMs: Number.NaN}))).toBeCloseTo(0.95);
    expect(control.update(sample({axisY: Number.NaN, nowMs: 200}))).toBeCloseTo(0.95);
    expect(control.update(sample({axisY: -1, nowMs: Number.POSITIVE_INFINITY}))).toBeCloseTo(0.95);
    expect(control.manualOffsetM).toBe(0);
    expect(control.update(sample({axisY: -1, nowMs: 300}))).toBeCloseTo(0.985);
  });

  it('contains storage read and write exceptions so XR updates continue', () => {
    const storage = {
      getItem: () => { throw new DOMException('read denied'); },
      setItem: () => { throw new DOMException('write denied'); },
    } as unknown as Storage;
    const control = new TableHeightController(storage);

    expect(() => control.beginSession()).not.toThrow();
    expect(control.update(sample())).toBeCloseTo(0.95);
    expect(() => control.update(sample({axisY: -1, nowMs: 200}))).not.toThrow();
    expect(control.manualOffsetM).toBeCloseTo(0.035);
  });

  it('persists only manual offset changes under the stable storage key', () => {
    const writes: Array<[string, string]> = [];
    const storage = {
      getItem: () => null,
      setItem: (key: string, value: string) => writes.push([key, value]),
    } as unknown as Storage;
    const control = new TableHeightController(storage);
    control.beginSession();

    control.update(sample());
    control.update(sample({axisY: 0.2, nowMs: 200}));
    control.update(sample({axisY: -1, nowMs: 300}));

    expect(writes).toEqual([[TABLE_HEIGHT_STORAGE_KEY, '0.035']]);
  });

  it('begins and ends sessions without deleting the saved manual offset', () => {
    const storage = new MemoryStorage();
    storage.setItem(TABLE_HEIGHT_STORAGE_KEY, '0.25');
    const control = new TableHeightController(storage);

    expect(control.update(sample())).toBe(0);
    control.beginSession();
    expect(control.update(sample())).toBeCloseTo(1.20);
    expect(control.endSession()).toBe(0);
    expect(control.heightM).toBe(0);
    expect(control.manualOffsetM).toBeCloseTo(0.25);
    expect(storage.getItem(TABLE_HEIGHT_STORAGE_KEY)).toBe('0.25');

    control.beginSession();
    expect(control.heightM).toBe(0);
    expect(control.update(sample({nowMs: 500}))).toBeCloseTo(1.20);
  });
});
