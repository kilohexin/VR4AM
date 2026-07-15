const DEFAULT_CAPACITY = 512;

export class LatencyTracker {
  private readonly samples: number[] = [];

  constructor(private readonly capacity = DEFAULT_CAPACITY) {
    if (!Number.isInteger(capacity) || capacity < 1) {
      throw new Error('延迟窗口容量必须是正整数');
    }
  }

  get count(): number {
    return this.samples.length;
  }

  get current(): number | null {
    return this.samples.at(-1) ?? null;
  }

  add(milliseconds: number): void {
    if (!Number.isFinite(milliseconds) || milliseconds < 0) return;
    if (this.samples.length === this.capacity) this.samples.shift();
    this.samples.push(milliseconds);
  }

  reset(): void {
    this.samples.length = 0;
  }

  p95(): number | null {
    if (this.samples.length === 0) return null;
    const sorted = [...this.samples].sort((a, b) => a - b);
    const index = Math.ceil(0.95 * sorted.length) - 1;
    return sorted[index];
  }
}
