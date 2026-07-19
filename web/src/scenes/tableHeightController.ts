export const TABLE_HEIGHT_STORAGE_KEY = 'vr4arm.xr.tableHeightOffsetM';
export const TABLE_HEIGHT_BASE_MIN_M = 0.55;
export const TABLE_HEIGHT_BASE_MAX_M = 1.10;
export const TABLE_HEIGHT_MIN_M = 0.30;
export const TABLE_HEIGHT_MAX_M = 1.40;
export const TABLE_HEIGHT_MANUAL_LIMIT_M = 0.6;
export const TABLE_HEIGHT_AXIS_DEAD_ZONE = 0.2;
export const TABLE_HEIGHT_SPEED_MPS = 0.35;
export const TABLE_HEIGHT_MAX_DT_SECONDS = 0.1;

export interface TableHeightInput {
  headY: number | null;
  axisY: number;
  resetPressed: boolean;
  enabled: boolean;
  nowMs: number;
}

const clamp = (value: number, minimum: number, maximum: number): number => (
  Math.min(maximum, Math.max(minimum, value))
);

export class TableHeightController {
  private active = false;
  private baseHeightM: number | null = null;
  private lastNowMs: number | null = null;
  private neutralRequired = false;
  private resetReleaseSeen = false;
  private manualOffset = 0;

  constructor(private readonly storage: Storage) {}

  get heightM(): number {
    if (!this.active || this.baseHeightM === null) return 0;
    return clamp(
      this.baseHeightM + this.manualOffset,
      TABLE_HEIGHT_MIN_M,
      TABLE_HEIGHT_MAX_M,
    );
  }

  get manualOffsetM(): number {
    return this.manualOffset;
  }

  beginSession(): void {
    this.active = true;
    this.baseHeightM = null;
    this.lastNowMs = null;
    this.neutralRequired = false;
    this.resetReleaseSeen = false;
    this.manualOffset = this.readStoredOffset();
  }

  endSession(): number {
    this.active = false;
    this.baseHeightM = null;
    this.lastNowMs = null;
    this.neutralRequired = false;
    this.resetReleaseSeen = false;
    return 0;
  }

  update(input: TableHeightInput): number {
    if (!this.active) return 0;

    if (!input.enabled) {
      this.neutralRequired = true;
      this.resetReleaseSeen = false;
    }

    if (this.baseHeightM === null) {
      if (!Number.isFinite(input.headY)) return 0;
      this.baseHeightM = clamp(
        (input.headY as number) - 0.72,
        TABLE_HEIGHT_BASE_MIN_M,
        TABLE_HEIGHT_BASE_MAX_M,
      );
      if (Number.isFinite(input.nowMs)) this.lastNowMs = input.nowMs;
    }

    const dtSeconds = this.elapsedSeconds(input.nowMs);

    if (!input.enabled) return this.heightM;

    const axisFinite = Number.isFinite(input.axisY);
    const axisNeutral = axisFinite && Math.abs(input.axisY) <= TABLE_HEIGHT_AXIS_DEAD_ZONE;
    if (this.neutralRequired) {
      if (!axisNeutral) return this.heightM;
      this.neutralRequired = false;
    }

    let didReset = false;
    if (!input.resetPressed) {
      this.resetReleaseSeen = true;
    } else if (this.resetReleaseSeen) {
      this.resetReleaseSeen = false;
      this.setManualOffset(0);
      didReset = true;
    }

    if (!didReset && axisFinite && !axisNeutral && dtSeconds > 0) {
      this.setManualOffset(
        this.manualOffset - input.axisY * TABLE_HEIGHT_SPEED_MPS * dtSeconds,
      );
    }

    return this.heightM;
  }

  private elapsedSeconds(nowMs: number): number {
    if (!Number.isFinite(nowMs)) return 0;

    const previousNowMs = this.lastNowMs;
    this.lastNowMs = nowMs;
    if (previousNowMs === null) return 0;

    return clamp((nowMs - previousNowMs) / 1000, 0, TABLE_HEIGHT_MAX_DT_SECONDS);
  }

  private readStoredOffset(): number {
    try {
      const stored = this.storage.getItem(TABLE_HEIGHT_STORAGE_KEY);
      if (stored === null || stored.trim() === '') return 0;
      const value = Number(stored);
      return Number.isFinite(value)
        ? clamp(value, -TABLE_HEIGHT_MANUAL_LIMIT_M, TABLE_HEIGHT_MANUAL_LIMIT_M)
        : 0;
    } catch {
      return 0;
    }
  }

  private setManualOffset(value: number): void {
    if (!Number.isFinite(value)) return;
    const next = clamp(value, -TABLE_HEIGHT_MANUAL_LIMIT_M, TABLE_HEIGHT_MANUAL_LIMIT_M);
    if (next === this.manualOffset) return;

    this.manualOffset = next;
    try {
      const stableValue = Number(next.toFixed(12));
      this.storage.setItem(TABLE_HEIGHT_STORAGE_KEY, String(stableValue));
    } catch {
      // Storage availability must never interrupt XR control.
    }
  }
}
