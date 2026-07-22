import type {Vec3} from '../protocol/messages';

const STORAGE_X = 'vr4arm.xr.workspaceOffsetX';
const STORAGE_Y = 'vr4arm.xr.workspaceOffsetY';
const STORAGE_Z = 'vr4arm.xr.workspaceOffsetZ';
const LEGACY_HEIGHT_STORAGE = 'vr4arm.xr.tableHeightOffsetM';

const BASE_HEIGHT_MIN_M = 0.55;
const BASE_HEIGHT_MAX_M = 1.10;
const FINAL_HEIGHT_MIN_M = 0.30;
const FINAL_HEIGHT_MAX_M = 1.40;
const HORIZONTAL_LIMIT_M = 1.2;
const HEIGHT_OFFSET_LIMIT_M = 0.6;
const AXIS_DEAD_ZONE = 0.2;
const HORIZONTAL_SPEED_MPS = 0.6;
const VERTICAL_SPEED_MPS = 0.35;
const MAX_DT_SECONDS = 0.1;

export interface WorkspacePlacementInput {
  headY: number | null;
  axisX: number;
  axisY: number;
  heightModifier: boolean;
  resetPressed: boolean;
  enabled: boolean;
  nowMs: number;
}

export class WorkspacePlacementController {
  private active = false;
  private baseHeightM: number | null = null;
  private lastNowMs: number | null = null;
  private neutralRequired = false;
  private resetReleaseSeen = false;
  private offsetX = 0;
  private offsetY = 0;
  private offsetZ = 0;

  constructor(private readonly storage: Storage) {}

  beginSession(): void {
    this.active = true;
    this.baseHeightM = null;
    this.lastNowMs = null;
    this.neutralRequired = false;
    this.resetReleaseSeen = false;
    this.offsetX = this.readOffset(STORAGE_X, 0, HORIZONTAL_LIMIT_M);
    this.offsetY = this.readOffset(STORAGE_Y, this.readLegacyHeight(), HEIGHT_OFFSET_LIMIT_M);
    this.offsetZ = this.readOffset(STORAGE_Z, 0, HORIZONTAL_LIMIT_M);
  }

  endSession(): Vec3 {
    this.active = false;
    this.baseHeightM = null;
    this.lastNowMs = null;
    this.neutralRequired = false;
    this.resetReleaseSeen = false;
    return [0, 0, 0];
  }

  update(input: WorkspacePlacementInput): Vec3 {
    if (!this.active) return [0, 0, 0];

    const axesFinite = Number.isFinite(input.axisX) && Number.isFinite(input.axisY);
    const axesNeutral = axesFinite
      && Math.abs(input.axisX) <= AXIS_DEAD_ZONE
      && Math.abs(input.axisY) <= AXIS_DEAD_ZONE;
    let inputAllowed = input.enabled;
    if (!input.enabled) {
      this.neutralRequired = true;
      this.resetReleaseSeen = false;
    } else if (this.neutralRequired) {
      if (axesNeutral) this.neutralRequired = false;
      else inputAllowed = false;
    }

    let didReset = false;
    if (inputAllowed) {
      if (!input.resetPressed) {
        this.resetReleaseSeen = true;
      } else if (this.resetReleaseSeen) {
        this.resetReleaseSeen = false;
        this.setOffsets(0, 0, 0);
        didReset = true;
      }
    }

    if (this.baseHeightM === null) {
      if (!Number.isFinite(input.headY)) return [0, 0, 0];
      this.baseHeightM = clamp(
        (input.headY as number) - 0.72,
        BASE_HEIGHT_MIN_M,
        BASE_HEIGHT_MAX_M,
      );
      if (Number.isFinite(input.nowMs)) this.lastNowMs = input.nowMs;
    }

    const dt = this.elapsedSeconds(input.nowMs);
    if (inputAllowed && !didReset && axesFinite && !axesNeutral && dt > 0) {
      const nextX = this.offsetX + applyDeadZone(input.axisX) * HORIZONTAL_SPEED_MPS * dt;
      const nextY = input.heightModifier
        ? this.offsetY - applyDeadZone(input.axisY) * VERTICAL_SPEED_MPS * dt
        : this.offsetY;
      const nextZ = input.heightModifier
        ? this.offsetZ
        : this.offsetZ + applyDeadZone(input.axisY) * HORIZONTAL_SPEED_MPS * dt;
      this.setOffsets(nextX, nextY, nextZ);
    }

    return [
      this.offsetX,
      clamp(this.baseHeightM + this.offsetY, FINAL_HEIGHT_MIN_M, FINAL_HEIGHT_MAX_M),
      this.offsetZ,
    ];
  }

  private elapsedSeconds(nowMs: number): number {
    if (!Number.isFinite(nowMs)) return 0;
    const previous = this.lastNowMs;
    this.lastNowMs = nowMs;
    if (previous === null) return 0;
    return clamp((nowMs - previous) / 1000, 0, MAX_DT_SECONDS);
  }

  private setOffsets(x: number, y: number, z: number): void {
    if (![x, y, z].every(Number.isFinite)) return;
    const nextX = clamp(x, -HORIZONTAL_LIMIT_M, HORIZONTAL_LIMIT_M);
    const nextY = clamp(y, -HEIGHT_OFFSET_LIMIT_M, HEIGHT_OFFSET_LIMIT_M);
    const nextZ = clamp(z, -HORIZONTAL_LIMIT_M, HORIZONTAL_LIMIT_M);
    if (nextX === this.offsetX && nextY === this.offsetY && nextZ === this.offsetZ) return;
    this.offsetX = nextX;
    this.offsetY = nextY;
    this.offsetZ = nextZ;
    this.writeOffset(STORAGE_X, nextX);
    this.writeOffset(STORAGE_Y, nextY);
    this.writeOffset(STORAGE_Z, nextZ);
  }

  private readLegacyHeight(): number {
    try {
      const value = Number(this.storage.getItem(LEGACY_HEIGHT_STORAGE));
      return Number.isFinite(value) ? value : 0;
    } catch {
      return 0;
    }
  }

  private readOffset(key: string, fallback: number, limit: number): number {
    try {
      const raw = this.storage.getItem(key);
      if (raw === null || raw.trim() === '') return clamp(fallback, -limit, limit);
      const value = Number(raw);
      return Number.isFinite(value) ? clamp(value, -limit, limit) : clamp(fallback, -limit, limit);
    } catch {
      return clamp(fallback, -limit, limit);
    }
  }

  private writeOffset(key: string, value: number): void {
    try {
      this.storage.setItem(key, String(Number(value.toFixed(12))));
    } catch {
      // Storage availability must never interrupt XR presentation.
    }
  }
}

function applyDeadZone(value: number): number {
  return Math.abs(value) <= AXIS_DEAD_ZONE ? 0 : value;
}

function clamp(value: number, minimum: number, maximum: number): number {
  return Math.min(maximum, Math.max(minimum, value));
}
