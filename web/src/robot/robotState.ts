import type {JointVector, RobotStateMessage} from '../protocol/messages';

const STALE_AFTER_NS = 100_000_000;

export interface RobotStateSample {
  state: RobotStateMessage;
  stale: boolean;
}

export class RobotStateBuffer {
  private states: RobotStateMessage[] = [];

  push(state: RobotStateMessage): void {
    const newest = this.states.at(-1);
    if (newest && state.server_mono_ns <= newest.server_mono_ns) return;
    this.states = [...this.states.slice(-1), state];
  }

  sample(nowNs: number): RobotStateSample | null {
    const newest = this.states.at(-1);
    if (!newest) return null;

    const stale = nowNs - newest.server_mono_ns > STALE_AFTER_NS;
    const older = this.states.at(-2);
    if (!older) {
      return {
        state: {...newest, gripper: clamp01(newest.gripper)},
        stale,
      };
    }

    const duration = newest.server_mono_ns - older.server_mono_ns;
    const alpha = clamp01((nowNs - older.server_mono_ns) / duration);
    return {
      state: {
        ...newest,
        actual_q: interpolateJoints(older.actual_q, newest.actual_q, alpha),
        gripper: clamp01(lerp(older.gripper, newest.gripper, alpha)),
      },
      stale,
    };
  }
}

function interpolateJoints(a: JointVector, b: JointVector, alpha: number): JointVector {
  return [
    lerp(a[0], b[0], alpha),
    lerp(a[1], b[1], alpha),
    lerp(a[2], b[2], alpha),
    lerp(a[3], b[3], alpha),
    lerp(a[4], b[4], alpha),
    lerp(a[5], b[5], alpha),
  ];
}

function lerp(a: number, b: number, alpha: number): number {
  return a + (b - a) * alpha;
}

function clamp01(value: number): number {
  return Math.min(1, Math.max(0, value));
}
