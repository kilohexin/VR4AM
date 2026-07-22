from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from app.sim.lm3_model import LM3Model


class VirtualRobot:
    def __init__(self, model: LM3Model):
        self.model = model
        self.q = np.asarray(model.home_q, dtype=float)
        self.qd = np.zeros(6, dtype=float)
        self.target_q: np.ndarray | None = None
        self.target_speed_radps: float | None = None

    def set_target_q(
        self,
        target: Sequence[float],
        max_speed_radps: float | None = None,
    ) -> None:
        try:
            target_q = np.asarray(target, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("joint_safety_window") from exc
        q_ref = np.asarray(self.model.home_q, dtype=float)
        if (
            target_q.shape != (6,)
            or not np.all(np.isfinite(target_q))
            or np.any(np.abs(target_q - q_ref) > self.model.joint_window_rad)
        ):
            raise ValueError("joint_safety_window")
        if max_speed_radps is not None and (
            not np.isfinite(max_speed_radps) or max_speed_radps <= 0
        ):
            raise ValueError("invalid_joint_speed")
        self.target_q = target_q.copy()
        self.target_speed_radps = max_speed_radps

    def stop(self) -> None:
        self.target_q = None
        self.target_speed_radps = None

    def step(self, dt: float) -> None:
        previous_qd = self.qd.copy()
        if self.target_q is None:
            desired_qd = np.zeros(6, dtype=float)
        else:
            position_error = self.target_q - self.q
            speed_limit = min(
                self.model.max_joint_speed_radps,
                self.target_speed_radps or self.model.max_joint_speed_radps,
            )
            desired_qd = np.clip(
                2.0 * position_error,
                -speed_limit,
                speed_limit,
            )

        max_velocity_change = self.model.max_joint_accel_radps2 * dt
        self.qd += np.clip(
            desired_qd - self.qd,
            -max_velocity_change,
            max_velocity_change,
        )
        self.q += self.qd * dt

        can_stop_within_acceleration_limit = np.max(np.abs(previous_qd)) <= max_velocity_change
        if (
            self.target_q is not None
            and np.max(np.abs(self.target_q - self.q)) < 1e-4
            and can_stop_within_acceleration_limit
        ):
            self.q = self.target_q.copy()
            self.qd[:] = 0.0
