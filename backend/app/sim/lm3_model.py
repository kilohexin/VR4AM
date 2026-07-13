from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LM3Model:
    home_q: tuple[float, ...] = (0.0, -0.7853981634, 1.5707963268, -0.7853981634, 1.5707963268, 0.0)
    a_prev_m: tuple[float, ...] = (0.0, 0.0, -0.28, -0.26, 0.0, 0.0)
    alpha_prev_rad: tuple[float, ...] = (0.0, math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2)
    d_m: tuple[float, ...] = (0.21583, 0.0, 0.0, 0.12063, 0.09833, 0.08343)
    tcp_offset_m: tuple[float, float, float] = (0.0, 0.0, 0.09)
    joint_window_rad: float = math.pi
    max_joint_speed_radps: float = 0.5
    max_joint_accel_radps2: float = 1.0
