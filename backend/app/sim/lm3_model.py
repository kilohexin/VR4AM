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

    def __post_init__(self) -> None:
        for field_name in ("home_q", "a_prev_m", "alpha_prev_rad", "d_m"):
            values = getattr(self, field_name)
            if len(values) != 6 or not all(math.isfinite(value) for value in values):
                raise ValueError(f"{field_name} must contain exactly six finite values")
        if len(self.tcp_offset_m) != 3 or not all(math.isfinite(value) for value in self.tcp_offset_m):
            raise ValueError("tcp_offset_m must contain exactly three finite values")
