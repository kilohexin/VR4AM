from __future__ import annotations

from dataclasses import replace

from app.config import (
    LebaiControlSettings,
    LebaiGripperSettings,
    LebaiSettings,
    TcpExpectation,
)


def readonly_settings(**changes) -> LebaiSettings:
    settings = LebaiSettings(
        mode="readonly",
        ip="192.168.10.20",
        expected_tcp=TcpExpectation(
            x=0.0,
            y=0.0,
            z=0.175,
            rz=0.0,
            ry=0.0,
            rx=0.0,
        ),
        home_q=(0.0, -1.0, 1.0, 0.0, 1.57, 0.0),
        teleop_ready_q=(0.0, -1.0, 1.0, 0.0, 0.2, 0.0),
        soft_joint_min_rad=(-3.0, -2.5, -2.5, -3.0, -2.5, -6.0),
        soft_joint_max_rad=(3.0, 2.5, 2.5, 3.0, 2.5, 6.0),
        joint_limit_margin_rad=0.05,
        startup_tcp_min_m=(0.1, -0.3, 0.1),
        startup_tcp_max_m=(0.5, 0.3, 0.6),
        tcp_position_tolerance_m=0.001,
        tcp_rotation_tolerance_deg=0.5,
        control=LebaiControlSettings(
            loop_hz=50,
            state_hz=25,
            pvat_send_hz=25,
            pvat_horizon_s=0.08,
            max_tcp_speed_mps=0.03,
            max_tcp_rotation_radps=0.25,
            max_tcp_acceleration_mps2=0.1,
            max_tcp_angular_acceleration_radps2=0.5,
            max_joint_speed_radps=0.15,
            max_joint_acceleration_radps2=0.5,
            max_joint_step_rad=0.05,
            max_tcp_step_m=0.002,
            max_tcp_rotation_step_deg=1.0,
            max_relative_translation_m=0.1,
            max_relative_rotation_deg=30.0,
            translation_scale=0.5,
        ),
        gripper=LebaiGripperSettings(
            max_force_percent=30,
            command_hz=10,
            open_amplitude_percent=100,
            closed_amplitude_percent=0,
        ),
    )
    return replace(settings, **changes)


def control_settings(**changes) -> LebaiSettings:
    return replace(readonly_settings(), mode="control", **changes)
