from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(slots=True)
class DroneConfig:
    connection_string: str


@dataclass(slots=True)
class TakeoffConfig:
    alt_m: float
    vel_mps: float
    timeout_s: float


@dataclass(slots=True)
class LandingConfig:
    switch_alt_m: float
    vel_mps: float
    descent_timeout_s: float
    no_vision_descent_max_s: float


@dataclass(slots=True)
class AlignmentConfig:
    timeout_s: float
    pid_kp: tuple[float, float]
    pid_ki: tuple[float, float]
    pid_kd: tuple[float, float]


@dataclass(slots=True)
class RuntimeConfig:
    loop_hz: float
    max_speed_mps: float


@dataclass(slots=True)
class IpcConfig:
    host: str
    vision_udp_port: int
    control_udp_port: int
    sample_ttl_s: float
    drop_to_zero_after_s: float
    render_fps: int
    send_hz_cap: int


@dataclass(slots=True)
class ServoConfig:
    pwm_channel: int
    pwm_chip: int
    steps_0_deg: float
    steps_180_deg: float
    angle_landing: float


@dataclass(slots=True)
class VisionConfig:
    frame_color_order: str
    red_lower_1: tuple[int, int, int]
    red_upper_1: tuple[int, int, int]
    red_lower_2: tuple[int, int, int]
    red_upper_2: tuple[int, int, int]
    red_min_area_px: float


@dataclass(slots=True)
class UasConfig:
    drone: DroneConfig
    takeoff: TakeoffConfig
    landing: LandingConfig
    alignment: AlignmentConfig
    runtime: RuntimeConfig
    ipc: IpcConfig
    servo: ServoConfig
    vision: VisionConfig
    logs_dir: Path
