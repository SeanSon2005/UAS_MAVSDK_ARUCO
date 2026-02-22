from __future__ import annotations

from pathlib import Path

import yaml

from config.schemas import (
    AlignmentConfig,
    DroneConfig,
    IpcConfig,
    LandingConfig,
    RuntimeConfig,
    ServoConfig,
    TakeoffConfig,
    UasConfig,
    VisionConfig,
)


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML root in {path}")
    return data


def load_config() -> UasConfig:
    root = Path(__file__).resolve().parents[2]
    base = _load_yaml(root / "config" / "default.yaml")

    return UasConfig(
        drone=DroneConfig(connection_string=str(base["drone"]["connection_string"])),
        takeoff=TakeoffConfig(
            alt_m=float(base["takeoff"]["alt_m"]),
            vel_mps=float(base["takeoff"]["vel_mps"]),
            timeout_s=float(base["takeoff"]["timeout_s"]),
        ),
        landing=LandingConfig(
            switch_alt_m=float(base["landing"]["switch_alt_m"]),
            vel_mps=float(base["landing"]["vel_mps"]),
            descent_timeout_s=float(base["landing"]["descent_timeout_s"]),
            no_vision_descent_max_s=float(base["landing"]["no_vision_descent_max_s"]),
        ),
        alignment=AlignmentConfig(
            timeout_s=float(base["alignment"]["timeout_s"]),
            pid_kp=(float(base["alignment"]["pid_kp"][0]), float(base["alignment"]["pid_kp"][1])),
            pid_ki=(float(base["alignment"]["pid_ki"][0]), float(base["alignment"]["pid_ki"][1])),
            pid_kd=(float(base["alignment"]["pid_kd"][0]), float(base["alignment"]["pid_kd"][1])),
        ),
        runtime=RuntimeConfig(
            loop_hz=float(base["runtime"]["loop_hz"]),
            max_speed_mps=float(base["runtime"]["max_speed_mps"]),
        ),
        ipc=IpcConfig(
            host=str(base["ipc"]["host"]),
            vision_udp_port=int(base["ipc"]["vision_udp_port"]),
            control_udp_port=int(base["ipc"]["control_udp_port"]),
            sample_ttl_s=float(base["ipc"]["sample_ttl_s"]),
            drop_to_zero_after_s=float(base["ipc"]["drop_to_zero_after_s"]),
            render_fps=int(base["ipc"]["render_fps"]),
            send_hz_cap=int(base["ipc"]["send_hz_cap"]),
        ),
        servo=ServoConfig(
            pwm_channel=int(base["servo"]["pwm_channel"]),
            pwm_chip=int(base["servo"]["pwm_chip"]),
            steps_0_deg=float(base["servo"]["steps_0_deg"]),
            steps_180_deg=float(base["servo"]["steps_180_deg"]),
            angle_landing=float(base["servo"]["angle_landing"]),
        ),
        vision=VisionConfig(
            frame_color_order=str(base["vision"]["frame_color_order"]),
            red_lower_1=tuple(int(v) for v in base["vision"]["red_lower_1"]),
            red_upper_1=tuple(int(v) for v in base["vision"]["red_upper_1"]),
            red_lower_2=tuple(int(v) for v in base["vision"]["red_lower_2"]),
            red_upper_2=tuple(int(v) for v in base["vision"]["red_upper_2"]),
            red_min_area_px=float(base["vision"]["red_min_area_px"]),
        ),
        logs_dir=(root / str(base.get("logs", {}).get("dir", "logs"))),
    )
