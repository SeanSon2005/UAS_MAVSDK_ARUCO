from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(slots=True)
class DroneConfig:
    connection_string: str
    sim_connection_string: str


@dataclass(slots=True)
class TakeoffConfig:
    alt_m: float
    vel_mps: float
    timeout_s: float
    alt_tolerance_m: float
    hold_s: float


@dataclass(slots=True)
class LandingConfig:
    timeout_s: float


@dataclass(slots=True)
class AlignmentConfig:
    timeout_s: float
    pid_kp: tuple[float, float]
    pid_ki: tuple[float, float]
    pid_kd: tuple[float, float]
    success_err_px: tuple[float, float]
    success_hold_s: float


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
    send_hz_cap: int
    render_fps: int


@dataclass(slots=True)
class VisionConfig:
    frame_color_order: str
    red_lower_1: tuple[int, int, int]
    red_upper_1: tuple[int, int, int]
    red_lower_2: tuple[int, int, int]
    red_upper_2: tuple[int, int, int]
    red_min_area_px: float
    gst_udp_port: int
    gst_payload: int
    gst_latency_ms: int


@dataclass(slots=True)
class LogsConfig:
    dir: Path


@dataclass(slots=True)
class UasConfig:
    is_simulation: bool
    drone: DroneConfig
    takeoff: TakeoffConfig
    landing: LandingConfig
    alignment: AlignmentConfig
    runtime: RuntimeConfig
    ipc: IpcConfig
    vision: VisionConfig
    logs: LogsConfig


class _Missing:
    pass


MISSING = _Missing()


def _to_tuple2(vals: Any, default: tuple[float, float]) -> tuple[float, float]:
    if not isinstance(vals, (list, tuple)) or len(vals) < 2:
        return default
    return float(vals[0]), float(vals[1])


def _to_tuple3(vals: Any, default: tuple[int, int, int]) -> tuple[int, int, int]:
    if not isinstance(vals, (list, tuple)) or len(vals) < 3:
        return default
    return int(vals[0]), int(vals[1]), int(vals[2])


def _lookup(section: dict[str, Any], keys: list[str], default: Any = MISSING) -> Any:
    for key in keys:
        if key in section:
            return section[key]
    if default is not MISSING:
        return default
    raise KeyError(f"Missing required key. Tried: {keys}")


def _as_bool(v: Any, default: bool) -> bool:
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        text = v.strip().lower()
        if text in {"1", "true", "yes", "y", "on"}:
            return True
        if text in {"0", "false", "no", "n", "off"}:
            return False
    if isinstance(v, (int, float)):
        return bool(v)
    return default


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"Invalid YAML root in {path}")
    return data


def load_config(path: Path | None = None) -> UasConfig:
    repo_root = Path(__file__).resolve().parents[2]
    cfg_path = path or (repo_root / "config.yaml")
    data = _load_yaml(cfg_path)

    drone = data.get("drone", {}) if isinstance(data.get("drone"), dict) else {}
    takeoff = data.get("takeoff", {}) if isinstance(data.get("takeoff"), dict) else {}
    landing = data.get("landing", {}) if isinstance(data.get("landing"), dict) else {}
    alignment = data.get("alignment", {}) if isinstance(data.get("alignment"), dict) else {}
    runtime = data.get("runtime", {}) if isinstance(data.get("runtime"), dict) else {}
    ipc = data.get("ipc", {}) if isinstance(data.get("ipc"), dict) else {}
    vision = data.get("vision", {}) if isinstance(data.get("vision"), dict) else {}
    logs = data.get("logs", {}) if isinstance(data.get("logs"), dict) else {}

    is_simulation_raw = _lookup(data, ["is_simulation", "is_simluation"], default=True)
    is_simulation = _as_bool(is_simulation_raw, default=True)

    takeoff_timeout = _lookup(takeoff, ["timeout_s", "timeout]_s"], default=7.0)
    landing_timeout = _lookup(landing, ["timeout_s", "timeout]_s"], default=12.0)

    return UasConfig(
        is_simulation=is_simulation,
        drone=DroneConfig(
            connection_string=str(_lookup(drone, ["connection_string"], default="serial:///dev/ttyAMA0:57600")),
            sim_connection_string=str(_lookup(drone, ["sim_connection_string"], default="udp:127.0.0.1:14540")),
        ),
        takeoff=TakeoffConfig(
            alt_m=float(_lookup(takeoff, ["alt_m"], default=1.0)),
            vel_mps=float(_lookup(takeoff, ["vel_mps"], default=0.6)),
            timeout_s=float(takeoff_timeout),
            alt_tolerance_m=float(_lookup(takeoff, ["alt_tolerance_m"], default=0.1)),
            hold_s=float(_lookup(takeoff, ["hold_s"], default=0.5)),
        ),
        landing=LandingConfig(
            timeout_s=float(landing_timeout),
        ),
        alignment=AlignmentConfig(
            timeout_s=float(_lookup(alignment, ["timeout_s"], default=15.0)),
            pid_kp=_to_tuple2(_lookup(alignment, ["pid_kp"], default=[0.002, 0.002]), (0.002, 0.002)),
            pid_ki=_to_tuple2(_lookup(alignment, ["pid_ki"], default=[0.0, 0.0]), (0.0, 0.0)),
            pid_kd=_to_tuple2(_lookup(alignment, ["pid_kd"], default=[0.0008, 0.0008]), (0.0008, 0.0008)),
            success_err_px=_to_tuple2(_lookup(alignment, ["success_err_px"], default=[20.0, 20.0]), (20.0, 20.0)),
            success_hold_s=float(_lookup(alignment, ["success_hold_s"], default=0.8)),
        ),
        runtime=RuntimeConfig(
            loop_hz=float(_lookup(runtime, ["loop_hz"], default=50.0)),
            max_speed_mps=float(_lookup(runtime, ["max_speed_mps"], default=1.0)),
        ),
        ipc=IpcConfig(
            host=str(_lookup(ipc, ["host"], default="127.0.0.1")),
            vision_udp_port=int(_lookup(ipc, ["vision_udp_port"], default=14601)),
            control_udp_port=int(_lookup(ipc, ["control_udp_port"], default=14602)),
            sample_ttl_s=float(_lookup(ipc, ["sample_ttl_s"], default=0.15)),
            drop_to_zero_after_s=float(_lookup(ipc, ["drop_to_zero_after_s"], default=0.30)),
            send_hz_cap=int(_lookup(ipc, ["send_hz_cap"], default=30)),
            render_fps=int(_lookup(ipc, ["render_fps"], default=12)),
        ),
        vision=VisionConfig(
            frame_color_order=str(_lookup(vision, ["frame_color_order"], default="RGB")),
            red_lower_1=_to_tuple3(_lookup(vision, ["red_lower_1"], default=[0, 70, 0]), (0, 70, 0)),
            red_upper_1=_to_tuple3(_lookup(vision, ["red_upper_1"], default=[5, 255, 255]), (5, 255, 255)),
            red_lower_2=_to_tuple3(_lookup(vision, ["red_lower_2"], default=[165, 70, 0]), (165, 70, 0)),
            red_upper_2=_to_tuple3(_lookup(vision, ["red_upper_2"], default=[180, 255, 255]), (180, 255, 255)),
            red_min_area_px=float(_lookup(vision, ["red_min_area_px"], default=300.0)),
            gst_udp_port=int(_lookup(vision, ["gst_udp_port"], default=5600)),
            gst_payload=int(_lookup(vision, ["gst_payload"], default=96)),
            gst_latency_ms=int(_lookup(vision, ["gst_latency_ms"], default=0)),
        ),
        logs=LogsConfig(
            dir=(repo_root / str(_lookup(logs, ["dir"], default="logs"))).resolve(),
        ),
    )
