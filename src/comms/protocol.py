from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any


PROTO_VERSION = 2
VISION_SAMPLE_TYPE = "vision_sample"
MAV_STATUS_TYPE = "mav_status"


@dataclass(slots=True)
class VisionSample:
    v: int
    type: str
    seq: int
    t_mono_ns: int
    has_target: bool
    err_x_px: float | None
    err_y_px: float | None
    contour_area: float
    frame_w: int
    frame_h: int
    vx_cmd_f_mps: float
    vy_cmd_r_mps: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class MavlinkStatus:
    v: int
    type: str
    phase: str
    armed: bool
    offboard_active: bool
    mode: str
    pos_n: float | None
    pos_e: float | None
    pos_d: float | None
    vel_n: float | None
    vel_e: float | None
    vel_d: float | None
    alt_m: float | None
    flow_quality: int | None
    distance_m: float | None
    last_vision_age_s: float | None
    cmd_vx_f: float
    cmd_vy_r: float
    cmd_vz_d: float
    t_mono_ns: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _decode_json(data: bytes) -> dict[str, Any] | None:
    try:
        payload = json.loads(data.decode("utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _as_float_or_none(v: Any) -> float | None:
    if v is None:
        return None
    return float(v)


def _as_int_or_none(v: Any) -> int | None:
    if v is None:
        return None
    return int(v)


def decode_vision_sample(data: bytes) -> VisionSample | None:
    payload = _decode_json(data)
    if payload is None:
        return None

    try:
        if int(payload.get("v")) != PROTO_VERSION:
            return None
        if payload.get("type") != VISION_SAMPLE_TYPE:
            return None
        sample = VisionSample(
            v=PROTO_VERSION,
            type=VISION_SAMPLE_TYPE,
            seq=int(payload["seq"]),
            t_mono_ns=int(payload["t_mono_ns"]),
            has_target=bool(payload["has_target"]),
            err_x_px=_as_float_or_none(payload.get("err_x_px")),
            err_y_px=_as_float_or_none(payload.get("err_y_px")),
            contour_area=float(payload["contour_area"]),
            frame_w=int(payload["frame_w"]),
            frame_h=int(payload["frame_h"]),
            vx_cmd_f_mps=float(payload["vx_cmd_f_mps"]),
            vy_cmd_r_mps=float(payload["vy_cmd_r_mps"]),
        )
    except (KeyError, TypeError, ValueError):
        return None

    if sample.seq < 0 or sample.t_mono_ns <= 0:
        return None
    if sample.frame_w <= 0 or sample.frame_h <= 0:
        return None
    return sample


def decode_mavlink_status(data: bytes) -> MavlinkStatus | None:
    payload = _decode_json(data)
    if payload is None:
        return None

    try:
        if int(payload.get("v")) != PROTO_VERSION:
            return None
        if payload.get("type") != MAV_STATUS_TYPE:
            return None
        status = MavlinkStatus(
            v=PROTO_VERSION,
            type=MAV_STATUS_TYPE,
            phase=str(payload["phase"]),
            armed=bool(payload["armed"]),
            offboard_active=bool(payload["offboard_active"]),
            mode=str(payload["mode"]),
            pos_n=_as_float_or_none(payload.get("pos_n")),
            pos_e=_as_float_or_none(payload.get("pos_e")),
            pos_d=_as_float_or_none(payload.get("pos_d")),
            vel_n=_as_float_or_none(payload.get("vel_n")),
            vel_e=_as_float_or_none(payload.get("vel_e")),
            vel_d=_as_float_or_none(payload.get("vel_d")),
            alt_m=_as_float_or_none(payload.get("alt_m")),
            flow_quality=_as_int_or_none(payload.get("flow_quality")),
            distance_m=_as_float_or_none(payload.get("distance_m")),
            last_vision_age_s=_as_float_or_none(payload.get("last_vision_age_s")),
            cmd_vx_f=float(payload["cmd_vx_f"]),
            cmd_vy_r=float(payload["cmd_vy_r"]),
            cmd_vz_d=float(payload["cmd_vz_d"]),
            t_mono_ns=int(payload["t_mono_ns"]),
        )
    except (KeyError, TypeError, ValueError):
        return None

    if status.t_mono_ns <= 0:
        return None
    return status


def encode_payload(payload: VisionSample | MavlinkStatus) -> bytes:
    return json.dumps(payload.to_dict(), separators=(",", ":")).encode("utf-8")
