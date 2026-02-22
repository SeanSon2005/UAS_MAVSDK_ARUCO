from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from typing import Any


PROTO_VERSION = 1
VISION_SAMPLE_TYPE = "vision_sample"
CONTROL_STATUS_TYPE = "control_status"


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

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(slots=True)
class ControlStatus:
    v: int
    type: str
    phase: str
    vx: float
    vy: float
    vz: float
    alt_m: float
    target_visible: bool
    t_mono_ns: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _decode_json(data: bytes) -> dict[str, Any] | None:
    try:
        payload = json.loads(data.decode("utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


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
            err_x_px=None if payload.get("err_x_px") is None else float(payload["err_x_px"]),
            err_y_px=None if payload.get("err_y_px") is None else float(payload["err_y_px"]),
            contour_area=float(payload["contour_area"]),
            frame_w=int(payload["frame_w"]),
            frame_h=int(payload["frame_h"]),
        )
    except (KeyError, TypeError, ValueError):
        return None

    if sample.seq < 0 or sample.t_mono_ns <= 0:
        return None
    if sample.frame_w <= 0 or sample.frame_h <= 0:
        return None
    return sample


def decode_control_status(data: bytes) -> ControlStatus | None:
    payload = _decode_json(data)
    if payload is None:
        return None

    try:
        if int(payload.get("v")) != PROTO_VERSION:
            return None
        if payload.get("type") != CONTROL_STATUS_TYPE:
            return None
        return ControlStatus(
            v=PROTO_VERSION,
            type=CONTROL_STATUS_TYPE,
            phase=str(payload["phase"]),
            vx=float(payload["vx"]),
            vy=float(payload["vy"]),
            vz=float(payload["vz"]),
            alt_m=float(payload["alt_m"]),
            target_visible=bool(payload["target_visible"]),
            t_mono_ns=int(payload["t_mono_ns"]),
        )
    except (KeyError, TypeError, ValueError):
        return None


def encode_payload(payload: VisionSample | ControlStatus) -> bytes:
    return json.dumps(payload.to_dict(), separators=(",", ":")).encode("utf-8")
