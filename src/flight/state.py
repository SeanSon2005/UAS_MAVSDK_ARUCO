from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class MissionPhase(str, Enum):
    INIT = "INIT"
    TAKEOFF = "TAKEOFF"
    ALIGN = "ALIGN"
    LAND_DESCENT = "LAND_DESCENT"
    SHUTDOWN = "SHUTDOWN"


class StaleVisionMode(str, Enum):
    FRESH = "fresh"
    HOLD = "hold"
    ZERO = "zero"


@dataclass(slots=True)
class VelocityCommand:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0


@dataclass(slots=True)
class VisionState:
    target_visible: bool = False
    last_valid_vx: float = 0.0
    last_valid_vy: float = 0.0
    last_valid_target_seen_time: float | None = None


@dataclass(slots=True)
class ControlState:
    phase: MissionPhase = MissionPhase.INIT
    armed: bool = False
    offboard_started: bool = False
    land_command_sent: bool = False
