from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from flight.state import StaleVisionMode, VelocityCommand, VisionState


class VisionSampleLike(Protocol):
    has_target: bool
    err_x_px: float | None
    err_y_px: float | None


class PidLike(Protocol):
    def reset(self, target=None, initial_measurement=None) -> None: ...

    def update(self, measurement, dt: float): ...


@dataclass(slots=True)
class AlignPolicyInput:
    sample: VisionSampleLike | None
    vision_age_s: float | None
    dt: float
    max_speed_mps: float
    sample_ttl_s: float
    drop_to_zero_after_s: float


@dataclass(slots=True)
class AlignPolicyOutput:
    mode: StaleVisionMode
    target_visible: bool
    err_x_px: float | None
    err_y_px: float | None
    cmd: VelocityCommand


@dataclass(slots=True)
class LandingPolicyInput:
    current_alt_m: float
    since_valid_s: float | None
    landing_switch_alt_m: float
    no_vision_descent_max_s: float
    landing_vel_mps: float
    mode: StaleVisionMode
    xy_cmd: VelocityCommand


@dataclass(slots=True)
class LandingPolicyOutput:
    should_switch_to_land_command: bool
    cmd: VelocityCommand


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def stale_mode_from_age(
    age_s: float | None,
    sample_ttl_s: float,
    drop_to_zero_after_s: float,
) -> StaleVisionMode:
    if age_s is None:
        return StaleVisionMode.ZERO
    if age_s <= sample_ttl_s:
        return StaleVisionMode.FRESH
    if age_s <= drop_to_zero_after_s:
        return StaleVisionMode.HOLD
    return StaleVisionMode.ZERO


def compute_xy_from_vision(
    inp: AlignPolicyInput,
    state: VisionState,
    pid: PidLike,
) -> AlignPolicyOutput:
    mode = stale_mode_from_age(inp.vision_age_s, inp.sample_ttl_s, inp.drop_to_zero_after_s)

    if inp.sample is None or mode == StaleVisionMode.ZERO:
        pid.reset(target=(0.0, 0.0))
        return AlignPolicyOutput(
            mode=StaleVisionMode.ZERO,
            target_visible=False,
            err_x_px=None,
            err_y_px=None,
            cmd=VelocityCommand(),
        )

    if mode == StaleVisionMode.FRESH:
        if inp.sample.has_target and inp.sample.err_x_px is not None and inp.sample.err_y_px is not None:
            err_x_px = float(inp.sample.err_x_px)
            err_y_px = float(inp.sample.err_y_px)
            pid_out = pid.update((-err_x_px, -err_y_px), inp.dt)
            vx = clamp(float(-pid_out[1]), -inp.max_speed_mps, inp.max_speed_mps)
            vy = clamp(float(pid_out[0]), -inp.max_speed_mps, inp.max_speed_mps)

            state.last_valid_vx = vx
            state.last_valid_vy = vy
            return AlignPolicyOutput(
                mode=StaleVisionMode.FRESH,
                target_visible=True,
                err_x_px=err_x_px,
                err_y_px=err_y_px,
                cmd=VelocityCommand(vx=vx, vy=vy, vz=0.0, yaw_rate=0.0),
            )

        pid.reset(target=(0.0, 0.0))
        return AlignPolicyOutput(
            mode=StaleVisionMode.ZERO,
            target_visible=False,
            err_x_px=None,
            err_y_px=None,
            cmd=VelocityCommand(),
        )

    return AlignPolicyOutput(
        mode=StaleVisionMode.HOLD,
        target_visible=False,
        err_x_px=None,
        err_y_px=None,
        cmd=VelocityCommand(vx=state.last_valid_vx, vy=state.last_valid_vy, vz=0.0, yaw_rate=0.0),
    )


def should_allow_descent_without_fresh_vision(
    current_alt_m: float,
    since_valid_s: float | None,
    landing_switch_alt_m: float,
    no_vision_descent_max_s: float,
) -> bool:
    return (
        since_valid_s is not None
        and since_valid_s <= no_vision_descent_max_s
        and current_alt_m > landing_switch_alt_m
    )


def compute_landing_command(inp: LandingPolicyInput) -> LandingPolicyOutput:
    if inp.current_alt_m <= inp.landing_switch_alt_m:
        return LandingPolicyOutput(should_switch_to_land_command=True, cmd=VelocityCommand())

    if inp.mode == StaleVisionMode.FRESH:
        return LandingPolicyOutput(
            should_switch_to_land_command=False,
            cmd=VelocityCommand(vx=inp.xy_cmd.vx, vy=inp.xy_cmd.vy, vz=abs(inp.landing_vel_mps), yaw_rate=0.0),
        )

    allow = should_allow_descent_without_fresh_vision(
        current_alt_m=inp.current_alt_m,
        since_valid_s=inp.since_valid_s,
        landing_switch_alt_m=inp.landing_switch_alt_m,
        no_vision_descent_max_s=inp.no_vision_descent_max_s,
    )
    vz = abs(inp.landing_vel_mps) if allow else 0.0
    return LandingPolicyOutput(
        should_switch_to_land_command=False,
        cmd=VelocityCommand(vx=0.0, vy=0.0, vz=vz, yaw_rate=0.0),
    )
