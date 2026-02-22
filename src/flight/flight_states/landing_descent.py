from __future__ import annotations

import asyncio

from flight.flight_states.common import (
    AlignPolicyInput,
    LandingPolicyInput,
    compute_landing_command,
    compute_xy_from_vision,
)
from flight.state import StaleVisionMode, VelocityCommand


async def run(controller: "FlightController") -> None:
    controller.logger.info("[FLIGHT] Landing descent phase started")

    dt = 1.0 / controller.cfg.runtime.loop_hz
    loop = asyncio.get_running_loop()
    end_time = loop.time() + controller.cfg.landing.descent_timeout_s
    next_log = loop.time()

    while loop.time() < end_time:
        tick_start = loop.time()

        align_out = compute_xy_from_vision(
            AlignPolicyInput(
                sample=controller.latest_vision_sample,
                vision_age_s=controller.get_vision_age_s(),
                dt=dt,
                max_speed_mps=controller.cfg.runtime.max_speed_mps,
                sample_ttl_s=controller.cfg.ipc.sample_ttl_s,
                drop_to_zero_after_s=controller.cfg.ipc.drop_to_zero_after_s,
            ),
            state=controller.vision_state,
            pid=controller.pid_controller,
        )
        if align_out.mode == StaleVisionMode.FRESH:
            controller.vision_state.last_valid_target_seen_time = loop.time()

        controller.vision_state.target_visible = align_out.target_visible
        current_alt = controller.telemetry.get_altitude()
        since_valid = None
        if controller.vision_state.last_valid_target_seen_time is not None:
            since_valid = loop.time() - controller.vision_state.last_valid_target_seen_time

        landing_out = compute_landing_command(
            LandingPolicyInput(
                current_alt_m=current_alt,
                since_valid_s=since_valid,
                landing_switch_alt_m=controller.cfg.landing.switch_alt_m,
                no_vision_descent_max_s=controller.cfg.landing.no_vision_descent_max_s,
                landing_vel_mps=controller.cfg.landing.vel_mps,
                mode=align_out.mode,
                xy_cmd=align_out.cmd,
            )
        )

        controller.set_desired_velocity(landing_out.cmd)
        controller.publish_control_status()

        if landing_out.should_switch_to_land_command:
            controller.logger.info("[LAND_DESCENT] Switch altitude reached (%.2fm)", current_alt)
            await controller.send_land_command_if_needed()
            break

        if loop.time() >= next_log:
            controller.logger.info(
                "[LAND_DESCENT] mode=%s visible=%s alt=%.2f vx=%.2f vy=%.2f vz=%.2f",
                align_out.mode.value,
                controller.vision_state.target_visible,
                current_alt,
                landing_out.cmd.vx,
                landing_out.cmd.vy,
                landing_out.cmd.vz,
            )
            next_log = loop.time() + 0.5

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)

    controller.set_desired_velocity(VelocityCommand())
    await controller.send_land_command_if_needed()


from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from flight.controller import FlightController
