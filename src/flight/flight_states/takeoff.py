from __future__ import annotations

import asyncio

from flight.state import VelocityCommand


async def run(controller: "FlightController") -> None:
    controller.logger.info("[FLIGHT] Takeoff phase started target=%.2fm", controller.cfg.takeoff.alt_m)
    controller.actuator.move(controller.cfg.servo.angle_landing, "DOWN")

    dt = 1.0 / controller.cfg.runtime.loop_hz
    loop = asyncio.get_running_loop()
    start_time = loop.time()

    while True:
        tick_start = loop.time()
        alt = controller.telemetry.get_altitude()

        if loop.time() - start_time > controller.cfg.takeoff.timeout_s:
            controller.set_desired_velocity(VelocityCommand())
            controller.publish_control_status()
            raise RuntimeError("[TAKEOFF] timeout: altitude target not reached")

        if alt >= controller.cfg.takeoff.alt_m:
            controller.set_desired_velocity(VelocityCommand())
            controller.publish_control_status()
            controller.logger.info("[TAKEOFF] target reached: alt=%.2fm", alt)
            break

        controller.set_desired_velocity(
            VelocityCommand(vx=0.0, vy=0.0, vz=-abs(controller.cfg.takeoff.vel_mps), yaw_rate=0.0)
        )
        controller.publish_control_status()

        remaining = dt - (loop.time() - tick_start)
        if remaining > 0:
            await asyncio.sleep(remaining)


from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from flight.controller import FlightController
