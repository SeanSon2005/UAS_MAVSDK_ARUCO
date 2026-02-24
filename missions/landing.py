from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.processes.mavlink import MavlinkProcessController


def run(controller: "MavlinkProcessController") -> None:
    controller.phase = "LANDING"
    deadline = time.monotonic() + float(controller.cfg.landing.timeout_s)

    mode_ok = controller.mav.set_mode("AUTO.LAND")
    if not mode_ok:
        controller.logger.warning("[LANDING] AUTO.LAND mode name unavailable; sending NAV_LAND command")
        controller.mav.command_land()
    else:
        controller.logger.info("[LANDING] AUTO.LAND requested")

    while True:
        tick = time.monotonic()
        controller.poll_inputs()

        controller.current_cmd_vx = 0.0
        controller.current_cmd_vy = 0.0
        controller.current_cmd_vz = 0.0
        controller.publish_status()

        armed = controller.mav.get_is_armed()
        if armed is False:
            controller.logger.info("[LANDING] disarmed; landing complete")
            return

        if tick > deadline:
            controller.logger.warning("[LANDING] timeout waiting for disarm")
            return

        controller.sleep_to_next_tick(tick)
