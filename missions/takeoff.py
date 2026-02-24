from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.processes.mavlink import MavlinkProcessController


def run(controller: "MavlinkProcessController") -> None:
    controller.phase = "TAKEOFF"
    cfg = controller.cfg.takeoff

    target_alt_m = float(cfg.alt_m)
    target_down_m = -target_alt_m
    hold_started_at: float | None = None
    deadline = time.monotonic() + float(cfg.timeout_s)

    controller.logger.info(
        "[TAKEOFF] target_alt=%.2fm tolerance=%.2fm hold=%.2fs timeout=%.2fs",
        target_alt_m,
        cfg.alt_tolerance_m,
        cfg.hold_s,
        cfg.timeout_s,
    )

    while True:
        tick = time.monotonic()
        controller.poll_inputs()

        controller.current_cmd_vx = 0.0
        controller.current_cmd_vy = 0.0
        controller.current_cmd_vz = 0.0
        controller.mav.set_position_ned(0.0, 0.0, target_down_m)

        alt = controller.get_altitude_m()
        controller.publish_status()

        if alt is not None:
            err = abs(target_alt_m - alt)
            if err <= cfg.alt_tolerance_m:
                if hold_started_at is None:
                    hold_started_at = tick
                if tick - hold_started_at >= cfg.hold_s:
                    controller.logger.info("[TAKEOFF] complete alt=%.2fm", alt)
                    return
            else:
                hold_started_at = None

        if tick > deadline:
            raise TimeoutError("[TAKEOFF] timeout waiting for altitude")

        controller.sleep_to_next_tick(tick)
