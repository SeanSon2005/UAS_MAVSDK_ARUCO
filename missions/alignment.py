from __future__ import annotations

import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from src.processes.mavlink import MavlinkProcessController


def _clamp(value: float, lim: float) -> float:
    return max(-lim, min(lim, value))


def run(controller: "MavlinkProcessController") -> None:
    controller.phase = "ALIGNMENT"
    cfg = controller.cfg
    deadline = time.monotonic() + float(cfg.alignment.timeout_s)
    hold_started_at: float | None = None

    success_err_x = float(cfg.alignment.success_err_px[0])
    success_err_y = float(cfg.alignment.success_err_px[1])

    controller.logger.info(
        "[ALIGN] timeout=%.2fs success_err=(%.1f,%.1f) hold=%.2fs",
        cfg.alignment.timeout_s,
        success_err_x,
        success_err_y,
        cfg.alignment.success_hold_s,
    )

    while True:
        tick = time.monotonic()
        controller.poll_inputs()

        sample = controller.latest_vision_sample
        age_s = controller.get_vision_age_s()

        cmd_vx = 0.0
        cmd_vy = 0.0
        target_visible = False
        err_x = None
        err_y = None

        if sample is not None and age_s is not None:
            if age_s <= cfg.ipc.sample_ttl_s and sample.has_target:
                cmd_vx = float(sample.vx_cmd_f_mps)
                cmd_vy = float(sample.vy_cmd_r_mps)
                controller.last_valid_vx = cmd_vx
                controller.last_valid_vy = cmd_vy
                target_visible = True
                err_x = sample.err_x_px
                err_y = sample.err_y_px
            elif age_s <= cfg.ipc.drop_to_zero_after_s:
                cmd_vx = controller.last_valid_vx
                cmd_vy = controller.last_valid_vy

        cmd_vx = _clamp(cmd_vx, cfg.runtime.max_speed_mps)
        cmd_vy = _clamp(cmd_vy, cfg.runtime.max_speed_mps)

        controller.current_cmd_vx = cmd_vx
        controller.current_cmd_vy = cmd_vy
        controller.current_cmd_vz = 0.0
        controller.mav.set_velocity_frd(cmd_vx, cmd_vy, 0.0)
        controller.publish_status()

        if target_visible and err_x is not None and err_y is not None:
            if abs(float(err_x)) <= success_err_x and abs(float(err_y)) <= success_err_y:
                if hold_started_at is None:
                    hold_started_at = tick
                if tick - hold_started_at >= cfg.alignment.success_hold_s:
                    controller.logger.info("[ALIGN] complete")
                    return
            else:
                hold_started_at = None
        else:
            hold_started_at = None

        if tick > deadline:
            controller.logger.warning("[ALIGN] timeout; transitioning to landing")
            return

        controller.sleep_to_next_tick(tick)
