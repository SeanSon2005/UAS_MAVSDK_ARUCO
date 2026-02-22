#!/usr/bin/env python3

import asyncio

from app.wiring import LoggerFactory
from config.load import load_config
from flight.flight_states.common import AlignPolicyInput, compute_xy_from_vision
from flight.pid import PID
from flight.state import VisionState
from vision.camera_picam2 import Picamera2Camera
from vision.detect_color import ColorDetector
from vision.render_cv import PreviewRenderer


async def run_aligner_preview() -> None:
    cfg = load_config()
    logger = LoggerFactory(cfg.logs_dir).get("vision")

    camera = Picamera2Camera(logger=logger)
    detector = ColorDetector(
        red_lower_1=cfg.vision.red_lower_1,
        red_upper_1=cfg.vision.red_upper_1,
        red_lower_2=cfg.vision.red_lower_2,
        red_upper_2=cfg.vision.red_upper_2,
        min_area_px=cfg.vision.red_min_area_px,
        frame_color_order=cfg.vision.frame_color_order,
    )
    renderer = PreviewRenderer(
        logger=logger,
        window_name="Aligner Test (Red Target)",
        enabled=True,
        frame_color_order=cfg.vision.frame_color_order,
    )

    pid = PID(
        kp=cfg.alignment.pid_kp,
        ki=cfg.alignment.pid_ki,
        kd=cfg.alignment.pid_kd,
        target=(0.0, 0.0),
    )
    pid.vmax = cfg.runtime.max_speed_mps
    vision_state = VisionState()

    dt = 1.0 / cfg.runtime.loop_hz
    await camera.start()
    renderer.init_window()

    try:
        while renderer.enabled:
            frame = await camera.capture_frame()
            if frame is None:
                await asyncio.sleep(0.001)
                continue

            (
                err_x_px,
                err_y_px,
                red_mask,
                centroid,
                contour,
                contour_area,
            ) = await asyncio.to_thread(detector.detect_with_debug, frame)

            sample = type(
                "Sample",
                (),
                {
                    "has_target": err_x_px is not None and err_y_px is not None,
                    "err_x_px": err_x_px,
                    "err_y_px": err_y_px,
                },
            )()

            out = compute_xy_from_vision(
                AlignPolicyInput(
                    sample=sample,
                    vision_age_s=0.0,
                    dt=dt,
                    max_speed_mps=cfg.runtime.max_speed_mps,
                    sample_ttl_s=cfg.ipc.sample_ttl_s,
                    drop_to_zero_after_s=cfg.ipc.drop_to_zero_after_s,
                ),
                state=vision_state,
                pid=pid,
            )

            renderer.render(
                frame,
                out.target_visible,
                out.err_x_px,
                out.err_y_px,
                out.cmd.vx,
                out.cmd.vy,
                red_mask,
                centroid,
                contour,
                contour_area,
                status_lines=[f"mode={out.mode.value}"],
            )
            await asyncio.sleep(dt)
    finally:
        renderer.close_window()
        await camera.stop()


if __name__ == "__main__":
    asyncio.run(run_aligner_preview())
