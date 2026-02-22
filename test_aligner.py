#!/usr/bin/env python3

import asyncio
import numpy as np

from camera import Camera
from color import ColorDetector
from constants import LOOP_HZ, MAX_SPEED_MPS, ALIGN_PID_KP, ALIGN_PID_KI, ALIGN_PID_KD
from pid import PID
from renderer import PreviewRenderer

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

async def run_test_aligner():
    camera = Camera()
    detector = ColorDetector()
    renderer = PreviewRenderer(window_name="Aligner Test (Red Target)", enabled=True)

    pid = PID(
        kp=ALIGN_PID_KP,
        ki=ALIGN_PID_KI,
        kd=ALIGN_PID_KD,
        target=(0.0, 0.0),
    )
    pid.vmax = MAX_SPEED_MPS

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()

    await camera.start_camera()
    renderer.init_window()

    try:
        while renderer.enabled:
            tick_start = loop.time()
            vx = 0.0
            vy = 0.0
            target_visible = False

            frame = await camera.capture_frame()
            if frame is not None:
                (
                    err_x_px,
                    err_y_px,
                    red_mask,
                    centroid,
                    contour,
                    contour_area,
                ) = await asyncio.to_thread(detector.detect_with_debug, frame)

                if err_x_px is not None and err_y_px is not None:
                    target_visible = True
                    measurement = np.array([-err_x_px, -err_y_px], dtype=float)
                    pid_out = pid.update(measurement, dt)
                    vx = clamp(float(-pid_out[1]), -MAX_SPEED_MPS, MAX_SPEED_MPS)
                    vy = clamp(float(pid_out[0]), -MAX_SPEED_MPS, MAX_SPEED_MPS)
                else:
                    pid.reset(target=(0.0, 0.0))
                    contour_area = contour_area if contour_area is not None else 0.0

                renderer.render(
                    frame,
                    target_visible,
                    err_x_px,
                    err_y_px,
                    vx,
                    vy,
                    red_mask,
                    centroid,
                    contour,
                    contour_area,
                )

            print(f"[TEST ALIGN] visible={target_visible} vx={vx:.2f} vy={vy:.2f}")

            remaining = dt - (loop.time() - tick_start)
            if remaining > 0:
                await asyncio.sleep(remaining)
    finally:
        renderer.close_window()
        await camera.stop_camera()

if __name__ == "__main__":
    asyncio.run(run_test_aligner())