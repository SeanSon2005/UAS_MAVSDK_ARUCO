from __future__ import annotations

import asyncio
import time

from comms.protocol import PROTO_VERSION, VISION_SAMPLE_TYPE, VisionSample


class VisionController:
    def __init__(self, cfg, logger, camera, detector, renderer, sample_sender, status_receiver):
        self.cfg = cfg
        self.logger = logger
        self.camera = camera
        self.detector = detector
        self.renderer = renderer
        self.sample_sender = sample_sender
        self.status_receiver = status_receiver

    async def run(self) -> None:
        seq = 0
        send_period_s = 1.0 / max(1.0, float(self.cfg.ipc.send_hz_cap))
        render_period_s = 1.0 / max(1.0, float(self.cfg.ipc.render_fps))
        last_send = 0.0
        last_render = 0.0

        await self.camera.start()
        self.renderer.init_window()

        try:
            while True:
                frame = await self.camera.capture_frame()
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
                ) = await asyncio.to_thread(self.detector.detect_with_debug, frame)

                has_target = err_x_px is not None and err_y_px is not None
                h, w = frame.shape[:2]

                now = time.monotonic()
                if now - last_send >= send_period_s:
                    sample = VisionSample(
                        v=PROTO_VERSION,
                        type=VISION_SAMPLE_TYPE,
                        seq=seq,
                        t_mono_ns=time.monotonic_ns(),
                        has_target=has_target,
                        err_x_px=float(err_x_px) if err_x_px is not None else None,
                        err_y_px=float(err_y_px) if err_y_px is not None else None,
                        contour_area=float(contour_area),
                        frame_w=int(w),
                        frame_h=int(h),
                    )
                    self.sample_sender.send(sample)
                    seq += 1
                    last_send = now

                status = self.status_receiver.poll()
                if now - last_render >= render_period_s and self.renderer.enabled:
                    status_lines = None
                    if status is not None:
                        status_lines = [
                            f"phase={status.phase}",
                            f"vx={status.vx:.2f} vy={status.vy:.2f} vz={status.vz:.2f}",
                            f"alt={status.alt_m:.2f}m",
                        ]

                    self.renderer.render(
                        frame,
                        has_target,
                        err_x_px,
                        err_y_px,
                        0.0,
                        0.0,
                        red_mask,
                        centroid,
                        contour,
                        contour_area,
                        status_lines=status_lines,
                    )
                    last_render = now

                await asyncio.sleep(0)
        finally:
            self.renderer.close_window()
            self.status_receiver.close()
            self.sample_sender.close()
            await self.camera.stop()
