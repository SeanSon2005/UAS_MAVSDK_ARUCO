from __future__ import annotations

import asyncio

from picamera2 import Picamera2
from camera import Camera


class Picamera2Camera(Camera):
    def __init__(self, logger):
        self._logger = logger
        self._picam2 = None
        self._started = False

    async def start(self) -> None:
        self._picam2 = Picamera2()
        config = self._picam2.create_video_configuration(
            main={"size": (640, 480), "format": "BGR888"},
            controls={"FrameDurationLimits": (33333, 33333)},
        )
        self._picam2.configure(config)
        self._picam2.start()
        self._started = True
        self._logger.info("[VISION] Camera started")

    async def stop(self) -> None:
        if self._picam2 and self._started:
            self._picam2.stop()
            self._started = False
            self._logger.info("[VISION] Camera stopped")

    async def capture_frame(self):
        if not self._picam2 or not self._started:
            return None
        try:
            return await asyncio.to_thread(self._picam2.capture_array, "main")
        except Exception:
            return None
