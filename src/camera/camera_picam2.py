from __future__ import annotations

from .camera import Camera


class Picamera2Camera(Camera):
    def __init__(self, logger):
        self._logger = logger
        self._picam2 = None
        self._started = False

    def start(self) -> None:
        try:
            from picamera2 import Picamera2
        except Exception as exc:
            raise RuntimeError("picamera2 is not available") from exc

        self._picam2 = Picamera2()
        config = self._picam2.create_video_configuration(
            main={"size": (640, 480), "format": "BGR888"},
            controls={"FrameDurationLimits": (33333, 33333)},
        )
        self._picam2.configure(config)
        self._picam2.start()
        self._started = True
        self._logger.info("[VISION] Picamera2 started")

    def stop(self) -> None:
        if self._picam2 is not None and self._started:
            self._picam2.stop()
            self._started = False
            self._logger.info("[VISION] Picamera2 stopped")

    def capture_frame(self):
        if self._picam2 is None or not self._started:
            return None
        try:
            return self._picam2.capture_array("main")
        except Exception:
            return None
