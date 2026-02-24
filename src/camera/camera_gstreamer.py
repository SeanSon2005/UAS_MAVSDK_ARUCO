from __future__ import annotations

from typing import Optional, Tuple, Union
import cv2
import numpy as np
from .camera import Camera


class GStreamerCamera(Camera):
    """
    OpenCV + GStreamer camera capture.

    Typical usage:
        cam = GStreamerCamera.udp_h264(port=5600)
        cam.start()
        frame = cam.capture_frame()
        cam.stop()

    Notes:
    - Requires OpenCV built with GStreamer support (`cv2.CAP_GSTREAMER`).
    - For UDP H264 streams, the sender must be RTP/H264 with matching caps.
    """

    def __init__(
        self,
        pipeline: str,
        *,
        api_preference: int = cv2.CAP_GSTREAMER,
        warmup_frames: int = 5,
        read_timeout_frames: int = 30,
        resize: Optional[Tuple[int, int]] = None,  # (width, height)
    ):
        self.pipeline = pipeline
        self.api_preference = api_preference
        self.warmup_frames = max(0, int(warmup_frames))
        self.read_timeout_frames = max(1, int(read_timeout_frames))
        self.resize = resize

        self._cap: Optional[cv2.VideoCapture] = None
        self._started: bool = False

    @staticmethod
    def udp_h264(
        port: int,
        *,
        payload: int = 96,
        latency_ms: int = 0,
        drop: bool = True,
        decode: str = "avdec_h264",
    ) -> "GStreamerCamera":
        """
        Build a pipeline for RTP/H264 over UDP.

        Many Gazebo GstCameraSystem setups stream RTP/H264. If yours differs (H265, MJPEG, raw),
        adjust accordingly.

        Args:
            port: UDP port to listen on
            payload: RTP payload type (commonly 96)
            latency_ms: jitterbuffer latency
            drop: appsink drops old frames if consumer is slow
            decode: decoder element (avdec_h264 or nvh264dec if available)
        """
        # rtpjitterbuffer can help when packets are bursty; latency 0 for lowest latency.
        # appsink drop=1 to keep newest frame.
        pipeline = (
            f"udpsrc port={int(port)} "
            f'caps="application/x-rtp,media=video,encoding-name=H264,payload={int(payload)}" ! '
            f"rtpjitterbuffer latency={int(latency_ms)} ! "
            f"rtph264depay ! h264parse ! {decode} ! videoconvert ! "
            f"appsink sync=false max-buffers=1 {'drop=true' if drop else 'drop=false'}"
        )
        return GStreamerCamera(pipeline=pipeline)

    @staticmethod
    def device(
        device: Union[int, str] = 0,
        *,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
    ) -> "GStreamerCamera":
        """
        Capture from a local camera device via OpenCV (not UDP/GStreamer pipeline).
        Useful for quick swapping between sim and real hardware.

        Args:
            device: index (0,1,2...) or path like "/dev/video0"
        """
        # This uses VideoCapture device opening, not a GStreamer pipeline string.
        cam = GStreamerCamera(pipeline="", api_preference=cv2.CAP_ANY)
        cam._device = device  # type: ignore[attr-defined]
        cam._device_settings = (width, height, fps)  # type: ignore[attr-defined]
        return cam

    def start(self) -> None:
        if self._started:
            return

        # Open either pipeline or device
        if getattr(self, "_device", None) is not None:
            self._cap = cv2.VideoCapture(getattr(self, "_device"), self.api_preference)
            width, height, fps = getattr(self, "_device_settings")
            if width:
                self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            if height:
                self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            if fps:
                self._cap.set(cv2.CAP_PROP_FPS, int(fps))
        else:
            if not self.pipeline.strip():
                raise ValueError("GStreamerCamera.start(): pipeline is empty. Use udp_h264(...) or provide a pipeline.")
            self._cap = cv2.VideoCapture(self.pipeline, self.api_preference)

        if self._cap is None or not self._cap.isOpened():
            self._cap = None
            raise RuntimeError(
                "Failed to open camera capture.\n"
                "If using GStreamer:\n"
                "  - Ensure OpenCV has GStreamer support.\n"
                "  - Verify your UDP port / caps / encoding match the sender.\n"
                f"Pipeline: {self.pipeline}"
            )

        # Warm up buffer
        for _ in range(self.warmup_frames):
            ok, _ = self._cap.read()
            if not ok:
                break

        self._started = True

    def stop(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            finally:
                self._cap = None
        self._started = False

    def capture_frame(self) -> np.ndarray:
        """
        Returns the latest frame as a BGR uint8 numpy array.

        Raises:
            RuntimeError if camera not started or no frame received in time.
        """
        if not self._started or self._cap is None:
            raise RuntimeError("Camera is not started. Call start() first.")

        # Try a few reads to handle transient drops
        for _ in range(self.read_timeout_frames):
            ok, frame = self._cap.read()
            if ok and frame is not None:
                if self.resize is not None:
                    w, h = self.resize
                    frame = cv2.resize(frame, (int(w), int(h)), interpolation=cv2.INTER_AREA)
                return frame

        raise RuntimeError("Timed out waiting for a camera frame.")

    # ---------- Convenience ----------
    def __enter__(self) -> "GStreamerCamera":
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()