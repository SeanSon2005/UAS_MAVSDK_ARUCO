from __future__ import annotations

import time

from camera.camera_gstreamer import GStreamerCamera
from camera.camera_picam2 import Picamera2Camera
from comms.protocol import PROTO_VERSION, VISION_SAMPLE_TYPE, VisionSample
from comms.udp import MavStatusReceiver, VisionSampleUdpSender
from config.config_loader import load_config
from src.logging.logger import get_logger
from vision.detect_color import ColorDetector
from vision.pid import PID
from vision.render_cv import PreviewRenderer


def _clamp(value: float, lim: float) -> float:
    return max(-lim, min(lim, value))


def _status_lines(status) -> list[str]:
    if status is None:
        return ["status=waiting"]

    pos = "None"
    if status.pos_n is not None and status.pos_e is not None and status.alt_m is not None:
        pos = f"N={status.pos_n:.2f} E={status.pos_e:.2f} Alt={status.alt_m:.2f}"

    flow = "None" if status.flow_quality is None else str(status.flow_quality)
    dist = "None" if status.distance_m is None else f"{status.distance_m:.2f}m"
    age = "None" if status.last_vision_age_s is None else f"{status.last_vision_age_s:.2f}s"

    return [
        f"phase={status.phase} mode={status.mode} armed={status.armed}",
        f"cmd_frd=({status.cmd_vx_f:.2f}, {status.cmd_vy_r:.2f}, {status.cmd_vz_d:.2f})",
        f"{pos}",
        f"flow_q={flow} dist={dist} vision_age={age}",
    ]


def run_vision_process() -> None:
    cfg = load_config()
    logger = get_logger("vision", cfg.logs.dir)

    if cfg.is_simulation:
        camera = GStreamerCamera.udp_h264(
            port=cfg.vision.gst_udp_port,
            payload=cfg.vision.gst_payload,
            latency_ms=cfg.vision.gst_latency_ms,
        )
        logger.info("[VISION] Camera source=GStreamer UDP port=%d", cfg.vision.gst_udp_port)
    else:
        camera = Picamera2Camera(logger)
        logger.info("[VISION] Camera source=Picamera2")

    detector = ColorDetector(
        red_lower_1=cfg.vision.red_lower_1,
        red_upper_1=cfg.vision.red_upper_1,
        red_lower_2=cfg.vision.red_lower_2,
        red_upper_2=cfg.vision.red_upper_2,
        min_area_px=cfg.vision.red_min_area_px,
        frame_color_order=cfg.vision.frame_color_order,
    )

    pid = PID(
        kp=cfg.alignment.pid_kp,
        ki=cfg.alignment.pid_ki,
        kd=cfg.alignment.pid_kd,
        target=(0.0, 0.0),
    )
    pid.vmax = cfg.runtime.max_speed_mps

    renderer = PreviewRenderer(
        logger=logger,
        window_name="Vision + Mavlink Status",
        enabled=True,
        frame_color_order=cfg.vision.frame_color_order,
    )

    sample_tx = VisionSampleUdpSender(cfg.ipc.host, cfg.ipc.vision_udp_port)
    status_rx = MavStatusReceiver(cfg.ipc.host, cfg.ipc.control_udp_port)

    seq = 0
    send_period_s = 1.0 / max(1.0, float(cfg.ipc.send_hz_cap))
    render_period_s = 1.0 / max(1.0, float(cfg.ipc.render_fps))
    last_send = 0.0
    last_render = 0.0
    prev_t = time.monotonic()

    try:
        camera.start()
        renderer.init_window()

        while True:
            frame = camera.capture_frame()
            if frame is None:
                time.sleep(0.001)
                continue

            now = time.monotonic()
            dt = max(1e-3, now - prev_t)
            prev_t = now

            (
                err_x_px,
                err_y_px,
                red_mask,
                centroid,
                contour,
                contour_area,
            ) = detector.detect_with_debug(frame)

            has_target = err_x_px is not None and err_y_px is not None
            if has_target:
                pid_out = pid.update((-float(err_x_px), -float(err_y_px)), dt)
                vx_cmd_f = _clamp(float(-pid_out[1]), cfg.runtime.max_speed_mps)
                vy_cmd_r = _clamp(float(pid_out[0]), cfg.runtime.max_speed_mps)
            else:
                pid.reset(target=(0.0, 0.0))
                vx_cmd_f = 0.0
                vy_cmd_r = 0.0

            h, w = frame.shape[:2]
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
                    vx_cmd_f_mps=float(vx_cmd_f),
                    vy_cmd_r_mps=float(vy_cmd_r),
                )
                sample_tx.send(sample)
                seq += 1
                last_send = now

            status = status_rx.poll()

            if now - last_render >= render_period_s:
                renderer.render(
                    frame=frame,
                    target_visible=has_target,
                    err_x=err_x_px,
                    err_y=err_y_px,
                    vx=vx_cmd_f,
                    vy=vy_cmd_r,
                    red_mask=red_mask,
                    centroid=centroid,
                    contour=contour,
                    contour_area=contour_area,
                    status_lines=_status_lines(status),
                )
                last_render = now

            time.sleep(0)
    except KeyboardInterrupt:
        logger.info("[VISION] Keyboard interrupt")
    finally:
        renderer.close_window()
        try:
            camera.stop()
        except Exception:
            pass
        sample_tx.close()
        status_rx.close()
