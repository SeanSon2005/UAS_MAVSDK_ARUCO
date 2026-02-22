from __future__ import annotations

import logging
import sys
from logging.handlers import RotatingFileHandler

from config.load import load_config


class LoggerFactory:
    _fmt = "%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s"

    def __init__(self, logs_dir):
        self._logs_dir = logs_dir

    def get(self, name: str, level: int = logging.INFO) -> logging.Logger:
        logger = logging.getLogger(name)
        logger.setLevel(level)
        if logger.handlers:
            return logger

        self._logs_dir.mkdir(parents=True, exist_ok=True)
        formatter = logging.Formatter(self._fmt)

        console = logging.StreamHandler(sys.stdout)
        console.setLevel(level)
        console.setFormatter(formatter)
        logger.addHandler(console)

        file_handler = RotatingFileHandler(
            self._logs_dir / f"{name}.log",
            maxBytes=10_485_760,
            backupCount=5,
            encoding="utf-8",
        )
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        return logger


def build_flight_controller():
    from mavsdk import System

    from comms.udp import ControlStatusUdpSender, VisionUdpReceiver
    from flight.controller import FlightController
    from hw.mavsdk_link import MavsdkLink
    from hw.servo_pwm import ServoManager
    from hw.telemetry import TelemetryCache

    cfg = load_config()
    logger = LoggerFactory(cfg.logs_dir).get("mavsdk")

    drone = System()
    flight_link = MavsdkLink(drone=drone, logger=logger)
    telemetry = TelemetryCache(drone=drone, logger=logger)
    vision_receiver = VisionUdpReceiver(host=cfg.ipc.host, port=cfg.ipc.vision_udp_port, logger=logger)
    status_sender = ControlStatusUdpSender(host=cfg.ipc.host, port=cfg.ipc.control_udp_port)
    actuator = ServoManager(
        logger=logger,
        pwm_chip=cfg.servo.pwm_chip,
        pwm_channel=cfg.servo.pwm_channel,
        steps_0_deg=cfg.servo.steps_0_deg,
        steps_180_deg=cfg.servo.steps_180_deg,
        angle_landing=cfg.servo.angle_landing,
    )

    return FlightController(
        cfg=cfg,
        logger=logger,
        flight_link=flight_link,
        telemetry=telemetry,
        vision_receiver=vision_receiver,
        status_sender=status_sender,
        actuator=actuator,
    )


def build_vision_controller():
    from comms.udp import ControlStatusReceiver, VisionSampleUdpSender
    from vision.camera_picam2 import Picamera2Camera
    from vision.controller import VisionController
    from vision.detect_color import ColorDetector
    from vision.render_cv import PreviewRenderer

    cfg = load_config()
    logger = LoggerFactory(cfg.logs_dir).get("vision")

    return VisionController(
        cfg=cfg,
        logger=logger,
        camera=Picamera2Camera(logger=logger),
        detector=ColorDetector(
            red_lower_1=cfg.vision.red_lower_1,
            red_upper_1=cfg.vision.red_upper_1,
            red_lower_2=cfg.vision.red_lower_2,
            red_upper_2=cfg.vision.red_upper_2,
            min_area_px=cfg.vision.red_min_area_px,
            frame_color_order=cfg.vision.frame_color_order,
        ),
        renderer=PreviewRenderer(
            logger=logger,
            window_name="Red Target Detection",
            enabled=True,
            frame_color_order=cfg.vision.frame_color_order,
        ),
        sample_sender=VisionSampleUdpSender(host=cfg.ipc.host, port=cfg.ipc.vision_udp_port),
        status_receiver=ControlStatusReceiver(host=cfg.ipc.host, port=cfg.ipc.control_udp_port),
    )
