from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from pymavlink import mavutil

from comms.protocol import MAV_STATUS_TYPE, PROTO_VERSION, MavlinkStatus, VisionSample
from comms.udp import MavStatusUdpSender, VisionSampleReceiver
from config.config_loader import UasConfig, load_config
from mavlink.mavlink import MavLink
from missions import alignment, landing, takeoff
from src.logging.logger import get_logger


@dataclass(slots=True)
class CommandState:
    vx_f: float = 0.0
    vy_r: float = 0.0
    vz_d: float = 0.0


class MavlinkProcessController:
    def __init__(self, cfg: UasConfig):
        self.cfg = cfg
        self.logger = get_logger("mavlink", cfg.logs.dir)

        conn = cfg.drone.sim_connection_string if cfg.is_simulation else cfg.drone.connection_string
        self.mav = MavLink(conn)

        self.vision_rx = VisionSampleReceiver(cfg.ipc.host, cfg.ipc.vision_udp_port)
        self.status_tx = MavStatusUdpSender(cfg.ipc.host, cfg.ipc.control_udp_port)

        self.phase = "INIT"
        self.offboard_active = False

        self.latest_vision_sample: Optional[VisionSample] = None
        self.latest_vision_rx_mono: Optional[float] = None
        self.last_valid_vx = 0.0
        self.last_valid_vy = 0.0

        self._period_s = 1.0 / max(1.0, float(cfg.runtime.loop_hz))

        self.current_cmd_vx = 0.0
        self.current_cmd_vy = 0.0
        self.current_cmd_vz = 0.0

    def get_altitude_m(self) -> Optional[float]:
        local = self.mav.get_local_position_ned()
        if local is None:
            return None
        return float(-local["z"])

    def get_vision_age_s(self) -> Optional[float]:
        if self.latest_vision_rx_mono is None:
            return None
        return time.monotonic() - self.latest_vision_rx_mono

    def poll_inputs(self) -> None:
        self.mav.poll(timeout_s=0.0)
        sample = self.vision_rx.poll()
        if sample is not None:
            if self.latest_vision_sample is None or sample.seq > self.latest_vision_sample.seq:
                self.latest_vision_sample = sample
                self.latest_vision_rx_mono = time.monotonic()

    def sleep_to_next_tick(self, tick_start_mono: float) -> None:
        remaining = self._period_s - (time.monotonic() - tick_start_mono)
        if remaining > 0:
            time.sleep(remaining)

    def publish_status(self) -> None:
        local = self.mav.get_local_position_ned()
        if local is None:
            pos_n = pos_e = pos_d = vel_n = vel_e = vel_d = None
            alt_m = None
        else:
            pos_n = float(local["x"])
            pos_e = float(local["y"])
            pos_d = float(local["z"])
            vel_n = float(local["vx"])
            vel_e = float(local["vy"])
            vel_d = float(local["vz"])
            alt_m = -pos_d

        status = MavlinkStatus(
            v=PROTO_VERSION,
            type=MAV_STATUS_TYPE,
            phase=self.phase,
            armed=bool(self.mav.get_is_armed()),
            offboard_active=self.offboard_active,
            mode=self.mav.get_flight_mode() or "UNKNOWN",
            pos_n=pos_n,
            pos_e=pos_e,
            pos_d=pos_d,
            vel_n=vel_n,
            vel_e=vel_e,
            vel_d=vel_d,
            alt_m=alt_m,
            flow_quality=self.mav.get_flow_quality(),
            distance_m=self.mav.get_distance_m(),
            last_vision_age_s=self.get_vision_age_s(),
            cmd_vx_f=float(self.current_cmd_vx),
            cmd_vy_r=float(self.current_cmd_vy),
            cmd_vz_d=float(self.current_cmd_vz),
            t_mono_ns=time.monotonic_ns(),
        )
        self.status_tx.send(status)

    def setup(self) -> None:
        self.logger.info("[MAVLINK] Connecting to %s", self.mav.connection_string)
        self.mav.connect(timeout_s=15.0)
        self.logger.info("[MAVLINK] Connected. body_frame=%s", self.mav.get_body_frame_name())

        self.mav.start_streams()

        self.logger.info("[MAVLINK] Arming")
        if not self.mav.arm(timeout_s=8.0):
            raise RuntimeError("Failed to arm")

        self.mav.prime_offboard(duration_s=1.0, hz=20)

        self.offboard_active = True

    def teardown(self) -> None:
        self.phase = "SHUTDOWN"
        for _ in range(5):
            try:
                self.current_cmd_vx = 0.0
                self.current_cmd_vy = 0.0
                self.current_cmd_vz = 0.0
                self.mav.set_velocity_frd(0.0, 0.0, 0.0)
                self.publish_status()
                time.sleep(0.02)
            except Exception:
                break

        try:
            self.status_tx.close()
        except Exception:
            pass

        try:
            self.vision_rx.close()
        except Exception:
            pass

        if self.mav.m is not None:
            try:
                self.mav.m.close()
            except Exception:
                pass

    def run(self) -> None:
        did_setup = False
        try:
            self.setup()
            did_setup = True

            takeoff.run(self)
            alignment.run(self)
            landing.run(self)
        except KeyboardInterrupt:
            self.logger.info("[MAVLINK] Keyboard interrupt")
            if did_setup:
                try:
                    landing.run(self)
                except Exception:
                    pass
        except Exception as exc:
            self.logger.exception("[MAVLINK] Run failure: %s", exc)
            if did_setup:
                try:
                    landing.run(self)
                except Exception:
                    pass
            raise
        finally:
            self.teardown()


def run_mavlink_process() -> None:
    cfg = load_config()
    MavlinkProcessController(cfg).run()
