#!/usr/bin/env python3
"""
mavlink.py

A small PX4-friendly MAVLink helper built on pymavlink.

Features:
- Connect via a MAVLink connection string (e.g., "udp:127.0.0.1:14540")
- Set OFFBOARD-friendly setpoints:
  - set_position_ned(n,e,d,yaw_rad=None)
  - set_velocity_ned(vn,ve,vd,yaw_rad=None)
- Getters:
  - get_flow_quality()  -> int | None  (0..255 from OPTICAL_FLOW_RAD)
  - get_distance_m()    -> float | None (from DISTANCE_SENSOR)
  - get_local_position_ned() -> dict | None (from LOCAL_POSITION_NED)
  - get_attitude()      -> dict | None (from ATTITUDE)
- Request message rates with MAV_CMD_SET_MESSAGE_INTERVAL
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any

from pymavlink import mavutil


def _u32_boot_ms() -> int:
    # uint32 wrapping monotonic milliseconds (safe for MAVLink fields)
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


@dataclass
class FlowSample:
    quality: int
    integrated_x: float
    integrated_y: float
    integration_time_us: int
    distance_m: Optional[float] = None


class MavLink:
    """
    Minimal MAVLink wrapper for PX4 SITL/HITL/companion scripting.
    """

    def __init__(
        self,
        connection_string: str,
        *,
        baud: Optional[int] = None,
        source_system: int = 255,
        source_component: int = 0,
        autoreconnect: bool = True,
        dialect: str = "ardupilotmega",
    ):
        """
        Args:
            connection_string: e.g. "udp:127.0.0.1:14540", "serial:/dev/ttyUSB0:921600"
            baud: optional for serial connections
            source_system/component: your GCS/companion sysid/compid
            autoreconnect: pymavlink reconnect behavior
            dialect: pymavlink dialect; "ardupilotmega" is most commonly installed
        """
        self.connection_string = connection_string
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.autoreconnect = autoreconnect
        self.dialect = dialect

        self.m: Optional[mavutil.mavlink_connection] = None
        self.target_system: Optional[int] = None
        self.target_component: Optional[int] = None

        # last-seen caches
        self._last_flow: Optional[FlowSample] = None
        self._last_distance_m: Optional[float] = None
        self._last_local_pos: Optional[Dict[str, float]] = None
        self._last_attitude: Optional[Dict[str, float]] = None

    # ------------------------
    # Connection / Setup
    # ------------------------
    def connect(self, timeout_s: float = 10.0) -> None:
        """
        Open the MAVLink connection and wait for heartbeat.
        """
        kwargs = dict(
            autoreconnect=self.autoreconnect,
            source_system=self.source_system,
            source_component=self.source_component,
        )
        if self.baud is not None:
            kwargs["baud"] = self.baud

        self.m = mavutil.mavlink_connection(self.connection_string, **kwargs)

        t0 = time.time()
        while True:
            if time.time() - t0 > timeout_s:
                raise TimeoutError(f"Timeout waiting for heartbeat on {self.connection_string}")

            hb = self.m.wait_heartbeat(timeout=1.0)
            if hb is not None:
                self.target_system = self.m.target_system
                self.target_component = self.m.target_component
                return

    def request_message_interval(self, msg_id: int, rate_hz: float) -> None:
        """
        Request a MAVLink message at a given rate using MAV_CMD_SET_MESSAGE_INTERVAL.
        PX4 may ignore unsupported requests depending on MAVLink instance.
        """
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        if rate_hz <= 0:
            interval_us = -1  # disable
        else:
            interval_us = int(1e6 / rate_hz)

        self.m.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),
            float(interval_us),
            0, 0, 0, 0, 0,
        )

    def prime_offboard(self, duration_s: float = 1.0, hz: int = 20) -> None:
        """
        Send neutral setpoints for a short time. PX4 requires setpoints before OFFBOARD.
        """
        dt = 1.0 / hz
        t0 = time.time()
        while time.time() - t0 < duration_s:
            self.set_position_ned(0.0, 0.0, 0.0, yaw_rad=None)
            time.sleep(dt)

    # ------------------------
    # Modes / Arming
    # ------------------------
    def set_mode(self, mode_name: str) -> bool:
        """
        Set vehicle mode by name (PX4 typically supports 'OFFBOARD', 'POSCTL', 'AUTO.LAND', etc.)
        Returns True if the mode exists in mapping and command sent.
        """
        if not self.m:
            raise RuntimeError("Not connected. Call connect() first.")
        mapping = self.m.mode_mapping()
        if not mapping or mode_name not in mapping:
            return False
        self.m.set_mode(mode_name)
        return True

    def arm(self, timeout_s: float = 5.0) -> bool:
        return self._arm_disarm(True, timeout_s=timeout_s)

    def disarm(self, timeout_s: float = 5.0) -> bool:
        return self._arm_disarm(False, timeout_s=timeout_s)

    def _arm_disarm(self, arm_it: bool, timeout_s: float) -> bool:
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self.m.mav.command_long_send(
            self.target_system,
            self.target_component,
            cmd,
            0,
            1.0 if arm_it else 0.0,
            0, 0, 0, 0, 0, 0,
        )

        # Wait for COMMAND_ACK (optional but helpful)
        t0 = time.time()
        ack_result = None
        while time.time() - t0 < timeout_s:
            ack = self.m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack and ack.command == cmd:
                ack_result = int(ack.result)
                break

        # Wait for HEARTBEAT armed flag to reflect desired state
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if not hb:
                continue
            is_armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if is_armed == arm_it:
                return True

        # Common outcomes:
        # 0 ACCEPTED, 1 TEMPORARILY_REJECTED, 2 DENIED
        if ack_result is not None:
            return False
        return False

    # ------------------------
    # Setpoints (NED)
    # ------------------------
    def set_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_rad: Optional[float] = None) -> None:
        """
        Send SET_POSITION_TARGET_LOCAL_NED position setpoint (position-only).
        NED: down is positive. To go up 1.5m, down = -1.5.
        """
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        if yaw_rad is None or math.isnan(yaw_rad):
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw_rad = 0.0

        self.m.mav.set_position_target_local_ned_send(
            _u32_boot_ms(),
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            north_m, east_m, down_m,
            0.0, 0.0, 0.0,    # vx,vy,vz ignored
            0.0, 0.0, 0.0,    # ax,ay,az ignored
            float(yaw_rad),
            0.0
        )

    def set_velocity_ned(self, vn_mps: float, ve_mps: float, vd_mps: float, yaw_rad: Optional[float] = None) -> None:
        """
        Send SET_POSITION_TARGET_LOCAL_NED velocity setpoint (velocity-only).
        """
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        if yaw_rad is None or math.isnan(yaw_rad):
            type_mask |= mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw_rad = 0.0

        self.m.mav.set_position_target_local_ned_send(
            _u32_boot_ms(),
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0.0, 0.0, 0.0,           # position ignored
            vn_mps, ve_mps, vd_mps,   # velocity
            0.0, 0.0, 0.0,            # accel ignored
            float(yaw_rad),
            0.0
        )

    # ------------------------
    # Getters / Telemetry
    # ------------------------
    def poll(self, timeout_s: float = 0.0) -> None:
        """
        Poll incoming MAVLink messages and update caches.
        Call this periodically in your main loop.
        """
        if not self.m:
            raise RuntimeError("Not connected. Call connect() first.")

        end = time.time() + max(0.0, timeout_s)
        while True:
            remaining = end - time.time()
            if timeout_s == 0.0:
                remaining = 0.0

            msg = self.m.recv_match(blocking=timeout_s > 0.0, timeout=max(0.0, remaining))
            if msg is None:
                return

            mtype = msg.get_type()
            if mtype == "OPTICAL_FLOW_RAD":
                # quality 0..255
                dist = getattr(msg, "distance", None)
                self._last_flow = FlowSample(
                    quality=int(msg.quality),
                    integrated_x=float(msg.integrated_x),
                    integrated_y=float(msg.integrated_y),
                    integration_time_us=int(msg.integration_time_us),
                    distance_m=float(dist) if dist is not None else None,
                )
            elif mtype == "DISTANCE_SENSOR":
                # current_distance in meters
                self._last_distance_m = float(msg.current_distance)
            elif mtype == "LOCAL_POSITION_NED":
                self._last_local_pos = {
                    "x": float(msg.x),
                    "y": float(msg.y),
                    "z": float(msg.z),
                    "vx": float(msg.vx),
                    "vy": float(msg.vy),
                    "vz": float(msg.vz),
                }
            elif mtype == "ATTITUDE":
                self._last_attitude = {
                    "roll": float(msg.roll),
                    "pitch": float(msg.pitch),
                    "yaw": float(msg.yaw),
                    "rollspeed": float(msg.rollspeed),
                    "pitchspeed": float(msg.pitchspeed),
                    "yawspeed": float(msg.yawspeed),
                }

            # If non-blocking, consume whatever is immediately available
            if timeout_s == 0.0:
                continue

    def get_flow_quality(self) -> Optional[int]:
        """
        Returns optical flow quality (0..255) if we've received OPTICAL_FLOW_RAD.
        Make sure you call request_message_interval(MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, hz)
        and/or call poll() regularly.
        """
        return self._last_flow.quality if self._last_flow else None

    def get_flow(self) -> Optional[FlowSample]:
        return self._last_flow

    def get_distance_m(self) -> Optional[float]:
        return self._last_distance_m

    def get_local_position_ned(self) -> Optional[Dict[str, float]]:
        return self._last_local_pos

    def get_attitude(self) -> Optional[Dict[str, float]]:
        return self._last_attitude


# ------------------------
# Example usage (optional)
# ------------------------
if __name__ == "__main__":
    # Example: print flow quality at 10 Hz
    ml = MavLink("udp:127.0.0.1:14540")
    ml.connect()

    # Ask PX4 to stream these messages (may depend on MAVLink instance/port)
    ml.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, 20)
    ml.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 20)
    ml.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)

    t0 = time.time()
    while True:
        ml.poll(timeout_s=0.1)
        q = ml.get_flow_quality()
        dist = ml.get_distance_m()
        lp = ml.get_local_position_ned()
        if q is not None:
            print(f"flow_quality={q:3d}  dist={dist if dist is not None else float('nan'):.3f}  "
                  f"pos=({lp['x']:.2f},{lp['y']:.2f},{lp['z']:.2f})" if lp else "")
        if time.time() - t0 > 60:
            break