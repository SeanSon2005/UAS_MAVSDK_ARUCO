#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

from pymavlink import mavutil


def _u32_boot_ms() -> int:
    return int(time.monotonic() * 1000) & 0xFFFFFFFF


@dataclass(slots=True)
class FlowSample:
    quality: int
    integrated_x: float
    integrated_y: float
    integration_time_us: int
    distance_m: Optional[float] = None


class MavLink:
    def __init__(
        self,
        connection_string: str,
        *,
        baud: Optional[int] = None,
        source_system: int = 255,
        source_component: int = 0,
        autoreconnect: bool = True
    ):
        self.connection_string = connection_string
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.autoreconnect = autoreconnect

        self.m: Optional[mavutil.mavlink_connection] = None
        self.target_system: Optional[int] = None
        self.target_component: Optional[int] = None

        self._last_flow: Optional[FlowSample] = None
        self._last_distance_m: Optional[float] = None
        self._last_local_pos: Optional[Dict[str, float]] = None
        self._last_attitude: Optional[Dict[str, float]] = None
        self._last_armed: Optional[bool] = None
        self._last_mode: Optional[str] = None

        frame_frd = getattr(mavutil.mavlink, "MAV_FRAME_BODY_FRD", None)
        if frame_frd is not None:
            self._body_frame = int(frame_frd)
            self._body_frame_name = "BODY_FRD"
        else:
            self._body_frame = int(getattr(mavutil.mavlink, "MAV_FRAME_BODY_NED"))
            self._body_frame_name = "BODY_NED"

    def connect(self, timeout_s: float = 10.0) -> None:
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
                self._last_armed = (
                    hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ) != 0
                try:
                    self._last_mode = str(self.m.flightmode)
                except Exception:
                    self._last_mode = None
                break
            
    def start_streams(self, *, hz: float = 20.0) -> None:
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, hz)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, hz)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10.0)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, hz)
        self.request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD, hz)

    def request_message_interval(self, msg_id: int, rate_hz: float) -> None:
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        interval_us = -1 if rate_hz <= 0 else int(1e6 / rate_hz)
        self.m.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),
            float(interval_us),
            0,
            0,
            0,
            0,
            0,
        )

    def prime_offboard(self, duration_s: float = 1.0, hz: int = 20) -> None:
        period = 1.0 / max(1, int(hz))
        t0 = time.time()
        while time.time() - t0 < duration_s:
            self.set_velocity_frd(0.0, 0.0, 0.0)
            time.sleep(period)

        PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6
        self.m.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(PX4_CUSTOM_MAIN_MODE_OFFBOARD),
            0, 0, 0, 0, 0
        )

    def arm(self, timeout_s: float = 5.0) -> bool:
        return self._arm_disarm(True, timeout_s)

    def disarm(self, timeout_s: float = 5.0) -> bool:
        return self._arm_disarm(False, timeout_s)

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
            0,
            0,
            0,
            0,
            0,
            0,
        )

        t0 = time.time()
        while time.time() - t0 < timeout_s:
            self.poll(timeout_s=0.2)
            if self._last_armed is arm_it:
                return True
        return False

    def command_land(self) -> None:
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")
        self.m.mav.command_long_send(
            self.target_system,
            self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def set_position_ned(self, north_m: float, east_m: float, down_m: float, yaw_rad: Optional[float] = None) -> None:
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
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
            float(north_m),
            float(east_m),
            float(down_m),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            float(yaw_rad),
            0.0,
        )

    def set_velocity_ned(self, vn_mps: float, ve_mps: float, vd_mps: float, yaw_rad: Optional[float] = None) -> None:
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
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
            0.0,
            0.0,
            0.0,
            float(vn_mps),
            float(ve_mps),
            float(vd_mps),
            0.0,
            0.0,
            0.0,
            float(yaw_rad),
            0.0,
        )

    def set_velocity_frd(self, vf_mps: float, vr_mps: float, vd_mps: float) -> None:
        if not self.m or self.target_system is None or self.target_component is None:
            raise RuntimeError("Not connected. Call connect() first.")

        body_frame = mavutil.mavlink.MAV_FRAME_BODY_NED  # <-- supported by PX4

        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        self.m.mav.set_position_target_local_ned_send(
            _u32_boot_ms(),
            self.target_system,
            self.target_component,
            body_frame,
            type_mask,
            0.0, 0.0, 0.0,
            float(vf_mps), float(vr_mps), float(vd_mps),
            0.0, 0.0, 0.0,
            0.0, 0.0
        )

    def poll(self, timeout_s: float = 0.01) -> None:
        if not self.m:
            raise RuntimeError("Not connected. Call connect() first.")

        if timeout_s <= 0.0:
            timeout_s = 0.01
        deadline = time.time() + timeout_s

        while True:
            remaining = max(0.0, deadline - time.time())  # type: ignore[operator]
            if remaining <= 0.0:
                return
            msg = self.m.recv_match(blocking=True, timeout=remaining)
            
            if msg is None:
                return

            mtype = msg.get_type()

            if mtype == "OPTICAL_FLOW_RAD":
                dist = getattr(msg, "distance", None)
                self._last_flow = FlowSample(
                    quality=int(msg.quality),
                    integrated_x=float(msg.integrated_x),
                    integrated_y=float(msg.integrated_y),
                    integration_time_us=int(msg.integration_time_us),
                    distance_m=float(dist) if dist is not None else None,
                )

            elif mtype == "DISTANCE_SENSOR":
                # MAVLink DISTANCE_SENSOR.current_distance is already in meters
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

            elif mtype == "HEARTBEAT":
                self._last_armed = (
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                ) != 0
                try:
                    mode = str(self.m.flightmode)
                except Exception:
                    mode = None

                if mode:
                    self._last_mode = mode
                else:
                    custom_mode = getattr(msg, "custom_mode", None)
                    if custom_mode is not None:
                        self._last_mode = f"custom:{int(custom_mode)}"

    def get_flow(self) -> Optional[FlowSample]:
        return self._last_flow

    def get_flow_quality(self) -> Optional[int]:
        return self._last_flow.quality if self._last_flow else None

    def get_distance_m(self) -> Optional[float]:
        return self._last_distance_m

    def get_local_position_ned(self) -> Optional[Dict[str, float]]:
        return self._last_local_pos

    def get_attitude(self) -> Optional[Dict[str, float]]:
        return self._last_attitude

    def get_is_armed(self) -> Optional[bool]:
        return self._last_armed

    def get_flight_mode(self) -> Optional[str]:
        return self._last_mode

    def get_body_frame_name(self) -> str:
        return self._body_frame_name
