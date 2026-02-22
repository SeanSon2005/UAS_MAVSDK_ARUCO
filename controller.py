#!/usr/bin/env python3

import asyncio
import cv2

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, PositionNedYaw

from aruco import ArucoDetector
from camera import Camera
from servo import ServoManager
from constants import (
    CONNECTION_STRING,
    TAKEOFF_ALT_M, TAKEOFF_ALT_TOL_M, TAKEOFF_HOLD_TIME_S, TAKEOFF_TIMEOUT_S,
    ALIGNMENT_TIMEOUT_S,
    LOOP_HZ, VELOCITY_KP, MAX_SPEED_MPS, ENFORCE_LOCAL_POSITION,
    MARKER_ID, MARKER_SIZE_METERS, CALIB_MTX_FILE, CALIB_DIST_FILE,
    PWM_CHANNEL, PWM_CHIP, STEPS_0_DEG, STEPS_180_DEG, ANGLE_LANDING,
)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class FlightController:
    def __init__(self):
        self.drone = System()
        self.offboard_started = False
        self.armed = False
        self.marker_visible = False
        self.preview_enabled = True
        self.preview_window_name = "ArUco Detection"
        self.preview_window_open = False

        self.servo_mgr = ServoManager(
            PWM_CHIP=PWM_CHIP,
            PWM_CHANNEL=PWM_CHANNEL,
            STEPS_0_DEG=STEPS_0_DEG,
            STEPS_180_DEG=STEPS_180_DEG,
            ANGLE_LANDING=ANGLE_LANDING,
        )
        self.aruco_det = ArucoDetector(
            tag_id=MARKER_ID,
            marker_length_m=MARKER_SIZE_METERS,
            camera_matrix_file=CALIB_MTX_FILE,
            dist_coeffs_file=CALIB_DIST_FILE,
        )
        self.camera = Camera()

    def _init_preview_window(self):
        if not self.preview_enabled or self.preview_window_open:
            return

        try:
            cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
            self.preview_window_open = True
        except Exception as exc:
            self.preview_enabled = False
            self.preview_window_open = False
            print(f"[VISION] Preview disabled (window init failed): {exc}")

    def _close_preview_window(self):
        if not self.preview_window_open:
            return

        try:
            cv2.destroyWindow(self.preview_window_name)
        except Exception as exc:
            print(f"[VISION] Preview close warning: {exc}")
        finally:
            self.preview_window_open = False

    def _render_preview(self, frame, err_x, err_y, vx, vy, all_corners, all_ids, rvec, tvec):
        if not self.preview_enabled or frame is None:
            return

        if not self.preview_window_open:
            self._init_preview_window()
            if not self.preview_enabled:
                return

        preview = frame.copy()

        if all_corners is not None and len(all_corners) > 0:
            try:
                cv2.aruco.drawDetectedMarkers(preview, all_corners, all_ids)
            except Exception:
                pass

        if (
            rvec is not None
            and tvec is not None
            and self.aruco_det.camera_matrix is not None
            and self.aruco_det.dist_coeffs is not None
        ):
            try:
                cv2.drawFrameAxes(
                    preview,
                    self.aruco_det.camera_matrix,
                    self.aruco_det.dist_coeffs,
                    rvec,
                    tvec,
                    self.aruco_det.marker_length_m * 0.5,
                )
            except Exception:
                pass

        status = "visible" if self.marker_visible else "not_visible"
        ex = "None" if err_x is None else f"{err_x:.3f}"
        ey = "None" if err_y is None else f"{err_y:.3f}"
        overlay = f"{status} err_x={ex} err_y={ey} vx={vx:.2f} vy={vy:.2f}"
        cv2.putText(
            preview,
            overlay,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0) if self.marker_visible else (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

        try:
            cv2.imshow(self.preview_window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.preview_enabled = False
                self._close_preview_window()
                print("[VISION] Preview closed by user")
        except Exception as exc:
            self.preview_enabled = False
            self._close_preview_window()
            print(f"[VISION] Preview disabled (render failed): {exc}")

    async def connect_and_wait_ready(self):
        print(f"[MAVSDK] Connecting to {CONNECTION_STRING}")
        await self.drone.connect(system_address=CONNECTION_STRING)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[MAVSDK] Connected")
                break

        print("[MAVSDK] Waiting for sensor calibration")
        async for health in self.drone.telemetry.health():
            if (
                health.is_gyrometer_calibration_ok
                and health.is_accelerometer_calibration_ok
                and health.is_magnetometer_calibration_ok
            ):
                print("[MAVSDK] Sensors OK")
                break

        if ENFORCE_LOCAL_POSITION:
            print("[MAVSDK] Waiting for local position")
            async for health in self.drone.telemetry.health():
                if health.is_local_position_ok:
                    print("[MAVSDK] Local position OK")
                    break

    async def set_velocity_body(self, vx: float, vy: float, vz: float = 0.0, yaw_rate: float = 0.0):
        await self.drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(vx, vy, vz, yaw_rate)
        )

    async def arm_and_start_offboard(self):
        print("[FLIGHT] Arming")
        await self.drone.action.arm()
        self.armed = True

        await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        try:
            # Set offboard position to zero
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
            # Start offboard
            await self.drone.offboard.start()
            self.offboard_started = True
            print("[FLIGHT] Offboard started")
        except OffboardError as exc:
            raise RuntimeError(f"Offboard start failed: {exc}") from exc

    async def takeoff_phase(self):
        print(f"[FLIGHT] Taking off to ~{TAKEOFF_ALT_M:.1f}m")
        self.servo_mgr.move(ANGLE_LANDING, "DOWN")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -1 * TAKEOFF_ALT_M, 0.0),
        )

        dt = 1.0 / LOOP_HZ
        hold_accum_s = 0.0
        loop = asyncio.get_running_loop()
        start_time = loop.time()

        async for pos_vel in self.drone.telemetry.position_velocity_ned():
            alt_m = -pos_vel.position.down_m
            err_m = TAKEOFF_ALT_M - alt_m

            print(f"[TAKEOFF] alt={alt_m:.2f}m err={err_m:.2f}m")

            # Ensures altitude held within tolerance for required time.
            if abs(err_m) <= TAKEOFF_ALT_TOL_M:
                hold_accum_s += dt
                if hold_accum_s >= TAKEOFF_HOLD_TIME_S:
                    print("[TAKEOFF] Altitude reached")
                    break
            else:
                hold_accum_s = 0.0

            if loop.time() - start_time > TAKEOFF_TIMEOUT_S:
                raise RuntimeError("[TAKEOFF] timeout: altitude target not reached")

            await asyncio.sleep(dt)

    async def alignment_phase(self):
        print("[FLIGHT] Alignment phase started")
        dt = 1.0 / LOOP_HZ
        loop = asyncio.get_running_loop()
        end_time = loop.time() + ALIGNMENT_TIMEOUT_S

        while loop.time() < end_time:
            tick_start = loop.time()
            vx = 0.0
            vy = 0.0

            frame = await self.camera.capture_frame()
            if frame is not None:
                (
                    err_x,
                    err_y,
                    _,
                    _,
                    all_corners,
                    all_ids,
                    rvec,
                    tvec,
                ) = await asyncio.to_thread(self.aruco_det.detect_with_debug, frame)

                if err_x is not None:
                    self.marker_visible = True
                    vx = clamp(-err_y * VELOCITY_KP, -MAX_SPEED_MPS, MAX_SPEED_MPS)
                    vy = clamp(err_x * VELOCITY_KP, -MAX_SPEED_MPS, MAX_SPEED_MPS)
                else:
                    self.marker_visible = False

                self._render_preview(frame, err_x, err_y, vx, vy, all_corners, all_ids, rvec, tvec)
            else:
                self.marker_visible = False

            await self.set_velocity_body(vx, vy, 0.0, 0.0)
            print(f"[ALIGN] visible={self.marker_visible} vx={vx:.2f} vy={vy:.2f}")

            remaining = dt - (loop.time() - tick_start)
            if remaining > 0:
                await asyncio.sleep(remaining)

    async def land_and_shutdown(self):
        try:
            if self.offboard_started:
                await self.set_velocity_body(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass

        if self.offboard_started:
            try:
                print("[FLIGHT] Stopping offboard")
                await self.drone.offboard.stop()
            except Exception as exc:
                print(f"[FLIGHT] Offboard stop failed: {exc}")
            finally:
                self.offboard_started = False

        if self.armed:
            try:
                print("[FLIGHT] Landing")
                await self.drone.action.land()
            except Exception as exc:
                print(f"[FLIGHT] Landing command failed: {exc}")

        self._close_preview_window()
        self.servo_mgr.close()
        await self.camera.stop_camera()

    async def run(self):
        await self.camera.start_camera()
        self._init_preview_window()
        try:
            await self.connect_and_wait_ready()
            await self.arm_and_start_offboard()
            await self.takeoff_phase()
            await self.alignment_phase()
        except KeyboardInterrupt:
            print("[SYSTEM] Keyboard interrupt")
        except Exception as exc:
            print(f"[SYSTEM] Error: {exc}")
        finally:
            await self.land_and_shutdown()
            self._close_preview_window()
