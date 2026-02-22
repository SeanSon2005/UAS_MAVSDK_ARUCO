from __future__ import annotations

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from flight.state import VelocityCommand


class MavsdkLink:
    def __init__(self, drone: System, logger):
        self._drone = drone
        self._logger = logger

    async def connect_and_wait_ready(self, connection_string: str) -> None:
        self._logger.info("Connecting to %s", connection_string)
        await self._drone.connect(system_address=connection_string)

        async for state in self._drone.core.connection_state():
            if state.is_connected:
                self._logger.info("[MAVSDK] Connected")
                break

        self._logger.info("[MAVSDK] Waiting for sensor calibration")
        async for health in self._drone.telemetry.health():
            if (
                health.is_gyrometer_calibration_ok
                and health.is_accelerometer_calibration_ok
                and health.is_magnetometer_calibration_ok
            ):
                self._logger.info("[MAVSDK] Sensors OK")
                break

        self._logger.info("[MAVSDK] Waiting for local position")
        async for health in self._drone.telemetry.health():
            if health.is_local_position_ok:
                self._logger.info("[MAVSDK] Local position OK")
                break

    async def arm(self) -> None:
        await self._drone.action.arm()

    async def offboard_start(self) -> None:
        try:
            await self._drone.offboard.start()
        except OffboardError as exc:
            raise RuntimeError(f"Offboard start failed: {exc}") from exc

    async def offboard_stop(self) -> None:
        await self._drone.offboard.stop()

    async def set_velocity_body(self, cmd: VelocityCommand) -> None:
        await self._drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate)
        )

    async def land(self) -> None:
        await self._drone.action.land()
