from __future__ import annotations

import asyncio

from mavsdk import System


class TelemetryCache:
    def __init__(self, drone: System, logger):
        self._drone = drone
        self._logger = logger
        self._task: asyncio.Task | None = None
        self._altitude_m = 0.0

    async def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="telemetry-cache")

    async def stop(self) -> None:
        if self._task is None:
            return
        self._task.cancel()
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        finally:
            self._task = None

    async def _run(self) -> None:
        try:
            async for pos_vel in self._drone.telemetry.position_velocity_ned():
                alt_m = float(-pos_vel.position.down_m)
                if alt_m == alt_m and alt_m not in (float("inf"), float("-inf")):
                    self._altitude_m = alt_m
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self._logger.error("[TELEM] Altitude stream stopped: %s", exc)

    def get_altitude(self) -> float:
        return self._altitude_m
