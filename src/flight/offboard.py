from __future__ import annotations

import asyncio
import math

from flight.state import VelocityCommand


class OffboardPublisher:
    def __init__(self, loop_hz: float, send_velocity_body, get_desired_cmd, get_vision_age_s, logger):
        self._period_s = 1.0 / float(loop_hz)
        self._send_velocity_body = send_velocity_body
        self._get_desired_cmd = get_desired_cmd
        self._get_vision_age_s = get_vision_age_s
        self._logger = logger
        self._task: asyncio.Task | None = None

    async def start(self) -> None:
        if self._task is None or self._task.done():
            self._task = asyncio.create_task(self._run(), name="offboard-publisher")

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
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        prev_tick = loop.time()
        late_ticks = 0
        max_jitter_ms = 0.0
        sum_jitter_ms = 0.0
        n_jitter = 0
        next_log = loop.time() + 0.5

        while True:
            now = loop.time()
            dt_s = now - prev_tick
            prev_tick = now

            jitter_ms = abs(dt_s - self._period_s) * 1000.0
            sum_jitter_ms += jitter_ms
            n_jitter += 1
            max_jitter_ms = max(max_jitter_ms, jitter_ms)

            if now - next_tick > self._period_s:
                late_ticks += 1

            cmd: VelocityCommand = self._get_desired_cmd()
            try:
                await self._send_velocity_body(cmd)
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self._logger.error("[OFFBOARD] Publish failed: %s", exc)

            if now >= next_log:
                avg_jitter_ms = (sum_jitter_ms / n_jitter) if n_jitter else math.nan
                vision_age = self._get_vision_age_s()
                vision_age_text = "none" if vision_age is None else f"{vision_age:.3f}s"
                self._logger.info(
                    "[OFFBOARD] period=%.1fms avg_jitter=%.2fms max_jitter=%.2fms late=%d vision_age=%s",
                    self._period_s * 1000.0,
                    avg_jitter_ms,
                    max_jitter_ms,
                    late_ticks,
                    vision_age_text,
                )
                late_ticks = 0
                max_jitter_ms = 0.0
                sum_jitter_ms = 0.0
                n_jitter = 0
                next_log = now + 0.5

            next_tick += self._period_s
            sleep_for = next_tick - loop.time()
            if sleep_for > 0:
                await asyncio.sleep(sleep_for)
            else:
                next_tick = loop.time()
                await asyncio.sleep(0)
