#!/usr/bin/env python3

import asyncio

from constants import LOOP_HZ
from vision.controller import VisionController


async def run_test_aligner():
    vision = VisionController(preview_enabled=True)

    dt = 1.0 / LOOP_HZ
    loop = asyncio.get_running_loop()

    await vision.start()

    try:
        while vision.renderer.enabled:
            tick_start = loop.time()

            offset = await vision.get_offset(dt=dt)
            vision.render(offset, status_lines=["phase=TEST_ALIGN"])

            print(f"[TEST ALIGN] visible={offset.target_visible} vx={offset.vx:.2f} vy={offset.vy:.2f}")

            remaining = dt - (loop.time() - tick_start)
            if remaining > 0:
                await asyncio.sleep(remaining)
    finally:
        await vision.stop()


if __name__ == "__main__":
    asyncio.run(run_test_aligner())
