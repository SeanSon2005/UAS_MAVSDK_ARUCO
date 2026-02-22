from __future__ import annotations

import asyncio

from app.wiring import build_vision_controller


def run_vision_worker_process() -> None:
    try:
        asyncio.run(build_vision_controller().run())
    except KeyboardInterrupt:
        pass
