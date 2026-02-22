from __future__ import annotations

import asyncio
import multiprocessing as mp

from app.vision_worker import run_vision_worker_process
from app.wiring import build_flight_controller


def main() -> None:
    vision_proc = mp.Process(target=run_vision_worker_process, name="vision-worker", daemon=False)
    vision_proc.start()

    try:
        asyncio.run(build_flight_controller().run())
    finally:
        if vision_proc.is_alive():
            vision_proc.terminate()
            vision_proc.join(timeout=3.0)
        if vision_proc.is_alive():
            vision_proc.kill()
            vision_proc.join(timeout=1.0)


if __name__ == "__main__":
    main()
