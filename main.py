from __future__ import annotations

import multiprocessing as mp
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent
SRC_ROOT = REPO_ROOT / "src"

for p in (str(REPO_ROOT), str(SRC_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)

from processes.mavlink import run_mavlink_process
from processes.vision import run_vision_process


def main() -> None:
    vision_proc = mp.Process(target=run_vision_process, name="vision-process", daemon=False)
    vision_proc.start()

    exit_code = 0
    try:
        run_mavlink_process()
    except KeyboardInterrupt:
        exit_code = 130
    except Exception:
        exit_code = 1
        raise
    finally:
        if vision_proc.is_alive():
            vision_proc.terminate()
            vision_proc.join(timeout=3.0)
        if vision_proc.is_alive():
            vision_proc.kill()
            vision_proc.join(timeout=1.0)

    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
