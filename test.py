#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path
from typing import Any


REPO_ROOT = Path(__file__).resolve().parent
SRC_ROOT = REPO_ROOT / "src"

for p in (str(REPO_ROOT), str(SRC_ROOT)):
    if p not in sys.path:
        sys.path.insert(0, p)


from mavlink.mavlink import MavLink

TAKEOFF_ALT_M = 1.5     # meters above home
HOLD_SEC = 3.0
SETPOINT_HZ = 20
CONNECTION = "udp:127.0.0.1:14540"   # try 14550 if needed


def wait_for_local_position(ml, timeout_s: float = 15.0) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        ml.poll(0.2)
        lp = ml.get_local_position_ned()
        if lp is not None:
            return
    raise RuntimeError("No LOCAL_POSITION_NED received (check message interval requests / port).")


def hold_position_ned(ml, n: float, e: float, d: float, dur_s: float) -> None:
    dt = 1.0 / SETPOINT_HZ
    t0 = time.time()
    while time.time() - t0 < dur_s:
        ml.set_position_ned(n, e, d, yaw_rad=None)
        ml.poll(0.05)

        lp = ml.get_local_position_ned()
        q = ml.get_flow_quality()
        dist = ml.get_distance_m()
        if lp:
            # NED: down is +, so altitude above home is -z (approximately in sim)
            alt = -lp["z"]
            print(f"X={lp['x']}, m, ALT={alt:5.2f}m  flowQ={q}  range={dist}  mode={ml.get_flight_mode()}  armed={ml.get_is_armed()}")
        time.sleep(dt)


def main():
    print(f"Connecting: {CONNECTION}")
    ml = MavLink(CONNECTION)
    ml.connect()
    print("Connected. body_frame =", ml.get_body_frame_name())

    # Ask PX4 to stream what we need
    ml.start_streams(hz=20.0)
    print("Requested streams (attitude/local_pos/distance/flow). Waiting for local position...")

    wait_for_local_position(ml, timeout_s=15.0)
    print("LOCAL_POSITION_NED OK")

    # Arm (Make sure arming checks won't block you. In SITL you may need COM_ARM_CHK=0.)
    print("Arming...")
    if not ml.arm(timeout_s=8.0):
        raise RuntimeError("Arming failed (check PX4 preflight failures; e.g. COM_ARM_CHK).")
    print("Armed!")

    # Prime offboard and switch mode inside prime_offboard()
    print("Priming OFFBOARD...")
    ml.prime_offboard(duration_s=1.5, hz=SETPOINT_HZ)
    time.sleep(0.2)

    # Takeoff: In NED, up is negative down.
    target_down = -TAKEOFF_ALT_M
    print(f"Taking off to {TAKEOFF_ALT_M:.2f}m (down={target_down:.2f})")
    hold_position_ned(ml, 0.0, 0.0, target_down, dur_s=6.0)

    print(f"Holding for {HOLD_SEC:.1f}s")
    hold_position_ned(ml, 0.0, 0.0, target_down, dur_s=HOLD_SEC)

    # Land
    print("Landing...")
    ml.command_land()

    # Keep polling while landing for a few seconds
    t0 = time.time()
    while time.time() - t0 < 8.0:
        ml.poll(0.2)
        lp = ml.get_local_position_ned()
        q = ml.get_flow_quality()
        dist = ml.get_distance_m()
        if lp:
            alt = -lp["z"]
            print(f"ALT={alt:5.2f}m  flowQ={q}  range={dist}  mode={ml.get_flight_mode()}  armed={ml.get_is_armed()}")
        time.sleep(0.2)

    print("Disarming...")
    ml.disarm(timeout_s=8.0)
    print("Done.")


if __name__ == "__main__":
    main()