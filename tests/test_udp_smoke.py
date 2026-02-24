import socket
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for p in (ROOT, SRC):
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)

from comms.protocol import (  # noqa: E402
    MAV_STATUS_TYPE,
    PROTO_VERSION,
    VISION_SAMPLE_TYPE,
    MavlinkStatus,
    VisionSample,
)
from comms.udp import (  # noqa: E402
    MavStatusReceiver,
    MavStatusUdpSender,
    VisionSampleReceiver,
    VisionSampleUdpSender,
)


def _free_port() -> int:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    except PermissionError as exc:
        raise unittest.SkipTest(f"socket operations not permitted in this environment: {exc}")
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class TestUdpSmoke(unittest.TestCase):
    def test_vision_udp_loopback(self) -> None:
        port = _free_port()
        rx = VisionSampleReceiver("127.0.0.1", port)
        tx = VisionSampleUdpSender("127.0.0.1", port)

        sample = VisionSample(
            v=PROTO_VERSION,
            type=VISION_SAMPLE_TYPE,
            seq=1,
            t_mono_ns=1,
            has_target=True,
            err_x_px=1.0,
            err_y_px=2.0,
            contour_area=3.0,
            frame_w=640,
            frame_h=480,
            vx_cmd_f_mps=0.1,
            vy_cmd_r_mps=0.2,
        )
        tx.send(sample)

        got = None
        for _ in range(50):
            got = rx.poll()
            if got is not None:
                break
            time.sleep(0.005)

        tx.close()
        rx.close()
        self.assertIsNotNone(got)
        self.assertEqual(got.seq, 1)

    def test_status_udp_loopback(self) -> None:
        port = _free_port()
        rx = MavStatusReceiver("127.0.0.1", port)
        tx = MavStatusUdpSender("127.0.0.1", port)

        status = MavlinkStatus(
            v=PROTO_VERSION,
            type=MAV_STATUS_TYPE,
            phase="TAKEOFF",
            armed=True,
            offboard_active=True,
            mode="OFFBOARD",
            pos_n=0.0,
            pos_e=0.0,
            pos_d=0.0,
            vel_n=0.0,
            vel_e=0.0,
            vel_d=0.0,
            alt_m=0.0,
            flow_quality=0,
            distance_m=0.0,
            last_vision_age_s=None,
            cmd_vx_f=0.0,
            cmd_vy_r=0.0,
            cmd_vz_d=0.0,
            t_mono_ns=1,
        )
        tx.send(status)

        got = None
        for _ in range(50):
            got = rx.poll()
            if got is not None:
                break
            time.sleep(0.005)

        tx.close()
        rx.close()
        self.assertIsNotNone(got)
        self.assertEqual(got.phase, "TAKEOFF")


if __name__ == "__main__":
    unittest.main()
