import sys
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
    decode_mavlink_status,
    decode_vision_sample,
    encode_payload,
)


class TestProtocol(unittest.TestCase):
    def test_vision_sample_roundtrip(self) -> None:
        msg = VisionSample(
            v=PROTO_VERSION,
            type=VISION_SAMPLE_TYPE,
            seq=10,
            t_mono_ns=123456,
            has_target=True,
            err_x_px=1.5,
            err_y_px=-2.5,
            contour_area=100.0,
            frame_w=640,
            frame_h=480,
            vx_cmd_f_mps=0.2,
            vy_cmd_r_mps=-0.1,
        )
        decoded = decode_vision_sample(encode_payload(msg))
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.seq, 10)
        self.assertEqual(decoded.vx_cmd_f_mps, 0.2)

    def test_mav_status_roundtrip(self) -> None:
        msg = MavlinkStatus(
            v=PROTO_VERSION,
            type=MAV_STATUS_TYPE,
            phase="ALIGNMENT",
            armed=True,
            offboard_active=True,
            mode="OFFBOARD",
            pos_n=1.0,
            pos_e=2.0,
            pos_d=-1.2,
            vel_n=0.1,
            vel_e=0.2,
            vel_d=0.0,
            alt_m=1.2,
            flow_quality=180,
            distance_m=0.7,
            last_vision_age_s=0.02,
            cmd_vx_f=0.2,
            cmd_vy_r=-0.3,
            cmd_vz_d=0.0,
            t_mono_ns=1,
        )
        decoded = decode_mavlink_status(encode_payload(msg))
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.phase, "ALIGNMENT")
        self.assertEqual(decoded.flow_quality, 180)


if __name__ == "__main__":
    unittest.main()
