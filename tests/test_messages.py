import sys
from pathlib import Path
import unittest

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from comms.protocol import (
    PROTO_VERSION,
    VISION_SAMPLE_TYPE,
    VisionSample,
    decode_vision_sample,
    encode_payload,
)


class TestMessages(unittest.TestCase):
    def test_vision_sample_roundtrip(self):
        sample = VisionSample(
            v=PROTO_VERSION,
            type=VISION_SAMPLE_TYPE,
            seq=10,
            t_mono_ns=123456789,
            has_target=True,
            err_x_px=1.5,
            err_y_px=-2.0,
            contour_area=120.0,
            frame_w=640,
            frame_h=480,
        )
        decoded = decode_vision_sample(encode_payload(sample))
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.seq, 10)
        self.assertTrue(decoded.has_target)

    def test_vision_sample_wrong_type(self):
        raw = b'{"v":1,"type":"wrong","seq":1,"t_mono_ns":1,"has_target":false,"err_x_px":null,"err_y_px":null,"contour_area":0,"frame_w":640,"frame_h":480}'
        self.assertIsNone(decode_vision_sample(raw))

    def test_vision_sample_missing_field(self):
        raw = b'{"v":1,"type":"vision_sample","seq":1}'
        self.assertIsNone(decode_vision_sample(raw))


if __name__ == "__main__":
    unittest.main()
