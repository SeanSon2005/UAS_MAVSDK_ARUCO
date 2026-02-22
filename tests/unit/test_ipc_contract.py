import sys
from pathlib import Path
import unittest

SRC = Path(__file__).resolve().parents[2] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from comms.protocol import ControlStatus, CONTROL_STATUS_TYPE, PROTO_VERSION, decode_control_status, encode_payload


class TestIpcContract(unittest.TestCase):
    def test_control_status_roundtrip(self):
        msg = ControlStatus(
            v=PROTO_VERSION,
            type=CONTROL_STATUS_TYPE,
            phase="ALIGN",
            vx=0.1,
            vy=-0.2,
            vz=0.0,
            alt_m=1.2,
            target_visible=True,
            t_mono_ns=1,
        )
        decoded = decode_control_status(encode_payload(msg))
        self.assertIsNotNone(decoded)
        self.assertEqual(decoded.phase, "ALIGN")


if __name__ == "__main__":
    unittest.main()
