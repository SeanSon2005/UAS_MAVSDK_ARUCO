import importlib
import sys
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for p in (ROOT, SRC):
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)


def _load_mavlink_module(has_body_frd: bool):
    fake_pkg = types.ModuleType("pymavlink")
    fake_mavutil = types.ModuleType("pymavlink.mavutil")

    class _Const:
        MAV_FRAME_BODY_NED = 8

    if has_body_frd:
        _Const.MAV_FRAME_BODY_FRD = 12

    fake_mavutil.mavlink = _Const
    fake_mavutil.mavlink_connection = lambda *args, **kwargs: None
    fake_pkg.mavutil = fake_mavutil

    sys.modules["pymavlink"] = fake_pkg
    sys.modules["pymavlink.mavutil"] = fake_mavutil

    if "mavlink.mavlink" in sys.modules:
        del sys.modules["mavlink.mavlink"]

    return importlib.import_module("mavlink.mavlink")


class TestFrdFrame(unittest.TestCase):
    def test_body_frd_selected_when_available(self) -> None:
        mod = _load_mavlink_module(has_body_frd=True)
        mav = mod.MavLink("udp:127.0.0.1:14540")
        self.assertEqual(mav.get_body_frame_name(), "BODY_FRD")

    def test_body_ned_fallback_when_frd_missing(self) -> None:
        mod = _load_mavlink_module(has_body_frd=False)
        mav = mod.MavLink("udp:127.0.0.1:14540")
        self.assertEqual(mav.get_body_frame_name(), "BODY_NED")


if __name__ == "__main__":
    unittest.main()
