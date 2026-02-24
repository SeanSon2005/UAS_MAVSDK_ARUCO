import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
for p in (ROOT, SRC):
    s = str(p)
    if s not in sys.path:
        sys.path.insert(0, s)

from comms.protocol import VisionSample  # noqa: E402
from missions import alignment, landing, takeoff  # noqa: E402


class _Log:
    def info(self, *args, **kwargs):
        return None

    def warning(self, *args, **kwargs):
        return None


class _MavFake:
    def __init__(self):
        self.last_velocity = None
        self.last_position = None
        self.mode_requests = []
        self.land_calls = 0
        self._armed = True

    def set_velocity_frd(self, vx, vy, vz):
        self.last_velocity = (vx, vy, vz)

    def set_position_ned(self, n, e, d):
        self.last_position = (n, e, d)

    def set_mode(self, mode_name):
        self.mode_requests.append(mode_name)
        return True

    def command_land(self):
        self.land_calls += 1

    def get_is_armed(self):
        return self._armed


class _Controller:
    def __init__(self):
        self.cfg = SimpleNamespace(
            takeoff=SimpleNamespace(alt_m=1.0, timeout_s=0.2, alt_tolerance_m=0.1, hold_s=0.0),
            alignment=SimpleNamespace(
                timeout_s=0.05,
                success_err_px=(20.0, 20.0),
                success_hold_s=0.0,
            ),
            ipc=SimpleNamespace(sample_ttl_s=0.15, drop_to_zero_after_s=0.30),
            runtime=SimpleNamespace(max_speed_mps=1.0),
            landing=SimpleNamespace(timeout_s=0.1),
        )
        self.logger = _Log()
        self.mav = _MavFake()
        self.phase = "INIT"
        self.current_cmd_vx = 0.0
        self.current_cmd_vy = 0.0
        self.current_cmd_vz = 0.0
        self.last_valid_vx = 0.0
        self.last_valid_vy = 0.0
        self.latest_vision_sample = None
        self._vision_age_s = None
        self._alt = 0.0
        self._alt_seq = [0.0, 0.4, 0.95, 1.0]
        self._land_polls = 0

    def poll_inputs(self):
        if self._alt_seq:
            self._alt = self._alt_seq.pop(0)
        self._land_polls += 1
        if self._land_polls > 2:
            self.mav._armed = False

    def get_altitude_m(self):
        return self._alt

    def publish_status(self):
        return None

    def sleep_to_next_tick(self, tick):
        return None

    def get_vision_age_s(self):
        return self._vision_age_s


class TestMissions(unittest.TestCase):
    def test_takeoff_completes_when_altitude_reached(self) -> None:
        c = _Controller()
        takeoff.run(c)
        self.assertEqual(c.phase, "TAKEOFF")
        self.assertEqual(c.mav.last_position, (0.0, 0.0, -1.0))

    def test_takeoff_timeout_raises(self) -> None:
        c = _Controller()
        c.cfg.takeoff.timeout_s = 0.01
        c._alt_seq = [0.0] * 100
        with self.assertRaises(TimeoutError):
            takeoff.run(c)

    def test_alignment_uses_fresh_vision_velocity(self) -> None:
        c = _Controller()
        c.latest_vision_sample = VisionSample(
            v=2,
            type="vision_sample",
            seq=1,
            t_mono_ns=1,
            has_target=True,
            err_x_px=5.0,
            err_y_px=5.0,
            contour_area=100.0,
            frame_w=640,
            frame_h=480,
            vx_cmd_f_mps=0.2,
            vy_cmd_r_mps=-0.3,
        )
        c._vision_age_s = 0.01
        alignment.run(c)
        self.assertIsNotNone(c.mav.last_velocity)
        self.assertAlmostEqual(c.mav.last_velocity[0], 0.2)
        self.assertAlmostEqual(c.mav.last_velocity[1], -0.3)

    def test_alignment_stale_drops_to_zero(self) -> None:
        c = _Controller()
        c.cfg.alignment.timeout_s = 0.01
        c.latest_vision_sample = VisionSample(
            v=2,
            type="vision_sample",
            seq=2,
            t_mono_ns=1,
            has_target=True,
            err_x_px=5.0,
            err_y_px=5.0,
            contour_area=100.0,
            frame_w=640,
            frame_h=480,
            vx_cmd_f_mps=0.5,
            vy_cmd_r_mps=0.5,
        )
        c._vision_age_s = 1.0
        alignment.run(c)
        self.assertEqual(c.current_cmd_vx, 0.0)
        self.assertEqual(c.current_cmd_vy, 0.0)

    def test_landing_requests_auto_land(self) -> None:
        c = _Controller()
        c._alt_seq = []
        landing.run(c)
        self.assertIn("AUTO.LAND", c.mav.mode_requests)


if __name__ == "__main__":
    unittest.main()
