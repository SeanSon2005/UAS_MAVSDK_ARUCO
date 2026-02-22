import sys
from pathlib import Path
import unittest

SRC = Path(__file__).resolve().parents[2] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from flight.flight_states.common import AlignPolicyInput, compute_xy_from_vision
from flight.state import StaleVisionMode, VisionState


class _PidFake:
    def reset(self, target=None, initial_measurement=None) -> None:
        return None

    def update(self, measurement, dt: float):
        ex, ey = measurement
        return (ex * 0.1, ey * 0.1)


class _Sample:
    def __init__(self, has_target=True, err_x_px=10.0, err_y_px=-5.0):
        self.has_target = has_target
        self.err_x_px = err_x_px
        self.err_y_px = err_y_px


class TestDomainPolicies(unittest.TestCase):
    def test_fresh_target_generates_cmd(self):
        state = VisionState()
        out = compute_xy_from_vision(
            AlignPolicyInput(
                sample=_Sample(),
                vision_age_s=0.01,
                dt=0.02,
                max_speed_mps=1.0,
                sample_ttl_s=0.15,
                drop_to_zero_after_s=0.3,
            ),
            state=state,
            pid=_PidFake(),
        )
        self.assertEqual(out.mode, StaleVisionMode.FRESH)
        self.assertTrue(out.target_visible)


if __name__ == "__main__":
    unittest.main()
