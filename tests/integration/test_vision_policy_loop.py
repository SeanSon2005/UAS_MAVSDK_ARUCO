import sys
from pathlib import Path
import unittest

SRC = Path(__file__).resolve().parents[2] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from flight.flight_states.common import LandingPolicyInput, compute_landing_command
from flight.state import StaleVisionMode, VelocityCommand


class TestVisionPolicyLoop(unittest.TestCase):
    def test_hold_mode_blocks_descent_after_window(self):
        out = compute_landing_command(
            LandingPolicyInput(
                current_alt_m=1.0,
                since_valid_s=3.0,
                landing_switch_alt_m=0.25,
                no_vision_descent_max_s=2.0,
                landing_vel_mps=0.3,
                mode=StaleVisionMode.HOLD,
                xy_cmd=VelocityCommand(vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0),
            )
        )
        self.assertEqual(out.cmd.vz, 0.0)


if __name__ == "__main__":
    unittest.main()
