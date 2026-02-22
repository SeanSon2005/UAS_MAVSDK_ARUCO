import sys
from pathlib import Path
import unittest

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from flight.flight_states.common import (
    LandingPolicyInput,
    compute_landing_command,
    stale_mode_from_age,
    should_allow_descent_without_fresh_vision,
)
from flight.state import StaleVisionMode, VelocityCommand


class TestPolicies(unittest.TestCase):
    def test_stale_mode(self):
        self.assertEqual(stale_mode_from_age(None, 0.15, 0.30), StaleVisionMode.ZERO)
        self.assertEqual(stale_mode_from_age(0.05, 0.15, 0.30), StaleVisionMode.FRESH)
        self.assertEqual(stale_mode_from_age(0.20, 0.15, 0.30), StaleVisionMode.HOLD)
        self.assertEqual(stale_mode_from_age(0.50, 0.15, 0.30), StaleVisionMode.ZERO)

    def test_landing_gate_allow(self):
        self.assertTrue(
            should_allow_descent_without_fresh_vision(
                current_alt_m=0.75,
                since_valid_s=0.5,
                landing_switch_alt_m=0.25,
                no_vision_descent_max_s=2.0,
            )
        )

    def test_landing_gate_block(self):
        self.assertFalse(
            should_allow_descent_without_fresh_vision(
                current_alt_m=0.20,
                since_valid_s=0.5,
                landing_switch_alt_m=0.25,
                no_vision_descent_max_s=2.0,
            )
        )
        self.assertFalse(
            should_allow_descent_without_fresh_vision(
                current_alt_m=0.75,
                since_valid_s=None,
                landing_switch_alt_m=0.25,
                no_vision_descent_max_s=2.0,
            )
        )

    def test_landing_cmd_switch(self):
        out = compute_landing_command(
            LandingPolicyInput(
                current_alt_m=0.2,
                since_valid_s=0.3,
                landing_switch_alt_m=0.25,
                no_vision_descent_max_s=2.0,
                landing_vel_mps=0.3,
                mode=StaleVisionMode.FRESH,
                xy_cmd=VelocityCommand(vx=0.2, vy=0.1, vz=0.0, yaw_rate=0.0),
            )
        )
        self.assertTrue(out.should_switch_to_land_command)
        self.assertEqual(out.cmd.vz, 0.0)


if __name__ == "__main__":
    unittest.main()
