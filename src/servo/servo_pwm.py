from __future__ import annotations

from rpi_hardware_pwm import HardwarePWM
from .servo import Servo

class HardwareServo(Servo):
    def __init__(self, logger, pwm_chip=0, pwm_channel=2, steps_0_deg=2.8, steps_180_deg=11.6, angle_landing=105):
        self._logger = logger
        self._logger.info("[SERVO] Initializing on Chip %s, Channel %s...", pwm_chip, pwm_channel)
        try:
            self.servo = HardwarePWM(pwm_channel=pwm_channel, hz=50, chip=pwm_chip)
            self._steps_0 = float(steps_0_deg)
            self._steps_180 = float(steps_180_deg)
            self.servo.start(self.angle_to_steps(angle_landing))
        except Exception as exc:
            self._logger.error("[SERVO] FATAL: %s (Need sudo?)", exc)
            self.servo = None

    def angle_to_steps(self, angle: float) -> float:
        return ((angle / 180.0) * (self._steps_180 - self._steps_0)) + self._steps_0

    def move(self, angle: float, label: str):
        if not self.servo:
            return
        self._logger.info("[SERVO] Moving to %s (%s deg)", label, angle)
        angle = max(0.0, min(180.0, float(angle)))
        self.servo.change_duty_cycle(self.angle_to_steps(angle))

    def close(self):
        if self.servo:
            self.servo.stop()
