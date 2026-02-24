from .servo import Servo

class DummyServo(Servo):
    def __init__(self, logger, pwm_chip=0, pwm_channel=2, steps_0_deg=2.8, steps_180_deg=11.6, angle_landing=105):
        pass
    
    def angle_to_steps(self, angle: float) -> float:
        pass

    def move(self, angle: float, label: str):
        pass
    
    def close(self):
        pass