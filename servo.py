from rpi_hardware_pwm import HardwarePWM

class ServoManager:
    def __init__(self, PWM_CHIP=0,
                 PWM_CHANNEL=2, 
                 STEPS_0_DEG=2.8, 
                 STEPS_180_DEG=11.6, 
                 ANGLE_LANDING=105):
        print(f"[SERVO] Initializing on Chip {PWM_CHIP}, Channel {PWM_CHANNEL}...")
        try:
            self.servo = HardwarePWM(pwm_channel=PWM_CHANNEL, hz=50, chip=PWM_CHIP)
            self.angle_to_steps = lambda a: ((a / 180) * (STEPS_180_DEG - STEPS_0_DEG)) + STEPS_0_DEG
            self.servo.start(self.angle_to_steps(ANGLE_LANDING)) 
        except Exception as e:
            print(f"[SERVO] FATAL: {e} (Need sudo?)")
            self.servo = None

    def move(self, angle, label):
        if not self.servo: return
        print(f"[SERVO] Moving to {label} ({angle}Â°)")
        angle = max(0, min(180, angle))
        self.servo.change_duty_cycle(self.angle_to_steps(angle))
        
    def close(self):
        if self.servo: self.servo.stop()