import numpy as np

class PID:
    def __init__(self, kp, ki, kd, target):
        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)
        self.target = np.asarray(target, dtype=float)

        self.integral = np.zeros_like(self.target)
        self.prev_error = np.zeros_like(self.target)
        self._first = True
        self.vmax = None  # set to a float (m/s) if you want output clamp

    def reset(self, target=None, initial_measurement=None):
        if target is not None:
            self.target = np.asarray(target, dtype=float)
        self.integral[:] = 0.0
        if initial_measurement is not None:
            # Initialize prev_error to current error to avoid derivative kick
            m = np.asarray(initial_measurement, dtype=float)
            self.prev_error = self.target - m
            self._first = False
        else:
            self.prev_error[:] = 0.0
            self._first = True

    def update(self, measurement, dt):
        # Use a minimum dt to avoid huge derivative from tiny dt
        dt = float(dt)
        if dt < 1e-3:  # 1 ms minimum
            dt = 1e-3

        m = np.asarray(measurement, dtype=float)
        error = self.target - m

        # PI
        self.integral += error * dt

        # D with first-sample guard
        if self._first:
            derivative = np.zeros_like(error)
            self._first = False
        else:
            derivative = (error - self.prev_error) / dt

        self.prev_error = error

        out = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.vmax is not None:
            out = np.clip(out, -self.vmax, self.vmax)

        return out