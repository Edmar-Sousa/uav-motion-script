import numpy as np


class PIDController:
    def __init__(self, kp = 1.0, ki = 0.0, kd = 1.0, imax = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = imax

        self.integral = 0
        self.prev_error = 0
    
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0
    

    def u(self, x, xd, dt):
        error = xd - x

        self.integral += error * dt
        self.integral = np.clip(self.integral, -self.imax, self.imax)

        derivative = -(x - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = x

        return (error * self.kp + derivative * self.kd + self.integral * self.ki)

