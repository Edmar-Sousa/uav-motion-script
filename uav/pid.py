

class PIDController:
    def __init__(self, **kargs):
        self.kp = 0.5
        self.ki = 0.04
        self.kd = 0.6

        self.integral = 0
        self.prev_error = 0
    

    def u(self, x, xd, dt):
        if dt == 0:
            return 0
        
        error = xd - x

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return (error * self.kp + derivative * self.kd + self.integral * self.ki)

