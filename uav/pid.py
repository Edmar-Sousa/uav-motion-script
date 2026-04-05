

class PIDController:
    def __init__(self, **kargs):
        self.xd = kargs['xd']

        self.kp = kargs['kp']
        self.kd = kargs['kd']
        self.ki = kargs['ki']

        self.integral = 0
        self.prev_error = 0
    

    def u(self, x, dt):
        if dt == 0:
            return 0
        
        error = self.xd - x

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return (error * self.kp + derivative * self.kd + self.integral * self.ki)

