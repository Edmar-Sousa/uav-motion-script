

class PIDController:
    def __init__(self, kp = 1.0, ki = 0.0, kd = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.integral = 0
        self.prev_error = 0
    
    
    def reset(self):
        self.integral = 0
        self.prev_error = 0
    

    def u(self, x, xd, dt):        
        error = xd - x

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error

        return (error * self.kp + derivative * self.kd + self.integral * self.ki)

