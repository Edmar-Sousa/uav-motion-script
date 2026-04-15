import numpy as np

from .constants import Ixx, Iyy, Izz, Ax, Ay, Az, G, K, B, L
from .utils import rotation_matrix


class Vant:
    def __init__(self, state, **kargs):
        self.m = state.mass
        self.state = state

        self.motor_controller = kargs.get('motor_controller')
        self.position_controller = kargs.get('position_controller')

        # posição 
        self.x, self.y, self.z = state.x, state.y, state.z
        self.phi, self.theta, self.psi = state.phi, state.theta, state.psi

        # velocidades
        self.p, self.q, self.r = 0, 0, 0
        self.vx, self.vy, self.vz = 0, 0, 0

        # rotores
        self.w1, self.w2, self.w3, self.w4 = 0, 0, 0, 0

        
        self.G = np.array([0, 0, -G]).reshape(3, 1)
        self.I = np.diag([Ixx, Iyy, Izz])
        self.drag = np.diag([Ax, Ay, Az])

    @property
    def linear_position(self):
        return np.array([self.x, self.y, self.z]).reshape(3, 1)


    @property
    def angular_position(self):
        return np.array([self.phi, self.theta, self.psi]).reshape(3, 1)
    
    
    @property
    def linear_velocity(self):
        return np.array([self.vx, self.vy, self.vz]).reshape(3, 1)

    
    @property
    def angular_velocity(self):
        return np.array([self.p, self.q, self.r]).reshape(3, 1)


    def __update_state__(self):
        x, y, z = self.linear_position.flatten()
        phi, theta, psi = self.angular_position.flatten()

        self.state.x = x
        self.state.y = y
        self.state.z = z

        self.state.phi = phi
        self.state.theta = theta
        self.state.psi = psi

        self.state.mass = self.m
    

    def __thrust__(self):
        Wi = self.w1 ** 2 + self.w2 ** 2 + self.w3 ** 2 + self.w4 ** 2
        return np.array([0, 0, K * Wi]).reshape(3, 1)


    def __torque__(self):
        Wi = -self.w1 ** 2 + self.w2 ** 2 - self.w3 ** 2 + self.w4 ** 2

        tau_b = np.array([
            L * K * (self.w4 ** 2 - self.w2 ** 2),
            L * K * (self.w3 ** 2 - self.w1 ** 2),
            B * Wi
        ])

        return tau_b.reshape(3, 1)


    def __fd__(self):
        return self.drag @ self.linear_velocity


    def acceleration(self):
        R = rotation_matrix(self.phi, self.theta, self.psi)
        T = R @ self.__thrust__()
        fd = self.__fd__()

        return self.G + 1 / self.m * T - 1 / self.m * fd
    

    def angular_acceleration(self):
        tau = self.__torque__()
        omega = self.angular_velocity

        cross = np.cross(omega.reshape(3,), (self.I @ omega).reshape(3,)).reshape(3, 1)
        return np.linalg.inv(self.I) @ (tau - cross)


    def step(self, dt):
        T, tau_phi, tau_theta, tau_psi = self.position_controller.update(self.state, dt)
        self.w1, self.w2, self.w3, self.w4 = self.motor_controller.motors_mixer(T, tau_phi, tau_theta, tau_psi)

        angular = self.angular_acceleration()

        self.p = angular[0, 0]
        self.q = angular[1, 0]
        self.r = angular[2, 0]

        w = self.angular_position + self.angular_velocity * dt

        self.phi = w[0, 0]
        self.theta = w[1, 0]
        self.psi = w[2, 0]

        acceleration = self.acceleration()

        v = self.linear_velocity + acceleration * dt

        self.vx = v[0, 0]
        self.vy = v[1, 0]
        self.vz = v[2, 0]

        s = self.linear_position + self.linear_velocity * dt

        self.x = s[0, 0]
        self.y = s[1, 0]
        self.z = s[2, 0]

        self.__update_state__()

        return [
            self.x, self.y, self.z,
            self.phi, self.theta, self.psi,
            self.m
        ]

