import numpy as np


from dataclasses import dataclass

from .pid import PIDController
from .constants import K, L, B, G, Ixx, Iyy, Izz


@dataclass
class UavState:
    x: float
    y: float
    z: float

    phi: float
    theta: float
    psi: float
    mass: float


class TrajectoryPlanner:
    def __init__(self):
        self.__position_tracker__ = [
            (0, 0, 0),
            (10, 10, 10),
        ]

        self.step = 0

    def update_step(self):
        self.step = (self.step + 1) % len(self.__position_tracker__)

    @property
    def current(self):
        return self.__position_tracker__[self.step]


class CascadeController:
    def __init__(self, planner):
        self.planner = planner

        self.pid_x = PIDController(0.5, 0.04, 0.6)
        self.pid_y = PIDController(0.5, 0.04, 0.6)
        self.pid_z = PIDController(0.5, 0.04, 0.6)

        self.pid_phi = PIDController(0.5, 0.04, 0.6)
        self.pid_theta = PIDController(0.5, 0.04, 0.6)
        self.pid_psi = PIDController(0.5, 0.04, 0.6)

    def position_controller(self, state, dt):
        xd, yd, zd = self.planner.current
        x, y, z = state.x, state.y, state.z

        ax = self.pid_x.u(x, xd, dt)
        ay = self.pid_y.u(y, yd, dt)
        az = self.pid_z.u(z, zd, dt)

        return ax, ay, az
    
    def acceleration_to_attitude(self, ax, ay, psi):
        phi_d = (1 / G) * (ax * np.sin(psi) - ay * np.cos(psi))
        theta_d = (1 / G) * (ax * np.cos(psi) + ay * np.sin(psi))

        phi_d = np.clip(phi_d, np.deg2rad(-30), np.deg2rad(30))
        theta_d = np.clip(theta_d, np.deg2rad(-30), np.deg2rad(30))

        return phi_d, theta_d
    
    def attitude_controller(self, state, phi_d, theta_d, psi_d, dt):
        tau_phi = Ixx * self.pid_phi.u(state.phi, phi_d, dt)
        tau_theta = Iyy * self.pid_theta.u(state.theta, theta_d, dt)
        tau_psi = Izz * self.pid_psi.u(state.psi, psi_d, dt)

        return tau_phi, tau_theta, tau_psi
    
    def thrust_controller(self, state, az):
        return state.mass * (G + az)


    def __check_waypoint_reached__(self, state):
        xd, yd, zd = self.planner.current
        x, y, z = state.x, state.y, state.z

        distance = np.sqrt((xd - x) ** 2 + (yd - y) ** 2 + (zd - z) ** 2)

        return distance < 0.5
    

    def __check_waypoint_reached_and_update__(self, state):
        if self.__check_waypoint_reached__(state):
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()

            self.pid_phi.reset()
            self.pid_theta.reset()
            self.pid_psi.reset()

            self.planner.update_step()
    
    
    def update(self, state, dt):
        self.__check_waypoint_reached_and_update__(state)
        psi_d = 0

        ax, ay, az = self.position_controller(state, dt)
        phi_d, theta_d = self.acceleration_to_attitude(ax, ay, state.psi)

        tau_phi, tau_theta, tau_psi = self.attitude_controller(state, phi_d, theta_d, psi_d, dt)
        T = self.thrust_controller(state, az)
        
        return T, tau_phi, tau_theta, tau_psi


class MotorController: 
    def motors_mixer(self, T, tau_phi, tau_theta, tau_psi):

        w1 = max(0, T / (4 * K) - tau_theta/ (2 * K * L) - tau_psi / (4 * B))
        w2 = max(0, T / (4 * K) - tau_phi  / (2 * K * L) + tau_psi / (4 * B))
        w3 = max(0, T / (4 * K) + tau_theta/ (2 * K * L) - tau_psi / (4 * B))
        w4 = max(0, T / (4 * K) + tau_phi  / (2 * K * L) + tau_psi / (4 * B))

        return np.sqrt(w1), np.sqrt(w2), np.sqrt(w3), np.sqrt(w4)
