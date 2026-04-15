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
            (5, 5, 20),
        ]

        self.step = 0

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
    
    def update(self, state, dt):
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


'''
class Controller:
    X, Y, Z = 0, 1, 2
    PHI, THETA, PSI = 3, 4, 5
    MASS = 6

    uav_state = [0, 0, 0, 0, 0, 0, 0]

    def handler(self, dt):
        raise Exception('Method not implemented')

    def set_state_uav(self, uav_state):
        Controller.uav_state = uav_state

    def get_state_uav(self, state):
        match state:
            case 'x': return Controller.uav_state[Controller.X]
            case 'y': return Controller.uav_state[Controller.Y]
            case 'z': return Controller.uav_state[Controller.Z]
            case 'phi': return Controller.uav_state[Controller.PHI]
            case 'theta': return Controller.uav_state[Controller.THETA]
            case 'psi': return Controller.uav_state[Controller.PSI]
            case 'mass': return Controller.uav_state[Controller.MASS]

class PositionController(Controller):
    def __init__(self, path_planner: TrajectoryPlanner):
        self.waypoint = path_planner.current
        self.step = 0

        self.pid_x = PIDController()
        self.pid_y = PIDController()
        self.pid_z = PIDController()

        self.phi_pid = PIDController()
        self.theta_pid = PIDController()
        self.psi_pid = PIDController()


    @property
    def step_point(self):
        return self.waypoint[self.step]


    def __psi__(self):
        uav_x = self.get_state_uav('x')
        uav_y = self.get_state_uav('y')

        dest_x = self.step_point[0]
        dest_y = self.step_point[1]

        return 0
        # return np.atan2(dest_y - uav_y, dest_x - uav_x)

    def __phi__(self, dt):
        yd = self.step_point[1]
        y = self.get_state_uav('y')
        a = self.pid_y.u(y, yd, dt)

        phi = np.arctan(-a / G)
        phi = np.clip(phi, np.deg2rad(-30), np.deg2rad(30))

        return phi

    def __theta__(self, dt):
        xd = self.step_point[0]
        x = self.get_state_uav('x')
        a = self.pid_x.u(x, xd, dt)

        theta = np.arctan(a / G)
        theta = np.clip(theta, np.deg2rad(-30), np.deg2rad(30))

        return theta

    def __tau_phi__(self, dt):
        phi_d = self.__phi__(dt)
        phi = self.get_state_uav('phi')
        result = self.phi_pid.u(phi, phi_d, dt)

        return Ixx * result

    def __tau_psi__(self, dt):
        psi_d = self.__psi__()
        psi = self.get_state_uav('psi')
        result = self.psi_pid.u(psi, psi_d, dt)

        return Izz * result

    def __tau_theta__(self, dt):
        theta_d = self.__theta__(dt)
        theta = self.get_state_uav('theta')
        result = self.theta_pid.u(theta, theta_d, dt)

        return Iyy * result


    def __thrust__(self, dt):
        z_d = self.step_point[2]
        z = self.get_state_uav('z')
        
        mass = self.get_state_uav('mass')
        theta = self.get_state_uav('theta')
        phi = self.get_state_uav('phi')

        result = self.pid_z.u(z, z_d, dt)

        return (G + result) * np.cos(theta) * np.cos(phi) * mass

    def __reset_all_pids__(self):
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()

        self.phi_pid.reset()
        self.theta_pid.reset()
        self.psi_pid.reset()


    def handler(self, dt):
        T = self.__thrust__(dt)

        tau_phi = self.__tau_phi__(dt)
        tau_theta = self.__tau_theta__(dt)
        tau_psi = self.__tau_psi__(dt)

        return { 
            'tau_phi': tau_phi,
            'tau_psi': tau_psi,
            'tau_theta': tau_theta,
            'T': T
        }


class ControllerExemplo:
    def __init__(self):
        self.kp = 0.5
        self.ki = 0.04
        self.kd = 0.6

        self.pid_z = PIDController(
            xd=20,
            kp=self.kp,
            kd=self.kd,
            ki=self.ki
        )

        self.pid_tau_phi = PIDController(
            xd=np.deg2rad(0),
            kp=self.kp,
            kd=self.kd,
            ki=self.ki
        )

        self.pid_tau_theta = PIDController(
            xd=np.deg2rad(0),
            kp=self.kp,
            kd=self.kd,
            ki=self.ki
        )

        self.pid_tau_psi = PIDController(
            xd=np.deg2rad(0),
            kp=self.kp,
            kd=self.kd,
            ki=self.ki
        )


    def __motors_mixer__(self, T, tau_phi, tau_theta, tau_psi):
        
        w1 = max(0, T / (4 * K) - tau_theta / (2 * K * L) - tau_psi / (4 * B))
        w2 = max(0, T / (4 * K) - tau_phi   / (2 * K * L) + tau_psi / (4 * B))
        w3 = max(0, T / (4 * K) + tau_theta / (2 * K * L) - tau_psi / (4 * B))
        w4 = max(0, T / (4 * K) + tau_phi   / (2 * K * L) + tau_psi / (4 * B))

        return np.sqrt(w1), np.sqrt(w2), np.sqrt(w3), np.sqrt(w4)


    def __tau_phi__(self, phi, dt):
        result = self.pid_tau_phi.u(phi, dt)
        return Ixx * result
    

    def __tau_theta__(self, theta, dt):
        result = self.pid_tau_theta.u(theta, dt)
        return Iyy * result
    

    def __tau_psi__(self, psi, dt):
        result = self.pid_tau_psi.u(psi, dt)
        return Izz * result


    def handler(self, pos, angular, dt):
        x, y, z = pos
        phi, theta, psi = angular

        T = self.__thrust__(z, phi, theta, dt)
        tau_phi   = self.__tau_phi__(phi, dt)
        tau_theta = self.__tau_theta__(theta, dt)
        tau_psi   = self.__tau_psi__(psi, dt)

        return self.__motors_mixer__(T, tau_phi, tau_theta, tau_psi)
'''