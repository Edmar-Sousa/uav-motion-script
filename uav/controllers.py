import numpy as np


from .pid import PIDController
from .constants import K, L, B, G, Ixx, Iyy, Izz


class TrajectoryPlanner:
    def __init__(self):
        self.__position_tracker__ = [
            (20, 20, 20),
        ]

    def get_position_tracker(self):
        return self.__position_tracker__


class Controller:
    X, Y, Z = 0, 1, 2
    PHI, THETA, PSI = 3, 4, 5
    MASS = 6

    ''' x, y, z, phi, theta, psi, m '''
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
    ''''
        1. Precisa da posição x, y, z do UAV
        2. Precisa da posição x*, y*, z* desejada
        3. Calcula 𝜑*, 𝜃*, 𝜓* e T*
    '''

    def __init__(self, path_planner: TrajectoryPlanner):
        self.waypoint = path_planner.get_position_tracker()
        self.step = 0

        self.pid_x = PIDController()
        self.pid_z = PIDController()
        self.psi_pid = PIDController()
        self.theta_pid = PIDController()


    @property
    def step_point(self):
        return self.waypoint[self.step]


    def __psi__(self):
        uav_x = self.get_state_uav('x')
        uav_y = self.get_state_uav('y')

        dest_x = self.step_point[0]
        dest_y = self.step_point[1]

        return np.atan2(dest_y - uav_y, dest_x - uav_x)


    # def __tau_theta__(self, dt):
    #     theta_d = self.__theta__(dt)
    #     theta = self.get_state_uav('theta')

    #     result = self.theta_pid.u(theta, theta_d, dt)

    #     return Iyy * result


    def __tau_psi__(self, dt):
        psi_d = self.__psi__()
        psi = self.get_state_uav('psi')
        result = self.psi_pid.u(psi, psi_d, dt)

        return Izz * result
    

    def __theta__(self, dt):
        xd = self.step_point[0]
        x = self.get_state_uav('x')
        a = self.pid_x.u(x, xd, dt)

        # TODO: colocar validação de valor maximo do angulo
        return np.arctan(a / G)


    def __thrust__(self, dt):
        z_d = self.step_point[2]
        z = self.get_state_uav('z')
        mass = self.get_state_uav('mass')
        theta = self.get_state_uav('theta')
        phi = self.get_state_uav('phi')

        result = self.pid_z.u(z, z_d, dt)

        return (G + result) * np.cos(theta) * np.cos(phi) * mass


    def handler(self, dt):
        T = self.__thrust__(dt)
        tau_psi = self.__tau_psi__(dt)
        # tau_theta = self.__tau_theta__(dt)
        # theta = self.__theta__(dt)

        return { 
            'phi': 0, 
            'theta': 0, 
            'psi': 0, 
            'tau_psi': tau_psi,
            'tau_theta': 0,
            'T': T
        }



class MotorController(Controller): 
    '''
        1. Precisda de thrust e torques
        2. Gera a velocidade dos motores
    '''
    def __init__(self, position_controller: PositionController):
        self.pos_controller = position_controller

    def __motors_mixer__(self, T, tau_phi, tau_theta, tau_psi):
        
        w1 = max(0, T / (4 * K) - tau_theta / (2 * K * L) - tau_psi / (4 * B))
        w2 = max(0, T / (4 * K) - tau_phi   / (2 * K * L) + tau_psi / (4 * B))
        w3 = max(0, T / (4 * K) + tau_theta / (2 * K * L) - tau_psi / (4 * B))
        w4 = max(0, T / (4 * K) + tau_phi   / (2 * K * L) + tau_psi / (4 * B))

        return np.sqrt(w1), np.sqrt(w2), np.sqrt(w3), np.sqrt(w4)

    def handler(self, dt):
        pos = self.pos_controller.handler(dt)

        w1, w2, w3, w4 = self.__motors_mixer__(pos['T'], 0, pos['tau_theta'], pos['tau_psi'])

        return { 'w1': w1, 'w2': w2, 'w3': w3, 'w4': w4 }

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


