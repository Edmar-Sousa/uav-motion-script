import numpy as np
import matplotlib.pyplot as plt


from matplotlib.animation import FuncAnimation
from .utils import rotation_matrix


class DrawVant:
    def __init__(self, vant):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.vant = vant

        self.vant_center = np.array([0, 0, 0])
        self.vant_arms = np.array([
            [ 1,  0, 0],
            [-1,  0, 0],
            [ 0,  1, 0],
            [ 0, -1, 0]
        ])

        self.vant_arms_plot = []

        self.__create_arms_plot__()

    def __create_arms_plot__(self):
        for _ in self.vant_arms:
            arm, = self.ax.plot([], [], [], lw=2)
            self.vant_arms_plot.append(arm)


    def update(self, frame):
        x, y, z, phi, theta, psi, mass = self.vant.step(frame * 0.01)

        pos = np.array([x, y, z])
        R = rotation_matrix(phi, theta, psi)

        for i, arm in enumerate(self.vant_arms):
            arm_world = pos + R @ arm

            self.vant_arms_plot[i].set_data(
                [pos[0], arm_world[0]],
                [pos[1], arm_world[1]]
            )

            self.vant_arms_plot[i].set_3d_properties(
                [pos[2], arm_world[2]]
            )

        return self.vant_arms_plot

    def show(self):
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')

        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
        self.ax.set_zlim(0, 40.0)

        ani = FuncAnimation(fig=self.fig, func=self.update, frames=100, interval=100)
        # ani.save("uav-animation.mp4", writer="ffmpeg", fps=30)
        plt.show()
