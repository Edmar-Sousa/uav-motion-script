import argparse

# from uav.trajectory import TrajectoryPlanner
from uav.controllers import UavState, TrajectoryPlanner, CascadeController, MotorController
from uav.vant import Vant
from uav.draw import DrawVant


def get_args_script():
    parser = argparse.ArgumentParser(
        prog='UAV motion',
        description='Script to simulate UAV motion to Food Delivery'
    )

    parser.add_argument('--osm')
    parser.add_argument('--number-uav', type=int)
    args = parser.parse_args()

    return args


def main():
    args = get_args_script()

    # trajectory_planner = TrajectoryPlanner(args.osm, args.number_uav)
    # trajectory_planner.generate()

    uav_state = UavState(x=0, y=0, z=0, phi=0, theta=0, psi=0, mass=1)
    trajectory_planner = TrajectoryPlanner()
    position_controller = CascadeController(trajectory_planner)
    motors_controller = MotorController()

    vant = Vant(
        uav_state,
        motor_controller=motors_controller,
        position_controller=position_controller,
    )
    draw_vant = DrawVant(vant)
    draw_vant.show()


if __name__ == '__main__':
    main()