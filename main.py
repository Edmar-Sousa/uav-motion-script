import argparse

from uav.trajectory import TrajectoryPlanner
from uav.controllers import PositionController, AltitudeController, MotorController
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

    trajectory_planner = TrajectoryPlanner(args.osm, args.number_uav)
    trajectory_planner.generate()

    # position_controller = PositionController(trajectory_planner)
    # altitude_controller = AltitudeController(position_controller)
    # motor_controller = MotorController(position_controller, altitude_controller)

    # vant = Vant(
    #     motor_controller=motor_controller,
    #     position_controller=position_controller,
    #     altitude_controller=altitude_controller
    # )
    # draw_vant = DrawVant(vant)

    # draw_vant.show()


if __name__ == '__main__':
    main()