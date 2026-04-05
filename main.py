
from uav.controllers import TrajectoryPlanner, PositionController, AltitudeController, MotorController
from uav.vant import Vant
from uav.draw import DrawVant



def main():
    osm_file_path = './map.osm'

    

    trajectory_planner = TrajectoryPlanner()

    position_controller = PositionController(trajectory_planner)
    altitude_controller = AltitudeController(position_controller)
    motor_controller = MotorController(position_controller, altitude_controller)
    
    vant = Vant(
        motor_controller=motor_controller,
        position_controller=position_controller,
        altitude_controller=altitude_controller
    )
    draw_vant = DrawVant(vant)
    
    draw_vant.show()


if __name__ == '__main__':
    main()
