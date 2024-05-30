from time import sleep

from robot.robots.robot import Robot

from robot.robots.universal_robot.universal_robot_connection import (
    UniversalRobotConnection,
    MotionState,
)


class UniversalRobot(Robot):
    """
    The class for communicating with Universal robot.
    """
    # def __init__(self, ip, config):
    def __init__(self, ip):
        # self.robot_speed = config['robot_speed']
        self.connection = UniversalRobotConnection(
            ip=ip,
            # use_new_api=use_new_api,
            # v = config['vel']
            # a = config['acc']
        )

    # Connection
    def connect(self):
        return self.connection.connect()

    def disconnect(self):
        return self.connection.disconnect()

    def is_connected(self):
        return self.connection.connected

    # Initialization
    def initialize(self):
        # self.connection.set_speed_ratio(self.robot_speed)
        pass

    # Robot state
    def get_pose(self):
        return self.connection.get_pose()

    def is_moving(self):
        return self.connection.get_motion_state() == MotionState.IN_MOTION

    def is_error_state(self):
        return self.connection.get_motion_state() == MotionState.ERROR

    def read_force_sensor(self):
        return self.connection.read_force_sensor()

    # Movement
    def move_linear(self, linear_target, a, v, t, r):
        return self.connection.move_linear(linear_target, a, v, t, r)
    

    def move_circular(self, waypoint, target, a, v, r, mode):
        return self.connection.move_circular(waypoint, target, a, v, r, mode)

    def stop_robot(self):
        success = self.connection.stop_robot()

        # After the stop command, it takes some milliseconds for the robot to stop. Wait for that time.
        sleep(0.05)

        return success

    # Destruction and cleanup
    def close(self):
        self.stop_robot()
        self.disconnect()

    # Other

    # TODO: A dummy function, can be removed once the corresponding function from Dobot class is removed.
    def set_target_reached(self, _):
        pass
