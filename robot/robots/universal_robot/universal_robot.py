from time import sleep

from robot.robots.robot import Robot

from robot.robots.universal_robot.universal_robot_connection import (
    UniversalRobotConnection,
)


class UniversalRobot(Robot):
    """
    The class for communicating with Universal robot.
    """
    def __init__(self, ip):
        self.connection = UniversalRobotConnection(
            ip=ip,
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
        pass

    # Robot state
    def get_pose(self):
        pass

    def is_moving(self):
        pass

    def is_error_state(self):
        pass

    def read_force_sensor(self):
        pass

    # Movement
    def move_linear(self, linear_target, a, v, t, r):
        return self.connection.move_linear(linear_target, a, v, t, r)

    def move_circular(self, waypoint, target, a, v, r, mode):
        return self.connection.move_circular(waypoint, target, a, v, r, mode)

    def stop_robot(self):
        return self.connection.stop_robot()

    # Destruction and cleanup
    def close(self):
        self.stop_robot()
        self.disconnect()

    # Other

    # TODO: A dummy function, can be removed once the corresponding function from Dobot class is removed.
    def set_target_reached(self, _):
        pass
