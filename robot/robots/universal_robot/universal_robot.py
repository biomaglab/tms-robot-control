from time import sleep

from robot.robots.robot import Robot

from robot.robots.universal_robot.command_connection import (
    CommandConnection,
)

from robot.robots.universal_robot.state_connection import (
    StateConnection,
)


class UniversalRobot(Robot):
    """
    The class for communicating with Universal robot.
    """
    def __init__(self, ip):
        self.command_connection = CommandConnection(
            ip=ip,
        )
        self.state_connection = StateConnection(
            ip=ip,
        )

    # Connection
    def connect(self):
        success = self.command_connection.connect()
        success = success and self.state_connection.connect()

        return success

    def disconnect(self):
        success = self.command_connection.disconnect()
        success = success and self.state_connection.disconnect()
        
        return success

    def is_connected(self):
        return self.command_connection.connected and self.state_connection.connected

    # Initialization
    def initialize(self):
        pass

    # Robot state
    def get_pose(self):
        return self.state_connection.get_pose()

    def is_moving(self):
        return self.state_connection.is_moving()

    def is_error_state(self):
        return self.state_connection.is_error_state()

    def read_force_sensor(self):
        # Not implemented.
        pass

    # Movement
    def move_linear(self, target, speed):
        return self.connection.move_linear(target, a, v, t, r)

    def move_circular(self, start_position, waypoint, target, speed):
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
