from time import sleep

from robot.robots.elfin.elfin_connection import (
    ElfinConnection,
    MotionState,
)


class Elfin():
    """
    The class for communicating with Elfin robot.
    """
    def __init__(self, ip, config, use_new_api=False):
        self.robot_speed = config['robot_speed']
        self.connection = ElfinConnection(
            ip=ip,
            use_new_api=use_new_api,
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
        self.connection.set_speed_ratio(self.robot_speed)

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
    def move_linear(self, linear_target):
        return self.connection.move_linear(linear_target)

    def move_circular(self, start_position, waypoint, target):
        return self.connection.move_circular(start_position, waypoint, target)

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
