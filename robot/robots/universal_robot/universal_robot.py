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
    # The typical maximum velocity for Universal Robot is 1 m/s, as described here:
    #
    # https://www.universal-robots.com/media/1827367/05_2023_collective_data-sheet.pdf
    #
    # Velocity is defined in m/s, so the maximum velocity is 1.0.
    MAX_VELOCITY = 1.0

    # Movement commands for Universal Robot include acceleration, velocity, and time.
    # The movement can be defined either by acceleration and velocity, or by time. In our use,
    # it is more convenient to use acceleration and velocity, as we want to ensure that the
    # velocity won't be too high. However, when velocity is defined, we also need to define
    # acceleration, so that the robot can reach the desired velocity in a controlled manner.
    #
    # Use a default acceleration value (in m/s^2), defined here.
    DEFAULT_ACCELERATION = 0.2

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
        success = success and self.state_connection.connect_and_start()

        return success

    def disconnect(self):
        success = self.command_connection.disconnect()
        success = success and self.state_connection.disconnect_and_stop()

        return success

    def is_connected(self):
        return self.command_connection.connected and self.state_connection.connected

    # Initialization
    def initialize(self):
        # Wait until the robot state has been received.
        while not self.state_connection.is_state_received():
            print("Waiting for the robot state...")
            sleep(0.2)

    # Robot state
    def get_pose(self):
        return self.state_connection.get_pose()

    def is_moving(self):
        return self.state_connection.is_moving()

    def is_error_state(self):
        return self.state_connection.is_error_state()

    def read_force_sensor(self):
        raise NotImplementedError

    # Movement
    def move_linear(self, target, speed_ratio):
        acceleration = self.DEFAULT_ACCELERATION
        velocity = self.MAX_VELOCITY * speed_ratio
        time = 0  # When acceleration and velocity are defined, time is not used.
        radius = 0  # Use a blend radius of 0.

        return self.command_connection.move_linear(
            target=target,
            acceleration=acceleration,
            velocity=velocity,
            time=time,
            radius=radius,
        )

    def move_circular(self, start_position, waypoint, target, speed_ratio):
        acceleration = self.DEFAULT_ACCELERATION
        velocity = self.MAX_VELOCITY * speed_ratio
        time = 0  # When acceleration and velocity are defined, time is not used.
        radius = 0  # Use a blend radius of 0.

        return self.command_connection.move_circular(
            start_position=start_position,
            waypoint=waypoint,
            target=target,
            acceleration=acceleration,
            velocity=velocity,
            radius=0,
            mode=0,
        )

    def stop_robot(self):
        return self.command_connection.stop_robot()

    # Destruction and cleanup
    def close(self):
        self.stop_robot()
        self.disconnect()

    # Other

    # TODO: A dummy function, can be removed once the corresponding function from Dobot class is removed.
    def set_target_reached(self, _):
        pass
