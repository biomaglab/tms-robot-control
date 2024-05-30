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

        # If the connection was successful, start the thread for
        # polling the robot state.
        if success:
            self.state_connection.start()

        return success

    def disconnect(self):
        success = self.command_connection.disconnect()
        success = success and self.state_connection.disconnect()

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
    def move_linear(self, target, speed):
        # Use a default acceleration value.
        acceleration = 0.2

        # Use constant velocity for now; ignore 'speed' argument.
        velocity = 0.02

        return self.command_connection.move_linear(
            target=target,
            acceleration=acceleration,
            velocity=velocity,
            time=0,
            radius=0,
        )

    def move_circular(self, start_position, waypoint, target, speed):
        # Use a default acceleration value.
        acceleration = 0.2

        # Use constant velocity for now; ignore 'speed' argument.
        velocity = 0.02

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
