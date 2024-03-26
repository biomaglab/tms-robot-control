from abc import ABC, abstractmethod

class Robot(ABC):
    """
    Abstract base class for robot communication.

    This class defines the methods for interacting with a robot.
    """

    @abstractmethod
    def __init__(self, ip, config):
        """
        Initialize the robot connection.

        :param ip: The IP address of the robot.
        :param config: Configuration settings for the robot.
        """
        pass

    @abstractmethod
    def connect(self):
        """
        Connect to the robot.
        """
        pass

    @abstractmethod
    def disconnect(self):
        """
        Disconnect the robot.
        """
        pass

    @abstractmethod
    def is_connected(self):
        """
        Check if the robot is connected.

        :return: True if connected, False otherwise.
        """
        pass

    @abstractmethod
    def initialize(self):
        """
        Initialize the robot settings.
        """
        pass

    @abstractmethod
    def get_pose(self):
        """
        Get the current pose of the robot.

        :return: The current pose.
        """
        pass

    @abstractmethod
    def is_moving(self):
        """
        Check if the robot is currently moving.

        :return: True if the robot is moving, False otherwise.
        """
        pass

    @abstractmethod
    def is_error_state(self):
        """
        Check if the robot is in an error state.

        :return: True if in error state, False otherwise.
        """
        pass

    @abstractmethod
    def read_force_sensor(self):
        """
        Read the force sensor's current value.

        :return: The force sensor reading.
        """
        pass

    @abstractmethod
    def move_linear(self, linear_target):
        """
        Move the robot in a linear path to the target in robot's base coordinate system.

        If the robot is already moving, this method should stop the current movement and start the new one.

        :param linear_target: The target position in a linear path.
        """
        pass

    @abstractmethod
    def move_circular(self, start_position, waypoint, target):
        """
        Move the robot in a circular path through a specified waypoint to the target in robot's base
        coordinate system.

        If the robot is already moving, this method should stop the current movement and start the new one.

        :param start_position: The starting position of the robot.
        :param waypoint: The intermediate waypoint.
        :param target: The target position.
        """
        pass

    @abstractmethod
    def stop_robot(self):
        """
        Stop the robot's movement.

        :return: True if the stop command was successful, False otherwise.
        """
        pass

    @abstractmethod
    def close(self):
        """
        Perform necessary cleanup and disconnection procedures for the robot.
        """
        pass
