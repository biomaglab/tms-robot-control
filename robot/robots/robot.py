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
        Get the current pose of the robot, or None if the pose is not available.

        :return: The current pose. The pose is a list of 6 values: [x, y, z, rx, ry, rz],
          where x, y, z are the coordinates in mm, and rx, ry, rz are the Euler angles in degrees
          (using 'sxyz' convention, that is, a rotation of rx degrees around the x-axis, followed
          by a rotation of ry degrees around the y-axis, and finally a rotation of rz degrees around
          the z-axis, in a static frame). Return None if the pose is not available.
        """
        pass

    @abstractmethod
    def is_moving(self):
        """
        Return True if the robot is currently moving, False if it is not moving, and None if the
        information is not available.

        :return: True if the robot is moving, False if not moving, None if the information is not available.
        """
        pass

    @abstractmethod
    def is_error_state(self):
        """
        Return True if the robot is in an error state, False if it is not in an error state, and None if the
        information is not available.

        :return: True if in error state, False if not, and None if the information is not available.
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
    def move_linear(self, target, speed_ratio):
        """
        Move the robot in a linear path to the target in robot's base coordinate system with a given speed.

        If the robot is already moving, this method should stop the current movement and start the new one.

        :param target: The target position in a linear path, as a list of 6 values: [x, y, z, rx, ry, rz].
            [x, y, z] define the position in mm, and [rx, ry, rz] define the Euler angles in degrees,
            using the 'sxyz' convention.
        :param speed_ratio: The speed of the movement (as a proportion of the maximum speed; 0-1).
        """
        pass

    @abstractmethod
    def move_circular(self, start_position, waypoint, target, speed_ratio):
        """
        Move the robot in a circular path through a specified waypoint to the target in robot's base
        coordinate system with a given speed.

        If the robot is already moving, this method should stop the current movement and start the new one.

        :param start_position: The starting pose of the robot, as a list of 6 values: [x, y, z, rx, ry, rz].
            The position [x, y, z] is in mm and the Euler angles [rx, ry, rz] are in degrees, using the 'sxyz' convention.
        :param waypoint: The intermediate waypoint, as a list of 6 values: [x, y, z, rx, ry, rz].
            The position [x, y, z] is in mm and the Euler angles [rx, ry, rz] are in degrees, using the 'sxyz' convention.
        :param target: The target pose, as a list of 6 values: [x, y, z, rx, ry, rz].
            The position [x, y, z] is in mm and the Euler angles [rx, ry, rz] are in degrees, using the 'sxyz' convention.
        :param speed_ratio: The speed of the movement (as a proportion of the maximum speed; 0-1).
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
    def enable_free_drive(self):
        pass

    @abstractmethod
    def disable_free_drive(self):
        pass

    @abstractmethod
    def close(self):
        """
        Perform necessary cleanup and disconnection procedures for the robot.
        """
        pass
