from time import sleep

import numpy as np

from robot.robots.robot import Robot

from robot.robots.universal_robot.command_connection import (
    CommandConnection,
    MotionMode,
)

from robot.robots.universal_robot.state_connection import (
    StateConnection,
)

from scipy.spatial.transform import Rotation as R

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
    DEFAULT_ACCELERATION = 0.05

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

    # Function to convert a rotation vector to RPY
    def rotvec_to_rpy(self, rotvec):
        # Convert the rotation vector to a Rotation object
        rot = R.from_rotvec(rotvec)

        # Convert the rotation matrix to roll, pitch, yaw (in radians)
        rpy = rot.as_euler('xyz', degrees=True)  # or 'zyx' depending on your convention

        return rpy

    # Function to convert RPY to a rotation vector
    def rpy_to_rotvec(self, rpy):
        # Convert RPY angles (in degrees) to a Rotation object
        rot = R.from_euler('xyz', rpy, degrees=True)  # or 'zyx' depending on your convention

        # Convert the Rotation object to a rotation vector
        rotvec = rot.as_rotvec()

        return rotvec

    # Robot state
    def get_pose(self):
        # TODO: This is currently incorrect; state_connection.get_pose() returns [x, y, z, rx, ry, rz] where
        #   [rx, ry, rz] define a rotation vector, whereas the robot interface should return Euler angles (using 'sxyz' convention;
        #   see robot.py). This would be the correct place to do the transformation.
        success, pose = self.state_connection.get_pose()
        if not success:
            return success, None
        euler_angles = self.rotvec_to_rpy(pose[3:])
        return success, [pose[0], pose[1], pose[2], euler_angles[0], euler_angles[1], euler_angles[2]]
        #return self.state_connection.get_pose()

    def is_moving(self):
        return self.state_connection.is_moving()

    def is_error_state(self):
        return self.state_connection.is_error_state()

    def read_force_sensor(self):
        raise NotImplementedError

    # Movement
    def move_linear(self, target, speed_ratio):
        # TODO: This is currently incorrect; target is defined as [x, y, z, rx, ry, rz] where
        #   [rx, ry, rz] are the Euler angles, whereas the robot interface takes in rotation vectors.
        #   This would be the correct correct place to do the transformation.

        acceleration = self.DEFAULT_ACCELERATION
        velocity = self.MAX_VELOCITY * speed_ratio
        time = 0  # When acceleration and velocity are defined, time is not used.
        radius = 0  # Use a blend radius of 0.

        return self.command_connection.move_linear(
            target=self.convert_to_meters_and_radians(target),
            acceleration=acceleration,
            velocity=velocity,
            time=time,
            radius=radius,
        )

    def move_circular(self, start_position, waypoint, target, speed_ratio):
        # TODO: This is currently incorrect; start_position, waypoint, and target as defined as [x, y, z, rx, ry, rz] where
        #   [rx, ry, rz] are the Euler angles, whereas the robot interface takes in rotation vectors. This would be the correct
        #   correct place to do the transformation.

        acceleration = self.DEFAULT_ACCELERATION
        velocity = self.MAX_VELOCITY * speed_ratio
        mode = MotionMode.UNCONSTRAINED
        radius = 0  # Use a blend radius of 0.

        return self.command_connection.move_circular(
            waypoint=self.convert_to_meters_and_radians(waypoint),
            target=self.convert_to_meters_and_radians(target),
            acceleration=acceleration,
            velocity=velocity,
            radius=radius,
            mode=mode,
        )

    def stop_robot(self):
        return self.command_connection.stop_robot(0.5)

    # Destruction and cleanup
    def close(self):
        self.stop_robot()
        self.disconnect()

    # Other
    def convert_to_meters_and_radians(self, pose):
        position_in_mm = pose[:3]
        orientation_in_degrees = pose[3:]

        position_in_meters = [value / 1000 for value in position_in_mm]
        rotations_in_radians = self.rpy_to_rotvec(orientation_in_degrees)

        return position_in_meters + list(rotations_in_radians)

    # TODO: A dummy function, can be removed once the corresponding function from Dobot class is removed.
    def set_target_reached(self, _):
        pass

    def enable_free_drive(self):
        pass

    def disable_free_drive(self):
        pass