from time import sleep

import numpy as np

from robot.robots.elfin.elfin_connection import ElfinConnection
from robot.robots.elfin.motion_state import MotionState


class Elfin():
    """
    The class for communicating with Elfin robot.
    """
    def __init__(self, ip, use_new_api=False):
        self.coordinates = [None]*6
        self.target_reached = False

        self.connection = ElfinConnection(
            ip=ip,
            use_new_api=use_new_api,
        )

    def connect(self):
        return self.connection.connect()

    def get_coordinates(self):
        success, coordinates = self.connection.get_coordinates()

        # Only update coordinates if the reading was successful.
        if success:
            self.coordinates = coordinates

        # Return the latest coordinates regardless of the success of the reading.
        return self.coordinates

    def set_target_reached(self, target_reached):
        self.target_reached = target_reached

    # Note: It is not possible to send a move command to elfin during movement.

    def move_linear(self, linear_target):
        motion_state = self.connection.get_motion_state()
        # TODO: Should motion state be used here to check that robot is free to move?

        self.connection.move_linear(linear_target)

    def move_circular(self, start_position, waypoint, target):
        motion_state = self.connection.get_motion_state()

        # If the robot is in an error state, stop the robot and return early.
        if motion_state == MotionState.ERROR:
            self.stop_robot()
            return

        self.connection.move_circular(start_position, waypoint[:3], target)

    def tune_robot(self, displacement):
        motion_state = self.connection.get_motion_state()

        # If the robot is not free to move, return early.
        #
        # TODO: Should this follow a logic similar to MoveCircular function?
        if motion_state != MotionState.FREE_TO_MOVE:
            return

        self.connection.set_tool_coordinate_motion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
        #self.connection.set_speed_ratio(0.1)  # Setting robot's movement speed

        # Move along the axis that has the largest displacement.
        axis = np.argmax(np.abs(displacement))
        distance = abs(displacement[axis])
        direction = 0 if displacement[axis] < 0 else 1

        self.connection.move_linear_relative(
            axis=axis,
            direction=direction,
            distance=distance,
        )

        self.connection.set_tool_coordinate_motion(0)

    def read_force_sensor(self):
        return self.connection.read_force_sensor()

    def compensate_force(self):
        motion_state = self.connection.get_motion_state()

        # If the robot is not free to move, return early.
        if motion_state != MotionState.FREE_TO_MOVE:
            return

        self.connection.set_tool_coordinate_motion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
        #self.connection.set_speed_ratio(0.1)  # Setting robot's movement speed

        axis = 2
        # Move to the negative direction; hence set direction to 0.
        direction = 0
        distance = 1

        self.connection.move_linear_relative(
            axis=axis,
            direction=direction,
            distance=distance,
        )
        self.connection.set_tool_coordinate_motion(0)

    def stop_robot(self):
        # Takes some microseconds to the robot actual stops after the command.
        # The sleep time is required to guarantee the stop
        self.connection.stop_robot()
        sleep(0.05)

    def force_stop_robot(self):
        self.stop_robot()

    def close(self):
        self.stop_robot()
        #TODO: robot function to close? self.connection.close()
