from time import sleep

import numpy as np

from robot.robots.elfin.elfin_connection import (
    ElfinConnection,
    MotionState,
    ReferenceFrame
)

from robot.robots.axis import Axis
from robot.robots.direction import Direction


class Elfin():
    """
    The class for communicating with Elfin robot.
    """
    SPEED_RATIO = 0.20

    def __init__(self, ip, use_new_api=False):
        self.coordinates = 6 * [None]
        self.target_reached = False

        self.connection = ElfinConnection(
            ip=ip,
            use_new_api=use_new_api,
        )

    def is_moving(self):
        motion_state = self.connection.get_motion_state()
        return motion_state == MotionState.IN_MOTION

    def initialize(self):
        # With the new Elfin firmware (5.51.9.beta.20230703), the speed ratio resets to 1% at start-up.
        # Hence, set it programmatically to a higher value.
        self.connection.set_speed_ratio(self.SPEED_RATIO)

        # With the new firmware (5.51.9.beta.20230703), changing between reference frames is not possible
        # during movement. Hence, set the reference frame to tool here, as it is the only reference frame used;
        # it only applies to relative movement, which is always done with respect to the tool.
        self.connection.set_reference_frame(ReferenceFrame.TOOL)

    def connect(self):
        return self.connection.connect()

    def get_coordinates(self):
        success, coordinates = self.connection.get_coordinates()

        # Only update coordinates if the reading was successful.
        if success:
            self.coordinates = coordinates

        # Return the latest coordinates regardless of the success of the reading.
        return self.coordinates

    # TODO: A dummy function, can be removed once the corresponding function from Dobot class is removed.
    def set_target_reached(self, _):
        pass

    # Note: It is not possible to send a move command to elfin during movement.

    def move_linear(self, linear_target):
        motion_state = self.connection.get_motion_state()
        # TODO: Should motion state be used here to check that robot is free to move?

        success = self.connection.move_linear(linear_target)
        return success

    def move_circular(self, start_position, waypoint, target):
        motion_state = self.connection.get_motion_state()

        # If the robot is in an error state, stop the robot and return early.
        if motion_state == MotionState.ERROR:
            self.stop_robot()
            return

        success = self.connection.move_circular(start_position, waypoint, target)
        return success

    # Not supported by Elfin.
    def move_linear_relative_to_tool(self, displacement):
        raise NotImplementedError

    def move_linear_relative_to_tool_on_single_axis(self, axis, direction, distance):
        motion_state = self.connection.get_motion_state()

        # If the robot is not free to move, return early.
        #
        # TODO: Should this follow a logic similar to MoveCircular function?
        if motion_state != MotionState.FREE_TO_MOVE:
            return

        success = self.connection.move_linear_relative(
            axis=axis,
            direction=direction,
            distance=distance,
        )
        return success

    def read_force_sensor(self):
        return self.connection.read_force_sensor()

    def compensate_force(self):
        # If the robot is not free to move, return early.
        motion_state = self.connection.get_motion_state()
        if motion_state != MotionState.FREE_TO_MOVE:
            return

        axis = Axis.Z
        direction = Direction.NEGATIVE
        distance = 1

        success = self.connection.move_linear_relative(
            axis=axis,
            direction=direction,
            distance=distance,
        )
        return success

    def stop_robot(self):
        self.connection.stop_robot()

        # After the stop command, it takes some microseconds for the robot to stop;
        # wait here to guarantee the stopping.
        sleep(0.05)

    def force_stop_robot(self):
        self.stop_robot()

    def close(self):
        self.stop_robot()
        # TODO: Should the socket connection to the robot be closed?
