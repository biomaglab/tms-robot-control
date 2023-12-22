from time import sleep

import numpy as np

from robot.robots.elfin.elfin_connection import (
    ElfinConnection,
    Axis,
    Direction,
    MotionState,
    ReferenceFrame
)


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

        # Move along the axis that has the largest displacement.
        axis_index = np.argmax(np.abs(displacement))
        axis = Axis(axis_index)
        distance = np.abs(displacement[axis])
        direction = Direction.NEGATIVE if displacement[axis] < 0 else Direction.POSITIVE

        self.connection.move_linear_relative(
            axis=axis,
            direction=direction,
            distance=distance,
        )

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

        self.connection.move_linear_relative(
            axis=axis,
            direction=direction,
            distance=distance,
        )

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
