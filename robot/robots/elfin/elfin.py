from time import sleep

import numpy as np

import robot.constants as const
from robot.constants import MotionType
from robot.robots.elfin.elfin_connection import ElfinConnection
from robot.robots.elfin.elfin_connection_linux import ElfinConnectionLinux
from robot.robots.elfin.motion_state import MotionState


class Elfin():
    """
    The class for communicating with Elfin robot.
    """
    def __init__(self, ip, use_linux_version=False):
        self.coordinates = [None]*6
        self.target_reached = False

        self.use_linux_version = use_linux_version

        self.connection = ElfinConnectionLinux(ip) if self.use_linux_version else ElfinConnection(ip)

    def Connect(self):
        self.connection.connect()

    def IsConnected(self):
        return self.connection.connected

    def GetCoordinates(self):
        coordinates = self.connection.GetCoordinates()
        if coordinates:
            self.coordinates = coordinates
        return self.coordinates

    def SetTargetReached(self, target_reached):
        self.target_reached = target_reached

    # Note: It is not possible to send a move command to elfin during movement.

    def MoveLinear(self, linear_target):
        motion_state = self.connection.GetMotionState()
        # TODO: Should motion state be used here to check that robot is free to move?

        self.connection.MoveLinear(linear_target)

    def MoveCircular(self, start_position, waypoint, target):
        motion_state = self.connection.GetMotionState()

        # If the robot is in an error state, stop the robot and return early.
        if motion_state == MotionState.ERROR:
            self.StopRobot()
            return

        self.connection.MoveCircular(start_position, waypoint[:3], target)

    def TuneRobot(self, displacement):
        motion_state = self.connection.GetMotionState()

        # If the robot is not free to move, return early.
        #
        # TODO: Should this follow a logic similar to MoveCircular function?
        if motion_state != MotionState.FREE_TO_MOVE:
            return

        self.connection.SetToolCoordinateMotion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
        #self.connection.SetSpeedRatio(0.1)  # Setting robot's movement speed

        # Move along the axis that has the largest displacement.
        axis = np.argmax(np.abs(displacement))
        distance = displacement[axis]

        # Always move to the positive direction; hence set direction to 1.
        #
        # TODO: Shouldn't direction be determined based on the sign of the max displacement
        #   rather than always being positive?
        direction = 1

        self.connection.MoveLinearRelative(
            axis=axis,
            direction=direction,
            distance=distance,
        )

        self.connection.SetToolCoordinateMotion(0)

    def ReadForceSensor(self):
        # If force sensor is not used, return early.
        if not const.FORCE_TORQUE_SENSOR:
            return False

        return self.connection.ReadForceSensor()

    def CompensateForce(self, flag):
        motion_state = self.connection.GetMotionState()

        # If the robot is not free to move, return early.
        if motion_state != MotionState.FREE_TO_MOVE:
            return

        if not flag:
            self.StopRobot()
        self.connection.SetToolCoordinateMotion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
        #self.connection.SetSpeedRatio(0.1)  # Setting robot's movement speed

        axis = 2
        # Move to the negative direction; hence set direction to 0.
        direction = 0
        distance = 1

        self.connection.MoveLinearRelative(
            axis=axis,
            direction=direction,
            distance=distance,
        )
        self.connection.SetToolCoordinateMotion(0)

    def StopRobot(self):
        # Takes some microseconds to the robot actual stops after the command.
        # The sleep time is required to guarantee the stop
        self.connection.StopRobot()
        sleep(0.05)

    def ForceStopRobot(self):
        self.StopRobot()

    def Close(self):
        self.StopRobot()
        #TODO: robot function to close? self.connection.close()
