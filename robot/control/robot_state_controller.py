import time
from enum import Enum

import numpy as np


class RobotState(Enum):
    READY = 0
    MOVING = 1
    WAITING = 2


class RobotStateController:
    """
    The class for controlling the state of the robot.

    The robot has three states:

    - READY: The robot is ready to receive a new command.
    - MOVING: The robot is moving.
    - WAITING: The robot has stopped moving and is waiting for a while before going back to READY.
    """
    def __init__(self, robot, dwell_time):
        self.robot = robot
        self.dwell_time = dwell_time

        self.state = RobotState.READY
        self.previous_state = None

    def update(self):
        self.previous_state = self.state

        # Check if the robot was moving but is not anymore.
        if self.state == RobotState.MOVING and not self.robot.is_moving():
            self.state = RobotState.WAITING
            self.waiting_start_time = time.time()

        # If we are in WAITING, check if we have waited long enough.
        if self.state == RobotState.WAITING:
            waited_for = time.time() - self.waiting_start_time
            self.remaining_dwell_time = self.dwell_time - waited_for

            # If we have waited long enough, go back to READY.
            if self.remaining_dwell_time <= 0:
                self.state = RobotState.READY

    def print_state(self):
        if self.state == RobotState.READY and self.previous_state != RobotState.READY:
            print("Robot state: READY")
        elif self.state == RobotState.MOVING and self.previous_state != RobotState.MOVING:
            print("Robot state: MOVING")
        elif self.state == RobotState.WAITING and self.previous_state != RobotState.WAITING:
            print("Robot state: WAITING, remaining dwell time: {:.2f} s".format(self.remaining_dwell_time))

    def get_state(self):
        return self.state

    def set_state_to_moving(self):
        # TODO: Dobot has internal methods to check and control if the robot is moving and the controller was making
        #  dobot have weird behavior. The best approach would be to use the controller instead of the internal methods.
        #  By now, for dobot, its requires that the self.config['dwell_time'] is equal to zero.
        if not self.dwell_time:
            return

        self.state = RobotState.MOVING
