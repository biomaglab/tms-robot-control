import time
from enum import Enum

import numpy as np


class RobotState(Enum):
    READY = 0
    START_MOVING = 1
    MOVING = 2
    WAITING = 3
    STOPPING = 4
    WAITING_FOR_KEYPRESS = 5


class RobotStateController:
    """
    The class for controlling the state of the robot.

    The robot has three states:

    - READY: The robot is ready to receive a new command.
    - START_MOVING: The robot has received a command to move but has not started moving yet.
    - MOVING: The robot is moving.
    - WAITING: The robot has stopped moving and is waiting for a while before going back to READY.
    - STOPPING: The robot has been issued a stop command and is stopping. Once finished, it will go back to READY.
    - WAITING_FOR_KEYPRESS: The robot is waiting for a keypress to move back to READY.
    """
    def __init__(self, robot, config):
        self.robot = robot

        self.dwell_time = config['dwell_time']
        self.wait_for_keypress = config['wait_for_keypress']

        self.state = RobotState.READY if not self.wait_for_keypress else RobotState.WAITING_FOR_KEYPRESS
        self.previous_state = None

        self.not_moving_counter = 0

        self.keypress = False

        self.print_state()

    def keypress_detected(self):
        self.keypress = True

    def update(self):
        self.previous_state = self.state

        stopped_moving = False

        # Check if the robot is starting to move
        if self.state == RobotState.START_MOVING:
            # Check if the robot has started moving; if it has, change the state to MOVING.
            if self.robot.is_moving():
                self.state = RobotState.MOVING
            else:
                # Sometimes the robot movement may be over already before this 'update' method is called.
                #
                # Infer if that's the case by checking if the movement command has been issued (resulting in the
                # robot being in the START_MOVING state) but the robot has not moved for a while.
                self.not_moving_counter += 1
                if self.not_moving_counter > 10:
                    stopped_moving = True

        # Check if the robot was previously detected to be moving but is not moving anymore.
        if self.state == RobotState.MOVING and not self.robot.is_moving():
            stopped_moving = True

        # If the robot has stopped moving, change the state to WAITING.
        if stopped_moving:
            self.state = RobotState.WAITING
            self.waiting_start_time = time.time()

        # If we are in WAITING, check if we have waited long enough.
        if self.state == RobotState.WAITING:
            waited_for = time.time() - self.waiting_start_time
            self.remaining_dwell_time = self.dwell_time - waited_for

            # If we have waited long enough, go back to READY.
            if self.remaining_dwell_time <= 0:
                self.state = RobotState.READY if not self.wait_for_keypress else RobotState.WAITING_FOR_KEYPRESS

        # If we are in STOPPING, check if we should go back to READY.
        if self.state == RobotState.STOPPING and not self.robot.is_moving():
            # XXX: At least Elfin's new version (using Linux) is not ready to receive a new movement
            #   command immediately when it reports not moving. This is a workaround to wait for a while
            #   before going back to READY.
            self.not_moving_counter += 1
            if self.not_moving_counter > 5:
                self.state = RobotState.READY if not self.wait_for_keypress else RobotState.WAITING_FOR_KEYPRESS

        # If we are in WAITING_FOR_KEYPRESS, check if a keypress has been detected; if so, set state to READY.
        if self.state == RobotState.WAITING_FOR_KEYPRESS:
            if self.keypress:
                self.state = RobotState.READY
                self.keypress = False

        # Print the state if it has changed.
        self.print_state()

    def print_state(self):
        state_changed = self.state != self.previous_state
        if state_changed:
            if self.state != RobotState.WAITING:
                print("Robot state: {}".format(self.state.name))
            else:
                print("Robot state: WAITING, remaining dwell time: {:.2f} s".format(self.remaining_dwell_time))

    def get_state(self):
        return self.state

    def set_state_to_start_moving(self):
        # TODO: Dobot has internal methods to check and control if the robot is moving and the controller was making
        #  dobot have weird behavior. The best approach would be to use the controller instead of the internal methods.
        #  By now, for dobot, its requires that the self.config['dwell_time'] is equal to zero.
        if not self.dwell_time:
            return

        self.state = RobotState.START_MOVING
        self.not_moving_counter = 0

    def set_state_to_stopping(self):
        self.state = RobotState.STOPPING
        self.not_moving_counter = 0
