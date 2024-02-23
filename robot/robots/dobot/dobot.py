# Code based on: https://github.com/Dobot-Arm/TCP-IP-CR-Python

from enum import Enum

import time
from threading import Thread

import numpy as np

import robot.control.robot_processing as robot_process
from robot.robots.dobot.dobot_connection import DobotConnection, RobotStatus


# TODO: The motion type, indicating the state that the robot movement algorithm is in, spills into Dobot class,
#   which should be the low-level robot control class. Hence the MotionType enum is copied below from the
#   RadiallyOutwardsAlgorithm class so that it can be accessed by Dobot class. This should be fixed by removing
#   references to MotionType enum from Dobot class; it should not know about the different motion types.
class MotionType(Enum):
    NORMAL = 0
    LINEAR_OUT = 1
    ARC = 2
    FORCE_LINEAR_OUT = 3
    TUNING = 4


class Dobot:
    """
    The class for communicating with Dobot robot.
    """
    TOOL_ID = 0
    TIMEOUT_START_MOTION = 10
    TIMEOUT_MOTION = 45

    def __init__(self, ip, config, robot_config):
        self.robot_config = robot_config

        # TODO: Unused for now.
        self.robot_speed = config['robot_speed']

        self.stop_threads = False

        self.moving = False
        self.thread_move = False

        self.coordinates = 6 * [None]
        self.force_torque_data = 6 * [None]
        self.robot_status = None
        self.running_status = 0

        self.status_move = False
        self.target = [None] * 6
        self.target_reached = False
        self.motion_type = MotionType.NORMAL

        self.connected = False

        self.connection = DobotConnection(ip=ip)

    def connect(self):
        self.connected = self.connection.connect()

        self.moving = False
        self._set_feedback()
        self._set_move_thread()

        time.sleep(2)
        print("Dobot initialization status: ", self.robot_status)
        if any(coord is None for coord in self.coordinates):
            print("Please, restart robot")
            return

        if self.robot_status == RobotStatus.DISABLED.value:
            print('Enabling robot...')
            self.connection.enable_robot()
            time.sleep(1)

        if self.robot_status == RobotStatus.ERROR.value:
            print('Cleaning errors...')
            self.connection.clear_error()
            time.sleep(1)

        return self.connected

    def is_moving(self):
        motion_state = self.robot_status
        return motion_state == RobotStatus.RUNNING.value

    def is_error_state(self):
        status = self.connection.get_robot_status()
        return status == RobotStatus.ERROR.value

    def initialize(self):
        self.connection.set_speed_ratio(self.robot_speed)

    def get_pose(self):
        # Always successfully return coordinates.
        return True, self.coordinates

    # TODO: This function is needlessly complicating the API between robot control and this class; there
    #   should not be a need for this low-level robot control class to know if the target has been reached.
    def set_target_reached(self, target_reached):
        self.target_reached = target_reached

    # TODO: Note that move_linear, move_circular, and
    #   tune_robot functions are almost identical at this stage.
    #   This is because the distinction between the motion types
    #   should really go deeper into the structure of this class,
    #   because the different motion types have different structure and
    #   meaning for the parameter 'target'. Hence, we should not use the same
    #   variable (self.target) for each. After that change is implemented,
    #   the functions below will branch off.

    def move_linear(self, linear_target):
        self.target = linear_target
        self.motion_type = MotionType.NORMAL
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

        # TODO: Properly handle errors and return the success of the movement here.
        return True

    def move_circular(self, start_position, waypoint, target):
        # TODO: Start position, waypoint, and target should be stored in three separate
        #   variables, not in one variable (self.target).
        self.target = start_position, waypoint, target
        self.motion_type = MotionType.ARC
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

        # TODO: Properly handle errors and return the success of the movement here.
        return True

    def read_force_sensor(self):
        # TODO: Should return False if the force sensor cannot be read. Currently the error does
        #   not propagate to the caller.
        return True, self.force_torque_data

    def compensate_force(self):
        # If the robot is in an error state, return early.
        if self.is_error_state():
            return

        offsets = [0, 0, -2, 0, 0, 0]
        self.connection.move_linear_relative_to_tool(
            offsets=offsets,
            tool=self.TOOL_ID,
        )

        # TODO: Properly handle errors and return the success of the movement here.
        return True

    def stop_robot(self):
        self.status_move = False
        self.connection.reset_robot()

        # Gracefully stop the movement thread.
        if self.moving:
            self.moving = False
            if self.thread_move:
                try:
                    self.thread_move.join()
                except RuntimeError:
                    pass

    def close(self):
        self.stop_robot()
        self.connected = False
        self.moving = False
        if self.thread_move:
            try:
                self.thread_move.join()
            except RuntimeError:
                pass
        #TODO: robot function to close? self.cobot.close()

    ## Internal methods

    def _set_feedback(self):
        if self.connected:
            thread = Thread(target=self._feedback)
            thread.setDaemon(True)
            thread.start()

    def _feedback(self):
        while True:
            if not self.connected:
                break

            feedback = self.connection.get_feedback()

            # Refresh coordinate points
            self.coordinates = np.array(feedback["tool_vector_actual"][0])
            self.force_torque_data = np.array(feedback["six_force_value"][0])
            #OR self.force_torque_data = feedback["actual_TCP_force"][0]
            self.robot_status = int(feedback["robot_mode"][0])
            self.running_status = int(feedback["running_status"][0])

            #time.sleep(0.001)

    def _set_move_thread(self):
        if self.connected:
            thread = Thread(target=self._move_thread)
            thread.daemon = True
            thread.start()
            self.thread_move = thread

    def _motion_loop(self):
        timeout_start = time.time()
        while self.running_status != 1:
            if time.time() > timeout_start + self.TIMEOUT_START_MOTION:
                print("break")
                self.stop_robot()
                break
            time.sleep(0.001)

        while self.running_status == 1:
            status = self.robot_status
            if status == RobotStatus.ERROR.value:
                self.stop_robot()
            if time.time() > timeout_start + self.TIMEOUT_MOTION:
                self.stop_robot()
                print("break")
                break
            time.sleep(0.001)

    def _move_thread(self):
        while True:
            if self.robot_status == RobotStatus.ERROR.value:
                print('Cleaning errors...')
                self.connection.clear_error()
            if not self.moving:
                self.stop_robot()
                break
            if self.status_move and not self.target_reached and not self.running_status:
                print('moving')
                if self.motion_type == MotionType.NORMAL or self.motion_type == MotionType.LINEAR_OUT:
                    self.connection.move_linear(self.target)
                    self._motion_loop()
                elif self.motion_type == MotionType.ARC:
                    arc_bezier_curve_step = self.robot_config['arc_bezier_curve_step']
                    curve_set = robot_process.bezier_curve(
                        points=np.asarray(self.target),
                        step=arc_bezier_curve_step,
                    )
                    target = self.target
                    for curve_point in curve_set:
                        self.connection.move_servo(curve_point)
                        self._motion_loop()
                        if self.motion_type != MotionType.ARC:
                            self.stop_robot()
                            break

                        distance_threshold_for_arc_motion = self.robot_config['distance_threshold_for_arc_motion']
                        if not np.allclose(np.array(self.target[2][:3]), np.array(target[2][:3]), 0, distance_threshold_for_arc_motion):
                            self.stop_robot()
                            break
                        if not self.moving:
                            self.stop_robot()
                            break
                elif self.motion_type == MotionType.TUNING:
                    self.connection.move_linear_relative_to_tool(
                      offsets=self.target,
                      tool=self.TOOL_ID
                    )
                    self._motion_loop()

            time.sleep(0.001)
