# Code based on: https://github.com/Dobot-Arm/TCP-IP-CR-Python

import time
from threading import Thread

import numpy as np

import robot.control.robot_processing as robot_process
from robot.constants import MotionType
from robot.robots.dobot.dobot_connection import DobotConnection


class Dobot:
    """
    The class for communicating with Dobot robot.
    """
    TOOL_ID = 0
    TIMEOUT_START_MOTION = 10
    TIMEOUT_MOTION = 45
    ERROR_STATUS = 9

    def __init__(self, ip, robot_config):
        self.robot_config = robot_config

        self.stop_threads = False

        self.moving = False
        self.thread_move = False

        self.coordinates = [None]*6
        self.force_torque_data = [None] * 6
        self.robot_mode = 0
        self.running_status = 0

        self.status_move = False
        self.target = [None] * 6
        self.target_reached = False
        self.motion_type = MotionType.NORMAL

        self.connected = False

        self.connection = DobotConnection(ip=ip)

    def connect(self):
        self.connection.connect()

        self.moving = False
        self._set_feedback()
        self._set_move_thread()

        time.sleep(2)
        if any(coord is None for coord in self.coordinates):
            print("Please, restart robot")
            return

        if self.robot_mode == 4:
            self.connection.enable_robot()
            time.sleep(1)

        if self.robot_mode == 9:
            self.connection.clear_error()
            time.sleep(1)

    def get_coordinates(self):
        return self.coordinates

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

    def move_circular(self, start_position, waypoint, target):
        # TODO: Start position, waypoint, and target should be stored in three separate
        #   variables, not in one variable (self.target).
        self.target = start_position, waypoint, target
        self.motion_type = MotionType.ARC
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

    def tune_robot(self, displacement):
        self.target = displacement
        self.motion_type = MotionType.TUNING
        self.status_move = True
        if not self.moving:
            self.moving = True
            self._set_move_thread()

    def read_force_sensor(self):
        # TODO: Should return False if the force sensor cannot be read. Currently the error does
        #   not propagate to the caller.
        return True, self.force_torque_data

    def compensate_force(self):
        status = self.connection.get_robot_status()
        print("CompensateForce")
        if status != self.ERROR_STATUS:
            #self.cobot.SetOverride(0.1)  # Setting robot's movement speed
            offset_x = 0
            offset_y = 0
            offset_z = -2
            offset_rx = 0
            offset_ry = 0
            offset_rz = 0
            self.connection.move_linear_relative_to_tool(
                offset_x,
                offset_y,
                offset_z,
                offset_rx,
                offset_ry,
                offset_rz,
                tool=self.TOOL_ID
            )

    def stop_robot(self):
        # Takes some microseconds to the robot actual stops after the command.
        # The sleep time is required to guarantee the stop
        self.status_move = False
        #if self.running_status == 1:
        self.connection.reset_robot()
        #time.sleep(0.05)

    def force_stop_robot(self):
        self.stop_robot()
        if self.moving:
            self.moving = False
            print("ForceStopRobot")
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
            self.coordinates = feedback["tool_vector_actual"][0]
            self.force_torque_data = feedback["six_force_value"][0]
            #OR self.force_torque_data = feedback["actual_TCP_force"][0]
            self.robot_mode = int(feedback["robot_mode"][0])
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
            status = int(self.robot_mode)
            if status == self.ERROR_STATUS:
                self.stop_robot()
            if time.time() > timeout_start + self.TIMEOUT_MOTION:
                self.stop_robot()
                print("break")
                break
            time.sleep(0.001)

    def _move_thread(self):
        while True:
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

                        arc_threshold_distance = self.robot_config['arc_threshold_distance']
                        if not np.allclose(np.array(self.target[2][:3]), np.array(target[2][:3]), 0, arc_threshold_distance):
                            self.stop_robot()
                            break
                        if not self.moving:
                            self.stop_robot()
                            break
                elif self.motion_type == MotionType.TUNING:
                    offset_x = self.target[0]
                    offset_y = self.target[1]
                    offset_z = self.target[2]
                    offset_rx = self.target[3]
                    offset_ry = self.target[4]
                    offset_rz = self.target[5]
                    self.connection.move_linear_relative_to_tool(
                      offset_x,
                      offset_y,
                      offset_z,
                      offset_rx,
                      offset_ry,
                      offset_rz,
                      tool=self.TOOL_ID
                    )
                    self._motion_loop()

            time.sleep(0.001)
