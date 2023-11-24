import socket

import numpy as np


class DobotConnection:
    DASHBOARD_PORT = 29999
    MOVEMENT_PORT = 30003
    FEEDBACK_PORT = 30004

    def __init__(self, ip):
        self.ip = ip
        self.connected = False

        self.dashboard_socket = None
        self.feedback_socket = None
        self.movement_socket = None

    def connect(self):
        try:
            self.dashboard_socket = socket.socket()
            self.dashboard_socket.connect((self.ip, self.DASHBOARD_PORT))
        except socket.error:
            raise Exception(f"Unable to connect using port {self.DASHBOARD_PORT}", socket.error)

        try:
            self.movement_socket = socket.socket()
            self.movement_socket.connect((self.ip, self.MOVEMENT_PORT))
        except socket.error:
            raise Exception(f"Unable to connect using port {self.MOVEMENT_PORT}", socket.error)

        try:
            self.feedback_socket = socket.socket()
            self.feedback_socket.connect((self.ip, self.FEEDBACK_PORT))
        except socket.error:
            raise Exception(f"Unable to connect using port {self.FEEDBACK_PORT}", socket.error)

        self.connected = True
        return self.connected

    def send_data(self, socket, string):
        try:
            socket.send(str.encode(string, 'utf-8'))
        except ConnectionAbortedError:
            print(f"ConnectionAbortedError. Unable to send message: {string}")

    def wait_reply(self, socket):
        """
        Read the return value
        """
        try:
            data = socket.recv(1024)
            data_str = str(data, encoding="utf-8")
            print(data_str)
            return data_str
        except ConnectionAbortedError:
            print(f"ConnectionAbortedError. Unable to read message")
            return False

    def close(self):
        """
        Close the socket connections.
        """
        if self.dashboard_socket is not None:
            self.dashboard_socket.close()

        if self.feedback_socket is not None:
            self.feedback_socket.close()

        if self.movement_socket is not None:
            self.movement_socket.close()

    def __del__(self):
        self.close()

    # Feedback
    def get_feedback(self):
        bytes_received = 0
        data = bytes()
        while bytes_received < 1440:
            new_bytes = self.connection.feedback_socket.recv(1440 - bytes_received)
            if len(new_bytes) > 0:
                bytes_received += len(new_bytes)
                data += new_bytes

        feedback = np.frombuffer(data, dtype=FeedbackType)
        return feedback

    # Robot commands

    def enable_robot(self):
        """
        Enables the robot.
        """
        request = "EnableRobot()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    def clear_error(self):
        """
        Clears the controller error.
        """
        request = "ClearError()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    def reset_robot(self):
        """
        Resets the robot.
        """
        request = "ResetRobot()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    def get_robot_status(self):
        """
        Gets the robot status.
        """
        request = "RobotMode()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    # Unused for now.
    def power_on(self):
        """
        Powers on the robot.

        Note: It takes about 10 seconds for the robot to be operational after it is powered on.
        """
        request = "PowerOn()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    # Unused for now.
    def get_error_id(self):
        """
        Gets robot error code.
        """
        request = "GetErrorID()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    # Unused for now.
    def get_pose(self):
        """
        Gets the current pose of the robot in the Cartesian coordinate system.
        """
        request = "GetPose()"
        self.send_data(self.dashboard_socket, request)
        return self.wait_reply(self.dashboard_socket)

    def move_linear(self, target):
        """
        Moves the robot to the given target using linear motion.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        """
        request = "MovL({:f},{:f},{:f},{:f},{:f},{:f})".format(
            target[0], target[1], target[2], target[3], target[4], target[5])
        self.send_data(self.movement_socket, request)
        return self.wait_reply(self.movement_socket)

    # Unused for now.
    def move_circular(self, waypoint, target):
        """
        Moves the robot to the given target via a waypoint using circular motion.

        :param: waypoint: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        """
        request = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            waypoint[0], waypoint[1], waypoint[2], waypoint[3], waypoint[4], waypoint[5],
            target[0], target[1], target[2], target[3], target[4], target[5])
        self.send_data(self.movement_socket, request)
        return self.wait_reply(self.movement_socket)

    def move_servo(self, target):
        """
        Moves the robot to the given target using servo-based control.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        """
        request = "ServoP({:f},{:f},{:f},{:f},{:f},{:f})".format(
            target[0], target[1], target[2], target[3], target[4], target[5])
        self.send_data(self.movement_socket, request)
        return self.wait_reply(self.movement_socket)

    def move_linear_relative_to_tool(self, x, y, z, rx, ry, rz, tool):
        """
        Moves the robot with the given offsets in the tool coordinate system.

        The end motion mode is linear motion.

        x: Offset in X-direction
        y: Offset in Y-direction
        z: Offset in Z-direction
        rx: Offset along Rx-axis
        ry: Offset along Ry-axis
        rz: Offset along Rz-axis
        tool: The selected tool, value range: 0-9
        """
        request = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}, {:d})".format(
            x, y, z, rx, ry, rz, tool)
        self.send_data(self.movement_socket, request)
        return self.wait_reply(self.movement_socket)


FeedbackType = np.dtype([(
    'len',
    np.int64,
), (
    'digital_input_bits',
    np.uint64,
), (
    'digital_output_bits',
    np.uint64,
), (
    'robot_mode',
    np.uint64,
), (
    'time_stamp',
    np.uint64,
), (
    'time_stamp_reserve_bit',
    np.uint64,
), (
    'test_value',
    np.uint64,
), (
    'test_value_keep_bit',
    np.float64,
), (
    'speed_scaling',
    np.float64,
), (
    'linear_momentum_norm',
    np.float64,
), (
    'v_main',
    np.float64,
), (
    'v_robot',
    np.float64,
), (
    'i_robot',
    np.float64,
), (
    'i_robot_keep_bit1',
    np.float64,
), (
    'i_robot_keep_bit2',
    np.float64,
), ('tool_accelerometer_values', np.float64, (3, )),
    ('elbow_position', np.float64, (3, )),
    ('elbow_velocity', np.float64, (3, )),
    ('q_target', np.float64, (6, )),
    ('qd_target', np.float64, (6, )),
    ('qdd_target', np.float64, (6, )),
    ('i_target', np.float64, (6, )),
    ('m_target', np.float64, (6, )),
    ('q_actual', np.float64, (6, )),
    ('qd_actual', np.float64, (6, )),
    ('i_actual', np.float64, (6, )),
    ('actual_TCP_force', np.float64, (6, )),
    ('tool_vector_actual', np.float64, (6, )),
    ('TCP_speed_actual', np.float64, (6, )),
    ('TCP_force', np.float64, (6, )),
    ('Tool_vector_target', np.float64, (6, )),
    ('TCP_speed_target', np.float64, (6, )),
    ('motor_temperatures', np.float64, (6, )),
    ('joint_modes', np.float64, (6, )),
    ('v_actual', np.float64, (6, )),
    # ('dummy', np.float64, (9, 6))])
    ('hand_type', np.byte, (4, )),
    ('user', np.byte,),
    ('tool', np.byte,),
    ('run_queued_cmd', np.byte,),
    ('pause_cmd_flag', np.byte,),
    ('velocity_ratio', np.byte,),
    ('acceleration_ratio', np.byte,),
    ('jerk_ratio', np.byte,),
    ('xyz_velocity_ratio', np.byte,),
    ('r_velocity_ratio', np.byte,),
    ('xyz_acceleration_ratio', np.byte,),
    ('r_acceleration_ratio', np.byte,),
    ('xyz_jerk_ratio', np.byte,),
    ('r_jerk_ratio', np.byte,),
    ('brake_status', np.byte,),
    ('enable_status', np.byte,),
    ('drag_status', np.byte,),
    ('running_status', np.byte,),
    ('error_status', np.byte,),
    ('jog_status', np.byte,),
    ('robot_type', np.byte,),
    ('drag_button_signal', np.byte,),
    ('enable_button_signal', np.byte,),
    ('record_button_signal', np.byte,),
    ('reappear_button_signal', np.byte,),
    ('jaw_button_signal', np.byte,),
    ('six_force_online', np.byte,),
    ('reserve2', np.byte, (82, )),
    ('m_actual', np.float64, (6, )),
    ('load', np.float64,),
    ('center_x', np.float64,),
    ('center_y', np.float64,),
    ('center_z', np.float64,),
    ('user[6]', np.float64, (6, )),
    ('tool[6]', np.float64, (6, )),
    ('trace_index', np.float64,),
    ('six_force_value', np.float64, (6, )),
    ('target_quaternion', np.float64, (4, )),
    ('actual_quaternion', np.float64, (4, )),
    ('reserve3', np.byte, (24, ))])
