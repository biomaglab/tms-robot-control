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

    def EnableRobot(self):
        """
        Enable the robot
        """
        string = "EnableRobot()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    def ClearError(self):
        """
        Clear controller alarm information
        """
        string = "ClearError()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    def ResetRobot(self):
        """
        Robot stop
        """
        string = "ResetRobot()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    def RobotMode(self):
        """
        View the robot status
        """
        string = "RobotMode()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    # Unused for now.
    def PowerOn(self):
        """
        Powering on the robot
        Note: It takes about 10 seconds for the robot to be enabled after it is powered on.
        """
        string = "PowerOn()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    # Unused for now.
    def GetErrorID(self):
        """
        Get robot error code
        """
        string = "GetErrorID()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    # Unused for now.
    def GetPose(self):
        """
        Description: get the current pose of the robot under the Cartesian coordinate system
        """
        string = "GetPose()"
        self.send_data(self.dashboard_socket, string)
        return self.wait_reply(self.dashboard_socket)

    def MoveLinear(self, target):
        """
        Coordinate system motion interface (linear motion mode)
        x: A number in the Cartesian coordinate system x
        y: A number in the Cartesian coordinate system y
        z: A number in the Cartesian coordinate system z
        rx: Position of Rx axis in Cartesian coordinate system
        ry: Position of Ry axis in Cartesian coordinate system
        rz: Position of Rz axis in Cartesian coordinate system
        """
        x, y, z, rx, ry, rz = target[0], target[1], target[2], target[3], target[4], target[5]
        string = "MovL({:f},{:f},{:f},{:f},{:f},{:f})".format(
            x, y, z, rx, ry, rz)
        self.send_data(self.movement_socket, string)
        return self.wait_reply(self.movement_socket)

    # Unused for now.
    def MoveCircular(self, target):
        """
        Circular motion instruction
        x1, y1, z1, a1, b1, c1 :Is the point value of intermediate point coordinates
        x2, y2, z2, a2, b2, c2 :Is the value of the end point coordinates
        Note: This instruction should be used together with other movement instructions
        """
        x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2 = target[0], target[1], target[2], target[3], target[4], target[5], \
                                                        target[6], target[7], target[8], target[9], target[10], target[11]
        string = "Arc({:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f},{:f})".format(
            x1, y1, z1, a1, b1, c1, x2, y2, z2, a2, b2, c2)
        self.send_data(self.movement_socket, string)
        return self.wait_reply(self.movement_socket)

    def ServoP(self, target):
        """
        Dynamic following command based on Cartesian space
        x, y, z, a, b, c :Cartesian coordinate point value
        """
        x, y, z, rx, ry, rz = target[0], target[1], target[2], target[3], target[4], target[5]
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f})".format(
            x, y, z, rx, ry, rz)
        self.send_data(self.movement_socket, string)
        return self.wait_reply(self.movement_socket)

    def RelMovLTool(self, offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, tool, *dynParams):
        """
        Carry out relative motion command along the tool coordinate system, and the end motion mode is linear motion
        offset_x: X-axis direction offset
        offset_y: Y-axis direction offset
        offset_z: Z-axis direction offset
        offset_rx: Rx axis position
        offset_ry: Ry axis position
        offset_rz: Rz axis position
        tool: Select the calibrated tool coordinate system, value range: 0 ~ 9
        *dynParams: parameter Settings（speed_l, acc_l, user）
                    speed_l: Set Cartesian speed scale, value range: 1 ~ 100
                    acc_l: Set acceleration scale value, value range: 1 ~ 100
                    user: Set user coordinate system index
        """
        string = "RelMovLTool({:f},{:f},{:f},{:f},{:f},{:f}, {:d}".format(
            offset_x, offset_y, offset_z, offset_rx, offset_ry, offset_rz, tool)
        for params in dynParams:
            print(type(params), params)
            string = string + ", SpeedJ={:d}, AccJ={:d}, User={:d}".format(
                params[0], params[1], params[2])
        string = string + ")"
        self.send_data(self.movement_socket, string)
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
