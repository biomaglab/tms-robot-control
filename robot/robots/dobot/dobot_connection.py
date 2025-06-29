import socket
from enum import Enum

import numpy as np


class RobotStatus(Enum):
    DISABLED = 4
    IDLE = 5
    RUNNING = 7
    ERROR = 9


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
            Exception(
                f"Unable to connect using port {self.DASHBOARD_PORT}", socket.error
            )
            return False

        try:
            self.movement_socket = socket.socket()
            self.movement_socket.connect((self.ip, self.MOVEMENT_PORT))
        except socket.error:
            Exception(
                f"Unable to connect using port {self.MOVEMENT_PORT}", socket.error
            )
            return False

        try:
            self.feedback_socket = socket.socket()
            self.feedback_socket.connect((self.ip, self.FEEDBACK_PORT))
        except socket.error:
            Exception(
                f"Unable to connect using port {self.FEEDBACK_PORT}", socket.error
            )
            return False

        self.connected = True
        return self.connected

    def _send_and_receive(self, socket, request):
        """
        Send a request via the given socket and wait for a reply.

        Return the string received, or an empty string if unsuccessful.
        """
        try:
            socket.send(str.encode(request, "utf-8"))
        except ConnectionAbortedError:
            print(f"ConnectionAbortedError. Unable to send message: {request}")
            return ""

        try:
            response = socket.recv(1024)
            response_str = str(response, encoding="utf-8")
            print(response_str)
            return response_str
        except ConnectionAbortedError:
            print("ConnectionAbortedError. Unable to read message")
            return ""

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
            new_bytes = self.feedback_socket.recv(1440 - bytes_received)
            if len(new_bytes) > 0:
                bytes_received += len(new_bytes)
                data += new_bytes

        feedback = np.frombuffer(data, dtype=FeedbackType)
        return feedback

    # Helpers
    def list_to_str(self, list):
        """
        Converts a list of numbers to a string.

        :param: list: A list of numbers, e.g. [1,2,3].
        :return: A string representation of the list, e.g. "1,2,3".
        """
        return ",".join([str(s) for s in list])

    # Robot commands

    def enable_robot(self):
        """
        Enables the robot.
        """
        request = "EnableRobot()"
        return self._send_and_receive(self.dashboard_socket, request)

    def clear_error(self):
        """
        Clears the controller error.
        """
        request = "ClearError()"
        return self._send_and_receive(self.dashboard_socket, request)

    def reset_robot(self):
        """
        Resets the robot.
        """
        request = "ResetRobot()"
        return self._send_and_receive(self.dashboard_socket, request)

    def get_robot_status(self):
        """
        Gets the robot status.
        """
        request = "RobotMode()"
        return self._send_and_receive(self.dashboard_socket, request)

    # Unused for now.
    def power_on(self):
        """
        Powers on the robot.

        Note: It takes about 10 seconds for the robot to be operational after it is powered on.
        """
        request = "PowerOn()"
        return self._send_and_receive(self.dashboard_socket, request)

    # Unused for now.
    def get_error_id(self):
        """
        Gets robot error code.
        """
        request = "GetErrorID()"
        return self._send_and_receive(self.dashboard_socket, request)

    # Unused for now.
    def get_pose(self):
        """
        Gets the current pose of the robot in the Cartesian coordinate system.
        """
        request = "GetPose()"
        return self._send_and_receive(self.dashboard_socket, request)

    def move_linear(self, target):
        """
        Moves the robot to the given target using linear motion.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        """
        request = "MovL(" + self.list_to_str(target) + ")"
        return self._send_and_receive(self.movement_socket, request)

    # Unused for now.
    def move_circular(self, waypoint, target):
        """
        Moves the robot to the given target via a waypoint using circular motion.

        :param: waypoint: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        """
        request = (
            "Arc(" + self.list_to_str(waypoint) + "," + self.list_to_str(target) + ")"
        )
        return self._send_and_receive(self.movement_socket, request)

    def move_servo(self, target):
        """
        Moves the robot to the given target using servo-based control.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        """
        request = "ServoP(" + self.list_to_str(target) + ")"
        return self._send_and_receive(self.movement_socket, request)

    def move_linear_relative_to_tool(self, offsets, tool):
        """
        Moves the robot with the given offsets in the tool coordinate system.

        The end motion mode is linear motion.

        :param: offsets: [x, y, z, rx, ry, rz], where x, y, z are
            the x-, y-, and z-offsets in mm and rx, ry, rz are the offsets for the
            rotation angles in degrees.
        tool: The selected tool, value range: 0-9
        """
        request = "RelMovLTool(" + self.list_to_str(offsets) + "," + str(tool) + ")"
        return self._send_and_receive(self.movement_socket, request)

    def set_speed_ratio(self, speed):
        """
        Setting the Global rate
        speed:Rate value(Value range:1~100)
        """
        speed = speed * 100
        request = "SpeedFactor(" + str(int(speed)) + ")"
        return self._send_and_receive(self.movement_socket, request)

    def enable_free_drive(self):
        """
        Start Drag Mode
        """
        request = "StartDrag()"
        return self._send_and_receive(self.dashboard_socket, request)

    def disable_free_drive(self):
        """
        Stop Drag Mode
        """
        request = "StopDrag()"
        return self._send_and_receive(self.dashboard_socket, request)


FeedbackType = np.dtype(
    [
        (
            "len",
            np.int64,
        ),
        (
            "digital_input_bits",
            np.uint64,
        ),
        (
            "digital_output_bits",
            np.uint64,
        ),
        (
            "robot_mode",
            np.uint64,
        ),
        (
            "time_stamp",
            np.uint64,
        ),
        (
            "time_stamp_reserve_bit",
            np.uint64,
        ),
        (
            "test_value",
            np.uint64,
        ),
        (
            "test_value_keep_bit",
            np.float64,
        ),
        (
            "speed_scaling",
            np.float64,
        ),
        (
            "linear_momentum_norm",
            np.float64,
        ),
        (
            "v_main",
            np.float64,
        ),
        (
            "v_robot",
            np.float64,
        ),
        (
            "i_robot",
            np.float64,
        ),
        (
            "i_robot_keep_bit1",
            np.float64,
        ),
        (
            "i_robot_keep_bit2",
            np.float64,
        ),
        ("tool_accelerometer_values", np.float64, (3,)),
        ("elbow_position", np.float64, (3,)),
        ("elbow_velocity", np.float64, (3,)),
        ("q_target", np.float64, (6,)),
        ("qd_target", np.float64, (6,)),
        ("qdd_target", np.float64, (6,)),
        ("i_target", np.float64, (6,)),
        ("m_target", np.float64, (6,)),
        ("q_actual", np.float64, (6,)),
        ("qd_actual", np.float64, (6,)),
        ("i_actual", np.float64, (6,)),
        ("actual_TCP_force", np.float64, (6,)),
        ("tool_vector_actual", np.float64, (6,)),
        ("TCP_speed_actual", np.float64, (6,)),
        ("TCP_force", np.float64, (6,)),
        ("Tool_vector_target", np.float64, (6,)),
        ("TCP_speed_target", np.float64, (6,)),
        ("motor_temperatures", np.float64, (6,)),
        ("joint_modes", np.float64, (6,)),
        ("v_actual", np.float64, (6,)),
        # ('dummy', np.float64, (9, 6))])
        ("hand_type", np.byte, (4,)),
        (
            "user",
            np.byte,
        ),
        (
            "tool",
            np.byte,
        ),
        (
            "run_queued_cmd",
            np.byte,
        ),
        (
            "pause_cmd_flag",
            np.byte,
        ),
        (
            "velocity_ratio",
            np.byte,
        ),
        (
            "acceleration_ratio",
            np.byte,
        ),
        (
            "jerk_ratio",
            np.byte,
        ),
        (
            "xyz_velocity_ratio",
            np.byte,
        ),
        (
            "r_velocity_ratio",
            np.byte,
        ),
        (
            "xyz_acceleration_ratio",
            np.byte,
        ),
        (
            "r_acceleration_ratio",
            np.byte,
        ),
        (
            "xyz_jerk_ratio",
            np.byte,
        ),
        (
            "r_jerk_ratio",
            np.byte,
        ),
        (
            "brake_status",
            np.byte,
        ),
        (
            "enable_status",
            np.byte,
        ),
        (
            "drag_status",
            np.byte,
        ),
        (
            "running_status",
            np.byte,
        ),
        (
            "error_status",
            np.byte,
        ),
        (
            "jog_status",
            np.byte,
        ),
        (
            "robot_type",
            np.byte,
        ),
        (
            "drag_button_signal",
            np.byte,
        ),
        (
            "enable_button_signal",
            np.byte,
        ),
        (
            "record_button_signal",
            np.byte,
        ),
        (
            "reappear_button_signal",
            np.byte,
        ),
        (
            "jaw_button_signal",
            np.byte,
        ),
        (
            "six_force_online",
            np.byte,
        ),
        ("reserve2", np.byte, (82,)),
        ("m_actual", np.float64, (6,)),
        (
            "load",
            np.float64,
        ),
        (
            "center_x",
            np.float64,
        ),
        (
            "center_y",
            np.float64,
        ),
        (
            "center_z",
            np.float64,
        ),
        ("user[6]", np.float64, (6,)),
        ("tool[6]", np.float64, (6,)),
        (
            "trace_index",
            np.float64,
        ),
        ("six_force_value", np.float64, (6,)),
        ("target_quaternion", np.float64, (4,)),
        ("actual_quaternion", np.float64, (4,)),
        ("reserve3", np.byte, (24,)),
    ]
)
