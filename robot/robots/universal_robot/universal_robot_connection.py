from enum import Enum
from socket import socket, AF_INET, SOCK_STREAM


class UniversalRobotConnection:
    PORT = 30011
    ip = "192.168.5.5"
    REQUEST_ENDING_CHARS = "\n"
    RESPONSE_LENGTH = 1024

    def __init__(self, ip):
        """
        Class for low-level communication with Universal robot.
        """
        self.ip = ip

        self.connected = False
        self.socket = None

    def connect(self):
        """
        Connects to the robot.

        :return: True if successful, otherwise False.
        """
        if self.connected:
            print("Already connected to the robot")
            return True

        try:
            new_socket = socket(AF_INET, SOCK_STREAM)
            new_socket.connect((self.ip, self.PORT))

            self.socket = new_socket

            self.connected = True
        except:
            print("Failed to connect to the robot")

        return self.connected

    def disconnect(self):
        """
        Disconnects from the robot.

        :return: True if successful, otherwise False.
        """
        success = False

        if not self.connected:
            print("Not connected to the robot, therefore cannot disconnect")
            return success

        try:
            self.socket.close()
            self.connected = False
            success = True
        except:
            print("Failed to disconnect from the robot")

        return success

    def _send_and_receive(self, request, verbose=False):
        if verbose:
            print("Sending request: {}".format(request))

        full_request = request + self.REQUEST_ENDING_CHARS
        try:
            # Send the request to the robot.
            self.socket.sendall(full_request.encode('utf-8'))

            # Receive the response from the robot.
            response = self.socket.recv(self.RESPONSE_LENGTH)
            print(response)
            return response, None
            response = response.decode('utf-8').split(',')

        except (BrokenPipeError, ConnectionResetError, TimeoutError) as e:
            print("Robot connection error: {}".format(e))
            self.connected = False
            return False, None

        if verbose:
            print("Done.")

        return True, response

    def list_to_str(self, listt):
        """
        Converts a list of numbers to a string.

        :param: list: A list of numbers, e.g. [1,2,3].
        :return: A string representation of the list, e.g. "1,2,3".
        """
        return ",".join([str(s) for s in listt])


    # Robot commands

    def stop_robot(self, a):
        """
        Stops the robot's movement.

        :return: True if successful, otherwise False.
        """
        request = "stopl" + '(' + str(a) + ')'
        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def move_linear(self, target, a, v, t, r):
        """
        Moves the robot to the specified space coordinates using linear motion.

        :params:
            • target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in [mm]
                and rx, ry, rz are the rotation angles in [degree].
            • a: tool acceleration [m/s^2]
            • v: tool speed [m/s]
            • t: time [S] to make the move. If it were specified the command would ignore the a and v values.
            • r: blend radius [m]
        :return: True if successful, otherwise False.
        """
        # command = "MoveL" if self.use_new_api else "MoveB"
        request = "movel([" + self.list_to_str(target) + '],a=' + str(a) +\
            ',v=' + str(v) + ',t=' + str(t) + ',r=' + str(r) + ')'

        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def move_circular(self, waypoint, target, a, v, r, mode):
        """
        Moves the robot to the specified space coordinates using circular motion.

        :params:
            • waypoint: [x, y, z, rx, ry, rz], where x, y, z are the coordinates [mm],
                and rx, ry, rz are the rotation angles [deg].
                • Note: Rotations are not used so they can be left as zeros.
                • Note: This position can also be represented as joint angles [j0,j1,j2,j3,j4,j5]
                    then forward kinematics is used to calculate the corresponding pose
            • target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates [mm],
                and rx, ry, rz are the rotation angles [deg].
            • a: tool acceleration [m/s^2]
            • v: tool speed [m/s]
            • r: blend radius (of target pose) [m]
            • mode:
                0: Unconstrained mode. Interpolate orientation from current pose to target pose (pose_to)
                1: Fixed mode. Keep orientation constant relative to the tangent of the circular arc (starting
                    from current pose)

        :return: True if successful, otherwise False.
        """
        request = "movec([" + self.list_to_str(waypoint) + '],[' + self.list_to_str(target) +\
            '],a=' + str(a) + ',v=' + str(v) + ',r=' + str(r) + ',mode=' + self.str(mode) + ')'

        success, _ = self._send_and_receive(request, verbose=True)
        return success
