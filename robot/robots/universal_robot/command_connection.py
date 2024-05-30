from enum import Enum
from socket import socket, AF_INET, SOCK_STREAM


class CommandConnection:
    PORT = 30002
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

    def _send(self, request, verbose=False):
        if verbose:
            print("Sending request: {}".format(request))

        full_request = request + self.REQUEST_ENDING_CHARS
        try:
            # Send the request to the robot.
            self.socket.sendall(full_request.encode('utf-8'))

        except (BrokenPipeError, ConnectionResetError, TimeoutError) as e:
            print("Robot connection error: {}".format(e))
            self.connected = False
            return False

        if verbose:
            print("Done.")

        return True

    def list_to_str(self, listt):
        """
        Converts a list of numbers to a string.

        :param: list: A list of numbers, e.g. [1,2,3].
        :return: A string representation of the list, e.g. "1,2,3".
        """
        return ",".join([str(s) for s in listt])


    # Robot commands

    def stop_robot(self, deceleration):
        """
        Stops the robot's movement.

        :params:
            • deceleration: tool deceleration [m/s^2]

        :return: True if successful, otherwise False.
        """
        request = "stopl({})".format(deceleration)

        success = self._send(request, verbose=True)
        return success

    def move_linear(self, target, acceleration, velocity, time, radius):
        """
        Moves the robot to the specified space coordinates using linear motion.

        :params:
            • target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in [mm]
                and rx, ry, rz are the rotation angles in [degree].
            • acceleration: tool acceleration [m/s^2]
            • velocity: tool speed [m/s]
            • time: time [S] to make the move. If it were specified the command would ignore the a and v values.
            • radius: blend radius [m]
        :return: True if successful, otherwise False.
        """
        target_str = "p[{}]".format(self.list_to_str(target))

        request = "movel({}, a={}, v={}, t={}, r={})".format(
            target_str,
            acceleration,
            velocity,
            time,
            radius,
        )
        return self._send(request, verbose=True)

    def move_circular(self, waypoint, target, acceleration, velocity, radius, mode):
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
            • acceleration: tool acceleration [m/s^2]
            • velocity: tool speed [m/s]
            • radius: blend radius (of target pose) [m]
            • mode:
                0: Unconstrained mode. Interpolate orientation from current pose to target pose (pose_to)
                1: Fixed mode. Keep orientation constant relative to the tangent of the circular arc (starting
                    from current pose)

        :return: True if successful, otherwise False.
        """
        waypoint_str = "p[{}]".format(self.list_to_str(waypoint))
        target_str = "p[{}]".format(self.list_to_str(target))

        request = "movec({},{},a={},v={},r={},mode={})".format(
            waypoint_str,
            target_str,
            acceleration,
            velocity,
            radius,
            mode,
        )
        return self._send(request, verbose=True)
