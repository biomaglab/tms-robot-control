from enum import Enum
from socket import socket, AF_INET, SOCK_STREAM


class MotionState(Enum):
    FREE_TO_MOVE = 0
    IN_MOTION = 1
    WAITING_FOR_EXECUTION = 2
    ERROR = 3
    UNKNOWN = 4

class ReferenceFrame(Enum):
    ROBOT = 0
    TOOL = 1


class ElfinConnection:
    PORT = 10003
    REQUEST_ENDING_CHARS = ",;"
    RESPONSE_LENGTH = 1024
    ROBOT_ID = 0

    def __init__(self, ip, use_new_api):
        """
        Class for low-level communication with Elfin robot.

        This class follows "HansRobot Communication Protocol Interface".
        """
        self.ip = ip
        self.use_new_api = use_new_api

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
            response = self.socket.recv(self.RESPONSE_LENGTH).decode('utf-8').split(',')

        except (BrokenPipeError, ConnectionResetError, TimeoutError) as e:
            print("Robot connection error: {}".format(e))
            self.connected = False
            return False, None

        if verbose:
            print("Done.")

        # Process the response.
        command = response[0]
        status = response[1]
        error_code = response[2]

        if status == 'OK':
            success = True

        elif status == 'Fail':
            print("The command {} returned the error code: {}".format(command, error_code))
            success = False

        else:
            print("Unknown status")
            success = False

        # XXX: Params returned by Elfin start from element 2 if the command was
        #   successful, otherwise element 2 is reserved for the error code. It would
        #   be cleaner if the params started always from element 3 and error code was 0
        #   if there is no error.
        fetch_params = type(response) != bool and len(response) > 3 and success

        # The elements 2...n-1 are the parameters, where n is the last element.
        params = response[2:-1] if fetch_params else None

        return success, params

    def list_to_str(self, list):
        """
        Converts a list of numbers to a string, with each number formatted to two decimal places.

        :param list: A list of numbers, e.g. [1, 2.333, 3].
        :return: A string representation of the list with two decimal places, e.g. "1.00,2.33,3.00".
        """
        return ",".join([f"{s:.2f}" for s in list])

    # Robot commands

    def stop_robot(self):
        """
        Stops the robot's movement.

        :return: True if successful, otherwise False.
        """
        request = "GrpStop," + str(self.ROBOT_ID)
        success, _ = self._send_and_receive(request, verbose=True)
        return success
    
    def enable_assistive_robot(self):
        """
        Enable assistive mode.

        :return: True if successful, otherwise False.
        """
        request = "StartAssistiveMode," + str(self.ROBOT_ID)
        success, _ = self._send_and_receive(request, verbose=True)
        return success
    
    def disable_assistive_robot(self):
        """
        Disable assistive mode.

        :return: True if successful, otherwise False.
        """
        request = "CloseAssistiveMode," + str(self.ROBOT_ID)
        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def set_speed_ratio(self, speed_ratio):
        """
        Sets the speed ratio.

        :param double speed_ratio: The desired speed ratio, range: 0.01-1.
        :return: True if successful, otherwise False.
        """
        request = "SetOverride," + str(self.ROBOT_ID) + ',' + str(speed_ratio)
        success, _ = self._send_and_receive(request)
        return success

    def get_pose(self):
        """
        Gets the pose of the robot TCP.

        :return: A pair of a success indicator and the current pose.

            The pose is a list [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.
        """
        command = "ReadActPos" if self.use_new_api else "ReadPcsActualPos"
        request = command + "," + str(self.ROBOT_ID)

        success, params = self._send_and_receive(request)
        if not success or params is None:
            coordinates = None
        else:
            coordinates = [float(s) for s in params[6:12]] if self.use_new_api else [float(s) for s in params]

        return success, coordinates

    def move_linear(self, target):
        """
        Moves the robot to the specified space coordinates using linear motion.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        :return: True if successful, otherwise False.
        """
        command = "MoveL" if self.use_new_api else "MoveB"
        request = command + "," + str(self.ROBOT_ID) + ',' + self.list_to_str(target)

        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def read_force_sensor(self):
        """
        Reads the state of the force sensor.

        :return: A pair of a success indicator and the force sensor values.

            The force sensor values are a list [Fx, Fy, Fz, Mx, My, Mz], where

            Fx, Fy, Fz are the forces in N, and
            Mx, My, Mz are the torques in Nm.
        """
        request = "ReadForceSensorData"
        success, params = self._send_and_receive(request)

        if success and params is not None:
            force_sensor_values = [float(s) for s in params]
        else:
            force_sensor_values = None

        return success, force_sensor_values

    def get_motion_state(self):
        """
        Gets the motion state of the robot.

        :return: A MotionState enum value, indicating the motion state of the robot.
        """
        command = "ReadRobotState" if self.use_new_api else "ReadMoveState"
        request = command + "," + str(self.ROBOT_ID)
        success, params = self._send_and_receive(request)

        if not success or params is None:
            print("Could not read robot motion state")
            return MotionState.ERROR

        if self.use_new_api:
            moving_state = bool(int(params[0]))
            error_state = bool(int(params[2]))

            if error_state:
                return MotionState.ERROR

            if moving_state:
                return MotionState.IN_MOTION

            return MotionState.FREE_TO_MOVE
        else:
            code = int(params[0])
            if code == 0:
                return MotionState.FREE_TO_MOVE
            elif code == 1009:
                return MotionState.IN_MOTION
            elif code == 1013:
                return MotionState.WAITING_FOR_EXECUTION
            elif code == 1025:
                return MotionState.ERROR
            else:
                print("Unknown motion state: {}".format(code))
                return MotionState.UNKNOWN

    def move_circular(self, start_position, waypoint, target):
        """
        Moves the robot to the specified space coordinates using circular motion.

        :param: start_position: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm and rx, ry, rz are the rotation angles in degrees.
        :param: waypoint: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm and rx, ry, rz are the rotation angles in degrees.
        :param: target: [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.

        :return: True if successful, otherwise False.
        """

        if self.use_new_api:
            # XXX: The velocity is set to 300 and the acceleration to 2500. It turns out that the units specified in the API manual
            #   (mm/s and mm/s^2, respectively) are most likely incorrect. It is uncertain what is the actual unit, but these values
            #   seem to work well enough.
            request = "MoveC," + str(self.ROBOT_ID) + ',' + self.list_to_str(start_position) + ',' + self.list_to_str(waypoint) + \
                    ',' + self.list_to_str(target) + ',0,1,0,300,2500,1,TCP,Base,0'
        else:
            # Always use movement type 0.
            movement_type_str = '0'

            # Note: The start position is unused in the old version of the Elfin API.
            request = "MoveC," + str(self.ROBOT_ID) + ',' + self.list_to_str(waypoint[:3]) + ',' + self.list_to_str(target) + ',' + movement_type_str

        success, _ = self._send_and_receive(request, verbose=True)
        return success
