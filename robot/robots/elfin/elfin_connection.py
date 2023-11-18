from socket import socket, AF_INET, SOCK_STREAM

from robot.robots.elfin.motion_state import MotionState


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

    def connect(self):
        try:
            new_socket = socket(AF_INET, SOCK_STREAM)
            new_socket.connect((self.ip, self.PORT))

            self.socket = new_socket

            self.connected = True

        except:
            print("Failed to connect")

    def send_and_receive(self, request):
        # Send the request to the robot.
        full_request = request + self.REQUEST_ENDING_CHARS
        self.socket.sendall(full_request.encode('utf-8'))

        # Receive the response from the robot.
        try:
            response = self.socket.recv(self.RESPONSE_LENGTH).decode('utf-8').split(',')
        except TimeoutError:
            print("Robot connection error: TimeoutError")
            self.connected = False
            return False, None

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
        Converts a list of numbers to a string.

        :param: list: A list of numbers, e.g. [1,2,3].
        :return: A string representation of the list, e.g. "1,2,3".
        """
        return ",".join([str(s) for s in list])

    # Robot commands

    def PowerUp(self):
        """
        Powers up the robot.
        Note: Waits until powered up. Power up time is about 44s.

        :return: True if successful, otherwise False.
        """
        request = "Electrify"
        success, _ = self.send_and_receive(request)
        return success

    def PowerOutage(self):
        """
        Creates a power outage for the robot.
        Note: Waits until power outage is over (3 seconds).

        :return: True if successful, otherwise False.
        """
        request = "BlackOut"
        success, _ = self.send_and_receive(request)
        return success

    def StartMasterStation(self):
        """
        Starts the master station.
        Note: Waits until the master station is started (approximately 4 seconds).

        :return: True if successful, otherwise False.
        """
        request = "StartMaster"
        success, _ = self.send_and_receive(request)
        return success

    def StopMasterStation(self):
        """
        Stops the master station.
        Note: Waits until the master station is stopped (approximately 2 seconds).

        :return: True if successful, otherwise False.
        """
        request = "CloseMaster"
        success, _ = self.send_and_receive(request)
        return success

    def EnableRobotServo(self):
        """
        Enables the robot's servo.

        :return: True if successful, otherwise False.
        """
        request = "GrpPowerOn," + str(self.ROBOT_ID)
        success, _ = self.send_and_receive(request)
        return success

    def DisableRobotServo(self):
        """
        Disables the robot's servo.

        :return: True if successful, otherwise False.
        """
        request = "GrpPowerOff," + str(self.ROBOT_ID)
        success, _ = self.send_and_receive(request)
        return success

    def StopRobot(self):
        """
        Stops the robot's movement.

        :return: True if successful, otherwise False.
        """
        request = "GrpStop," + str(self.ROBOT_ID)
        success, _ = self.send_and_receive(request)
        return success

    def SetSpeedRatio(self, speed_ratio):
        """
        Sets the speed ratio.

        :param double speed_ratio: The desired speed ratio, range: 0.01-1.
        :return: True if successful, otherwise False.
        """
        request = "SetOverride," + str(self.ROBOT_ID) + ',' + str(speed_ratio)
        success, _ = self.send_and_receive(request)
        return success

    def GetCoordinates(self):
        """
        Gets the space coordinates of the robot.

        :return: A pair of a success indicator and the current coordinates.

            The coordinates are a list [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.
        """
        command = "ReadActPos" if self.use_new_api else "ReadPcsActualPos"
        request = command + "," + str(self.ROBOT_ID)

        success, params = self.send_and_receive(request)
        if not success or params is None:
            coordinates = None
        else:
            coordinates = [float(s) for s in params[6:12]] if self.use_new_api else [float(s) for s in params]

        return success, coordinates

    def MoveLinear(self, target):
        """
        Moves the robot to the specified space coordinates using linear motion.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        :return: True if successful, otherwise False.
        """
        command = "MoveL" if self.use_new_api else "MoveB"
        request = command + "," + str(self.ROBOT_ID) + ',' + self.list_to_str(target)

        success, _ = self.send_and_receive(request)
        return success

    def MoveLinearRelative(self, axis, direction, distance):
        """"
        Moves the robot a given distance along the specified coordinate axis.

        Note: There is a singular point in space motion.

        TODO: This note could be clarified. How does the singularity affect this function?

        :param: axis: 0-5, corresponding to x-, y-, z-, rx-, ry-, rz-axes, respectively.
        :param: direction: 0 for negative, 1 for positive direction along the axis.
        :param: distance: The distance to move (TODO: unit?)
        :return: True if successful, otherwise False.
        """
        request = "MoveRelL," + str(self.ROBOT_ID) + ',' + \
            str(axis) + ',' + str(direction) + ',' + self.list_to_str(distance)

        success, _ = self.send_and_receive(request)
        return success

    def ReadForceSensor(self):
        """
        Reads the state of the force sensor.

        :return: Fx, Fy, Fz, Mx, My, Mz, where

            Fx, Fy, Fz are the forces in N, and
            Mx, My, Mz are the torques in Nm.

            If unsuccessful, returns a list of zeros.
        """
        request = "ReadForceSensorData"
        success, params = self.send_and_receive(request)
        if params:
            return [float(s) for s in params]
        return [0]*6

    def SetToolCoordinateMotion(self, state):
        """
        Sets the tool coordinate motion.

        TODO: This description could be clarified further. Also, the function naming could
          be improved. It's not clear from the name what it does.

        :param: int state: 0 = close, 1 = open.
        :return: True if successful, otherwise False.
        """
        command = "SetToolMotion" if self.use_new_api else "SetToolCoordinateMotion"
        request = command + "," + str(self.ROBOT_ID) + ',' + str(state)

        success, _ = self.send_and_receive(request)
        return success

    def GetMotionState(self):
        """
        Gets the motion state of the robot.

        :return: A MotionState enum value, indicating the motion state of the robot.
        """
        command = "ReadRobotState" if self.use_new_api else "ReadMoveState"
        request = command + "," + str(self.ROBOT_ID)
        success, params = self.send_and_receive(request)

        if not success or params is None:
            print("Could not read robot motion state")
            return MotionState.ERROR

        if self.use_new_api:
            moving_state = bool(params[0])
            error_state = bool(params[2])

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

    def HomeRobot(self):
        """
        Homes the robot (= returns the robot to the origin).

        :return: True if successful, otherwise False.
        """
        request = "MoveHoming," + str(self.ROBOT_ID)
        success, _ = self.send_and_receive(request)
        return success

    def MoveCircular(self, start_position, waypoint, target):
        """
        Moves the robot to the specified space coordinates using circular motion.

        :param: start_position: [x, y, z], where x, y, z are the coordinates in mm.
        :param: waypoint: [x, y, z], where x, y, z are the coordinates in mm.
        :param: target: [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.

        :return: True if successful, otherwise False.
        """
        # Always use movement type 0.
        movement_type_str = '0'

        if self.use_new_api:
            request = "MoveC," + str(self.ROBOT_ID) + ',' + self.list_to_str(start_position) + ',' + self.list_to_str(waypoint) + \
                    ',' + self.list_to_str(target) + ',' + movement_type_str + ',0,1,10,10,1,TCP,Base,0'
        else:
            # Note: The start position is unused in the old version of the Elfin API.
            request = "MoveC," + str(self.ROBOT_ID) + ',' + self.list_to_str(waypoint) + ',' + self.list_to_str(target) + ',' + movement_type_str

        success, _ = self.send_and_receive(request)
        return success

    def MoveLinearWithWaypoint(self, waypoint, target):
        """
        Moves the robot to the specified space coordinates through a waypoint, using linear motion.

        :param: waypoint: [x, y, z], where x, y, z are the coordinates in mm.
        :param: target: [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.

        :return: True if successful, otherwise False.
        """
        request = "MoveB," + str(self.ROBOT_ID) + ',' + self.list_to_str(waypoint) + ',' + self.list_to_str(target)

        success, _ = self.send_and_receive(request)
        return success
