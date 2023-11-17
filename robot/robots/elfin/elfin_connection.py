from socket import socket, AF_INET, SOCK_STREAM

from robot.robots.elfin.motion_state import MotionState


class ElfinConnection:
    MESSAGE_ENDING_CHARS = ",;"
    MESSAGE_SIZE = 1024
    ROBOT_ID = 0

    def __init__(self, ip, port):
        """
        Class for low-level communication with Elfin robot.

        This class follows "HansRobot Communication Protocol Interface".
        """
        self.ip = ip
        self.port = port
        self.connected = False

    def connect(self):
        try:
            mySocket = socket(AF_INET, SOCK_STREAM)
            mySocket.connect((self.ip, self.port))

            self.mySocket = mySocket

            self.connected = True

        except:
            print("Failed to connect")

    def send(self, message):
        message_with_ending = message + self.MESSAGE_ENDING_CHARS
        encoded_message = message_with_ending.encode('utf-8')
        self.mySocket.sendall(encoded_message)
        try:
            data = self.mySocket.recv(self.MESSAGE_SIZE).decode('utf-8').split(',')
        except TimeoutError:
            print("Robot connection error: TimeoutError")
            self.connected = False
            return False

        status = self.check_status(data)
        if status and type(data) != bool:
            if len(data) > 3:
                return data[2:-1]
        return status

    def check_status(self, recv_message):
        status = recv_message[1]
        if status == 'OK':
            return True

        elif status == 'Fail':
            print("The message {} is returning the error code: {}".format(recv_message[0], recv_message[2]))
            return False

    def PowerUp(self):
        """
        Powers up the robot.
        Note: Waits until powered up. Power up time is about 44s.

        :return: True if successful, otherwise False.
        """
        message = "Electrify"
        status = self.send(message)
        return status

    def PowerOutage(self):
        """
        Creates a power outage for the robot.
        Note: Waits until power outage is over (3 seconds).

        :return: True if successful, otherwise False.
        """
        message = "BlackOut"
        status = self.send(message)
        return status

    def StartMasterStation(self):
        """
        Starts the master station.
        Note: Waits until the master station is started (approximately 4 seconds).

        :return: True if successful, otherwise False.
        """
        message = "StartMaster"
        status = self.send(message)
        return status

    def StopMasterStation(self):
        """
        Stops the master station.
        Note: Waits until the master station is stopped (approximately 2 seconds).

        :return: True if successful, otherwise False.
        """
        message = "CloseMaster"
        status = self.send(message)
        return status

    def EnableRobotServo(self):
        """
        Enables the robot's servo.

        :return: True if successful, otherwise False.
        """
        message = "GrpPowerOn," + str(self.ROBOT_ID)
        status = self.send(message)
        return status

    def DisableRobotServo(self):
        """
        Disables the robot's servo.

        :return: True if successful, otherwise False.
        """
        message = "GrpPowerOff," + str(self.ROBOT_ID)
        status = self.send(message)
        return status

    def StopRobot(self):
        """
        Stops the robot's movement.

        :return: True if successful, otherwise False.
        """
        message = "GrpStop," + str(self.ROBOT_ID)
        status = self.send(message)
        return status

    def SetSpeedRatio(self, speed_ratio):
        """
        Sets the speed ratio.

        :param double speed_ratio: The desired speed ratio, range: 0.01-1.
        :return: True if successful, otherwise False.
        """
        message = "SetOverride," + str(self.ROBOT_ID) + ',' + str(speed_ratio)
        status = self.send(message)
        return status

    def GetCoordinates(self):
        """
        Gets the space coordinates of the robot.

        :return: Return x, y, z, rx, ry, rz, where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.

            If unsuccessful, return False.
        """
        message = "ReadPcsActualPos," + str(self.ROBOT_ID)
        coord = self.send(message)
        if coord:
            return [float(s) for s in coord]

        return coord

    def MoveLinear(self, target):
        """
        Moves the robot to the specified space coordinates using linear motion.

        :param: target: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm
            and rx, ry, rz are the rotation angles in degrees.
        :return: True if successful, otherwise False.
        """
        target = [str(s) for s in target]
        target = (",".join(target))
        message = "MoveL," + str(self.ROBOT_ID) + ',' + target
        return self.send(message)

    def MoveLinearRelative(self, distance):
        """"
        Moves the robot a given distance from the specified spatial coordinate directional.

        TODO: This description could be clarified further.

        Note: There is a singular point in space motion.

        TODO: This note could be clarified. How does the singularity affect this function?

        :param: distance: [directionID; direction (0:negative, 1:positive); distance]
        :return: True if successful, otherwise False.
        """
        distance = [str(s) for s in distance]
        distance = (",".join(distance))
        message = "MoveRelL," + str(self.ROBOT_ID) + ',' + distance
        self.send(message)

    def ReadForceSensor(self):
        """
        Reads the state of the force sensor.

        :return: Fx, Fy, Fz, Mx, My, Mz, where

            Fx, Fy, Fz are the forces in N, and
            Mx, My, Mz are the torques in Nm.

            If unsuccessful, returns a list of zeros.
        """
        message = "ReadForceSensorData"
        status = self.send(message)
        if status:
            return [float(s) for s in status]
        return [0]*6

    def SetToolCoordinateMotion(self, state):
        """
        Sets the tool coordinate motion.

        TODO: This description could be clarified further. Also, the function naming could
          be improved. It's not clear from the name what it does.

        :param: int state: 0 = close, 1 = open.
        :return: True if successful, otherwise False.
        """
        message = "SetToolCoordinateMotion," + str(self.ROBOT_ID) + ',' + str(status)
        status = self.send(message)
        return status

    def GetMotionState(self):
        """
        Gets the motion state of the robot.

        :return: A MotionState enum value, indicating the motion state of the robot.
        """
        message = "ReadMoveState," + str(self.ROBOT_ID)
        readmovestate = self.send(message)
        if not readmovestate:
            print("Could not read robot motion state")
            return MotionState.ERROR

        code = int(readmovestate[0])
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
        message = "MoveHoming," + str(self.ROBOT_ID)
        status = self.send(message)
        return status

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
        # Note: The start position is unused in this version of the Elfin API.

        waypoint_str = [str(s) for s in waypoint]
        waypoint_str = ",".join(waypoint_str)

        target_str = [str(s) for s in target]
        target_str = ",".join(target_str)

        # Always use movement type 0.
        movement_type_str = '0'

        message = "MoveC," + str(self.ROBOT_ID) + ',' + waypoint_str + ',' + target_str + ',' + movement_type_str
        return self.send(message)

    def MoveLinearWithWaypoint(self, waypoint, target):
        """
        Moves the robot to the specified space coordinates through a waypoint, using linear motion.

        :param: waypoint: [x, y, z], where x, y, z are the coordinates in mm.
        :param: target: [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.

        :return: True if successful, otherwise False.
        """
        waypoint_str = [str(s) for s in waypoint_str]
        waypoint_str = ",".join(waypoint_str)

        target_str = [str(s) for s in target_str]
        target_str = ",".join(target_str)

        message = "MoveB," + str(self.ROBOT_ID) + ',' + waypoint_str + ',' + target_str
        return self.send(message)
