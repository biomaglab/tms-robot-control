from socket import socket, AF_INET, SOCK_STREAM


# TODO: The naming for this class could be improved to be descriptive of how it
#   differs from ElfinConnection.
class ElfinConnectionLinux:
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

        :return: If successful, return x, y, z, rx, ry, rz, where x, y, z are the
            coordinates in mm and rx, ry, rz are the rotation angles in degrees.
            If unsuccessful, return False.
        """
        message = "ReadActPos," + str(self.ROBOT_ID)
        coord = self.send(message)
        if coord:
            return [float(s) for s in coord][6:12]

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

        :return: Fx, Fy, Fz, Mx, My, Mz, where Fx, Fy, Fz are the forces in N and
            Mx, My, Mz are the torques in Nm. If unsuccessful, returns a list of
            zeros.
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
        message = "SetToolMotion," + str(self.ROBOT_ID) + ',' + str(status)
        status = self.send(message)
        return status

    def GetMotionState(self):
        """
        Gets the motion state of the robot.

        :return:
            Current state of motion of robot:
            0=motion completion;
            1009=in motion;
            1013=waiting for execution;
            1025 =Error reporting
        """
        message = "ReadRobotState," + str(self.ROBOT_ID)
        read_robot_state = self.send(message)
        moving_state = bool(read_robot_state[0])
        error_state = bool(read_robot_state[2])
        if error_state:
            #ErrorState
            return const.ROBOT_ELFIN_MOVE_STATE["error"]
        if moving_state:
            #robot is moving
            return const.ROBOT_ELFIN_MOVE_STATE["in motion"]
        # robot is not moving
        return const.ROBOT_ELFIN_MOVE_STATE["free to move"]

    def HomeRobot(self):
        """
        Homes the robot, that is, returns the robot to the origin.

        :return: True if successful, otherwise False.
        """
        message = "MoveHoming," + str(self.ROBOT_ID)
        status = self.send(message)
        return status

    def MoveCircular(self, target):
        """
        Moves the robot to the specified space coordinates using circular motion.

        TODO: Improve parameter description.
        :param: Through position[X,Y,Z],GoalCoord[X,Y,Z,RX,RY,RZ],Type[0 or 1],;
        :return: True if successful, otherwise False.
        """
        start_position, middle_position, final_target = target

        start_position = [str(s) for s in start_position]
        middle_position = [str(s) for s in middle_position]
        final_target = [str(s) for s in final_target]

        start_position = (",".join(start_position))
        middle_position = (",".join(middle_position))
        final_target = (",".join(final_target))

        message = "MoveC," + str(self.ROBOT_ID) + ',' + start_position + ',' + middle_position + ',' + final_target + \
                  ',0,0,1,10,10,1,TCP,Base,0'
        # FixedPosure,nMoveCType,dRadLen,dVelocity,dAcc,dRadius,sTcpName,sUcsName,strCmdID
        return self.send(message)

    def MoveLinearWithWaypoint(self, target):
        """
        Moves the robot to the specified space coordinates through a waypoint, using linear motion.

        TODO: Improve parameter description.
        :param: Through position[X,Y,Z],GoalCoord[X,Y,Z,RX,RY,RZ],Type[0 or 1],;
        :return:
        """
        target = [str(s) for s in target]
        target = (",".join(target))
        message = "MoveB," + str(self.ROBOT_ID) + ',' + target
        return self.send(message)
