from socket import socket, AF_INET, SOCK_STREAM


# TODO: What is this, how does it differ, why does it exist? The naming should be improved.
class ElfinConnectionLinux:
    def __init__(self):
        """
        Class for low-level communication with Elfin robot.

        This class follows "HansRobot Communication Protocol Interface".
        """
        self.connected = False
        self.end_msg = ",;"

    def connect(self, ip, port, message_size, robot_id):
        try:
            mySocket = socket(AF_INET, SOCK_STREAM)
            mySocket.connect((ip, port))

            self.ip = ip
            self.port = port
            self.message_size = message_size
            self.robot_id = str(robot_id)
            self.mySocket = mySocket

            self.connected = True

        except:
            print("Failed to connect")

    def send(self, message):
        self.mySocket.sendall(message.encode('utf-8'))
        try:
            data = self.mySocket.recv(self.message_size).decode('utf-8').split(',')
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

    def Electrify(self):
        """
        Function: Power the robot
        Notes: successful completion of power up before returning, power up time is
        about 44s.
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "Electrify" + self.end_msg
        status = self.send(message)
        return status

    def BlackOut(self):
        """
        Function: Robot blackout
        Notes: successful power outage will only return, power failure time is 3s.
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "BlackOut" + self.end_msg
        status = self.send(message)
        return status

    def StartMaster(self):
        """
        Function: Start master station
        Notes: the master station will not be returned until successfully started, startup
        master time is about 4s.
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "StartMaster" + self.end_msg
        status = self.send(message)
        return status

    def CloseMaster(self):
        """
        Function: Close master station
        Notes: the master station will not be returned until successfully closed, shut
        down the master station time is about 2s.
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "CloseMaster" + self.end_msg
        status = self.send(message)
        return status

    def GrpPowerOn(self):
        """
        Function: Robot servo on
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "GrpPowerOn," + self.robot_id + self.end_msg
        status = self.send(message)
        return status

    def GrpPowerOff(self):
        """
        Function: Robot servo off
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "GrpPowerOff," + self.robot_id + self.end_msg
        status = self.send(message)
        return status

    def GrpStop(self):
        """
        Function: Stop robot
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "GrpStop," + self.robot_id + self.end_msg
        status = self.send(message)
        return status

    def SetOverride(self, override):
        """
        function: Set speed ratio
        :param override:
            double: set speed ratio, range of 0.01~1
        :return:
        if Error Return False
            if not Error Return True
        """

        message = "SetOverride," + self.robot_id + ',' + str(override) + self.end_msg
        status = self.send(message)
        return status

    def ReadPcsActualPos(self):
        """Function: Get the actual position of the space coordinate
        :return:
            if True Return x,y,z,a,b,c
            if Error Return False
        """
        message = "ReadActPos," + self.robot_id + self.end_msg
        coord = self.send(message)
        if coord:
            return [float(s) for s in coord][6:12]

        return coord

    def MoveL(self, target):
        """
        function: Robot moves straight to the specified space coordinates
        :param: target:[X,Y,Z,RX,RY,RZ]
        :return:
        """
        target = [str(s) for s in target]
        target = (",".join(target))
        message = "MoveL," + self.robot_id + ',' + target + self.end_msg
        return self.send(message)

    def MoveRelL(self, distance):
        """"
        Function: Robot moves a certain distance from the specified spatial coordinate directional
        Note: there is a singular point in space motion
        :param: target:[directionID; direction (0:negative, 1:positive); distance]
        :return:
        """
        distance = [str(s) for s in distance]
        distance = (",".join(distance))
        message = "MoveRelL," + self.robot_id + ',' + distance + self.end_msg
        self.send(message)

    def ReadForceSensorData(self):
        """Function: Read force sensor data
        :return: ” ReadForceSensorData, OK, Fx, Fy, Fz, Mx, My, Mz,;”
        Fail
        :return: ”ReadForceSensorData, Fail, ErrorCode,;”
        """
        message = "ReadForceSensorData" + self.end_msg
        status = self.send(message)
        if status:
            return [float(s) for s in status]
        #print("Error code: ", status)
        return [0]*6

    def SetToolCoordinateMotion(self, status):
        """
        function: Function: Set tool coordinate motion
        :param: int Switch 0=close 1=open
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "SetToolMotion," + self.robot_id + ',' + str(status) + self.end_msg
        status = self.send(message)
        return status

    def ReadMoveState(self):
        """
        Function: Get the motion state of the robot
        :return:
            Current state of motion of robot:
            0=motion completion;
            1009=in motion;
            1013=waiting for execution;
            1025 =Error reporting
        """
        message = "ReadRobotState," + self.robot_id + self.end_msg
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


    def MoveHoming(self):
        """
        Function: Robot returns to the origin
        :return:
            if Error Return False
            if not Error Return True
        """
        message = "MoveHoming," + self.robot_id + self.end_msg
        status = self.send(message)
        return status

    def MoveC(self, target):
        """
        function: Arc motion
        :param: Through position[X,Y,Z],GoalCoord[X,Y,Z,RX,RY,RZ],Type[0 or 1],;
        :return:
        """
        start_position, middle_position, final_target = target

        start_position = [str(s) for s in start_position]
        middle_position = [str(s) for s in middle_position]
        final_target = [str(s) for s in final_target]

        start_position = (",".join(start_position))
        middle_position = (",".join(middle_position))
        final_target = (",".join(final_target))

        message = "MoveC," + self.robot_id + ',' + start_position + ',' + middle_position + ',' + final_target + \
                  ',0,0,1,10,10,1,TCP,Base,0' + self.end_msg
        # FixedPosure,nMoveCType,dRadLen,dVelocity,dAcc,dRadius,sTcpName,sUcsName,strCmdID
        return self.send(message)

    def MoveB(self, target):
        """
        function: Linear motion.
        :param: Through position[X,Y,Z],GoalCoord[X,Y,Z,RX,RY,RZ],Type[0 or 1],;
        :return:
        """
        target = [str(s) for s in target]
        target = (",".join(target))
        message = "MoveB," + self.robot_id + ',' + target + self.end_msg
        return self.send(message)