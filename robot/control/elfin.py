
from time import sleep
from socket import socket, AF_INET, SOCK_STREAM
import robot.constants as const

class Elfin_Server():
    """
    This class is similar to tracker devices wrappers.
    It follows the same functions as the others (Initialize, Run and Close)
    """
    def __init__(self, server_ip, port_number, remote_control):
        self.server_ip = server_ip
        self.port_number = port_number
        self.remote_control = remote_control
        self.coordinate = [None]*6

    def Initialize(self):
        message_size = 1024
        robot_id = 0
        self.cobot = Elfin(self.remote_control)
        status_connection = self.cobot.connect(self.server_ip, self.port_number, message_size, robot_id)
        return status_connection

    def Run(self):
        coord = self.cobot.ReadPcsActualPos()
        if coord:
            self.coordinate = coord
        return self.coordinate

    def SendCoordinatesControl(self, target, motion_type=const.ROBOT_MOTIONS["normal"]):
        """
        It's not possible to send a move command to elfin if the robot is during a move.
         Status 1009 means robot in motion.
        """
        status = self.cobot.ReadMoveState()
        if motion_type == const.ROBOT_MOTIONS["normal"] or motion_type == const.ROBOT_MOTIONS["linear out"]:
            self.cobot.MoveB(target)
        elif motion_type == const.ROBOT_MOTIONS["arc"]:
            if status == const.ROBOT_MOVE_STATE["free to move"]:
                self.cobot.MoveC(target)
            elif status == const.ROBOT_MOVE_STATE["error"]:
                self.StopRobot()

    def GetForceSensorData(self):
        if const.FORCE_TORQUE_SENSOR:
            return self.cobot.ReadForceSensorData()[2]
        else:
            return False

    def CompensateForce(self, flag):
        status = self.cobot.ReadMoveState()
        if status == const.ROBOT_MOVE_STATE["free to move"]:
            if not flag:
                self.StopRobot()
            self.cobot.SetToolCoordinateMotion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
            #self.cobot.SetOverride(0.1)  # Setting robot's movement speed
            CompenDistance = [2, 0, 1]  # [directionID; direction (0:negative, 1:positive); distance]
            self.cobot.MoveRelL(CompenDistance)  # Robot moves in specified spatial coordinate directional
            self.cobot.SetToolCoordinateMotion(0)

    def TuneTarget(self, distance_to_target):
        status = self.cobot.ReadMoveState()
        if status == const.ROBOT_MOVE_STATE["free to move"]:
            self.cobot.SetToolCoordinateMotion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
            #self.cobot.SetOverride(0.1)  # Setting robot's movement speed
            abs_distance_to_target = [abs(x) for x in distance_to_target]
            direction = abs_distance_to_target.index(max(abs_distance_to_target))
            CompenDistance = [direction, 1, distance_to_target[direction]]
            self.cobot.MoveRelL(CompenDistance)
            self.cobot.SetToolCoordinateMotion(0)

    def StopRobot(self):
        # Takes some microseconds to the robot actual stops after the command.
        # The sleep time is required to guarantee the stop
        self.cobot.GrpStop()
        sleep(0.05)

    def Close(self):
        self.StopRobot()
        #TODO: robot function to close? self.cobot.close()

class Elfin:
    def __init__(self, remote_control):
        """
        Class to communicate with elfin robot. This class follows "HansRobot Communication Protocol Interface".
        """
        self.remote_control = remote_control
        self.end_msg = ",;"

    def connect(self, server_ip, port_number, message_size, robot_id):
        try:
            mySocket = socket(AF_INET, SOCK_STREAM)
            mySocket.connect((server_ip, port_number))

            self.server_ip = server_ip
            self.port_number = port_number
            self.message_size = message_size
            self.robot_id = str(robot_id)
            self.mySocket = mySocket

            return True

        except:
            return False

    def reconnect(self):
        topic = 'Update robot status'
        data = {'robot_status': False}
        self.remote_control.send_message(topic, data)
        print("Trying to reconnect to robot...")
        while self.connect(self.server_ip, self.port_number, self.message_size, self.robot_id) is False:
            sleep(1)
            print("Trying to reconnect to robot...")
        self.GrpStop()
        sleep(0.1)
        print("Reconnect!")

    def send(self, message):
        self.mySocket.sendall(message.encode('utf-8'))
        try:
            data = self.mySocket.recv(self.message_size).decode('utf-8').split(',')
        except TimeoutError:
            print("Robot connection error: TimeoutError")
            self.reconnect()
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
        message = "ReadPcsActualPos," + self.robot_id + self.end_msg
        coord = self.send(message)
        if coord:
            return [float(s) for s in coord]

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
        message = "SetToolCoordinateMotion," + self.robot_id + ',' + str(status) + self.end_msg
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
        message = "ReadMoveState," + self.robot_id + self.end_msg
        readmovestate = self.send(message)
        if readmovestate:
            status = int(readmovestate[0])
            return status
        return 1025

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
        target = [str(s) for s in target]
        target = (",".join(target))
        message = "MoveC," + self.robot_id + ',' + target + self.end_msg
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
