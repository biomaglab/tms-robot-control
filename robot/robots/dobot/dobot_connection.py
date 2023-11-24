import socket


class DobotConnection:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket_dobot = 0

        if self.port == 29999 or self.port == 30003 or self.port == 30004:
            try:
                self.socket_dobot = socket.socket()
                self.socket_dobot.connect((self.ip, self.port))
            except socket.error:
                print(socket.error)
                raise Exception(
                    f"Unable to set socket connection using port {self.port} !", socket.error)
        else:
            raise Exception(
                f"Connection to dobot needs to use one of ports: 29999, 30003, 30004 !")

    def send_data(self, string):
        try:
            self.socket_dobot.send(str.encode(string, 'utf-8'))
        except ConnectionAbortedError:
            print(f"ConnectionAbortedError. Unable to send message: {string}")

    def wait_reply(self):
        """
        Read the return value
        """
        try:
            data = self.socket_dobot.recv(1024)
            data_str = str(data, encoding="utf-8")
            print(data_str)
            return data_str
        except ConnectionAbortedError:
            print(f"ConnectionAbortedError. Unable to read message")
            return False

    def close(self):
        """
        Close the port
        """
        if (self.socket_dobot != 0):
            self.socket_dobot.close()

    def __del__(self):
        self.close()


class DobotApiDashboard(DobotConnection):
    """
    Define class dobot_api_dashboard to establish a connection to Dobot
    """

    def EnableRobot(self):
        """
        Enable the robot
        """
        string = "EnableRobot()"
        self.send_data(string)
        return self.wait_reply()

    def ClearError(self):
        """
        Clear controller alarm information
        """
        string = "ClearError()"
        self.send_data(string)
        return self.wait_reply()

    def ResetRobot(self):
        """
        Robot stop
        """
        string = "ResetRobot()"
        self.send_data(string)
        return self.wait_reply()

    def RobotMode(self):
        """
        View the robot status
        """
        string = "RobotMode()"
        self.send_data(string)
        return self.wait_reply()

    # Unused for now.
    def PowerOn(self):
        """
        Powering on the robot
        Note: It takes about 10 seconds for the robot to be enabled after it is powered on.
        """
        string = "PowerOn()"
        self.send_data(string)
        return self.wait_reply()

    # Unused for now.
    def GetErrorID(self):
        """
        Get robot error code
        """
        string = "GetErrorID()"
        self.send_data(string)
        return self.wait_reply()

    # Unused for now.
    def GetPose(self):
        """
        Description: get the current pose of the robot under the Cartesian coordinate system
        """
        string = "GetPose()"
        self.send_data(string)
        return self.wait_reply()


class DobotApiMove(DobotConnection):
    """
    Define class dobot_api_move to establish a connection to Dobot
    """

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
        self.send_data(string)
        return self.wait_reply()

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
        self.send_data(string)
        return self.wait_reply()

    def ServoP(self, target):
        """
        Dynamic following command based on Cartesian space
        x, y, z, a, b, c :Cartesian coordinate point value
        """
        x, y, z, rx, ry, rz = target[0], target[1], target[2], target[3], target[4], target[5]
        string = "ServoP({:f},{:f},{:f},{:f},{:f},{:f})".format(
            x, y, z, rx, ry, rz)
        self.send_data(string)
        return self.wait_reply()

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
        self.send_data(string)
        return self.wait_reply()
