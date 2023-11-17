from time import sleep
import robot.constants as const

from robot.robots.elfin.elfin_connection import ElfinConnection
from robot.robots.elfin.elfin_connection_linux import ElfinConnectionLinux


class Elfin():
    """
    The class for communicating with Elfin robot.
    """
    def __init__(self, ip, port, use_linux_version=False):
        self.coordinates = [None]*6
        self.target_reached = False

        self.use_linux_version = use_linux_version

        self.connection = ElfinConnectionLinux(ip, port) if self.use_linux_version else ElfinConnection(ip, port)

    def Connect(self):
        self.connection.connect()

    def IsConnected(self):
        return self.connection.connected

    def GetCoordinates(self):
        coordinates = self.connection.GetCoordinates()
        if coordinates:
            self.coordinates = coordinates
        return self.coordinates

    def SetTargetReached(self, target_reached):
        self.target_reached = target_reached

    def SendTargetToControl(self, target, motion_type=const.ROBOT_MOTIONS["normal"]):
        """
        It's not possible to send a move command to elfin if the robot is during a move.
         Status 1009 means robot in motion.
        """
        status = self.connection.GetMotionState()
        if motion_type == const.ROBOT_MOTIONS["normal"] or motion_type == const.ROBOT_MOTIONS["linear out"]:
            if self.use_linux_version:
                self.connection.MoveLinear(target)
            else:
                self.connection.MoveLinearWithWaypoint(target)
        elif motion_type == const.ROBOT_MOTIONS["arc"]:
            if status == const.ROBOT_ELFIN_MOVE_STATE["free to move"]:
                target_arc = target[1][:3] + target[2]
                self.connection.MoveCircular(target_arc)
            elif status == const.ROBOT_ELFIN_MOVE_STATE["error"]:
                self.StopRobot()
        elif motion_type == const.ROBOT_MOTIONS["tunning"]:
            if status == const.ROBOT_ELFIN_MOVE_STATE["free to move"]:
                self.connection.SetToolCoordinateMotion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
                #self.connection.SetSpeedRatio(0.1)  # Setting robot's movement speed
                abs_distance_to_target = [abs(x) for x in target]
                direction = abs_distance_to_target.index(max(abs_distance_to_target))
                CompenDistance = [direction, 1, target[direction]]
                self.connection.MoveLinearRelative(CompenDistance)
                self.connection.SetToolCoordinateMotion(0)

    def ReadForceSensor(self):
        if const.FORCE_TORQUE_SENSOR:
            return self.connection.ReadForceSensor()
        else:
            return False

    def CompensateForce(self, flag):
        status = self.connection.GetMotionState()
        if status == const.ROBOT_ELFIN_MOVE_STATE["free to move"]:
            if not flag:
                self.StopRobot()
            self.connection.SetToolCoordinateMotion(1)  # Set tool coordinate motion (0 = Robot base, 1 = TCP)
            #self.connection.SetSpeedRatio(0.1)  # Setting robot's movement speed
            CompenDistance = [2, 0, 1]  # [directionID; direction (0:negative, 1:positive); distance]
            self.connection.MoveLinearRelative(CompenDistance)  # Robot moves in specified spatial coordinate directional
            self.connection.SetToolCoordinateMotion(0)

    def StopRobot(self):
        # Takes some microseconds to the robot actual stops after the command.
        # The sleep time is required to guarantee the stop
        self.connection.StopRobot()
        sleep(0.05)

    def ForceStopRobot(self):
        self.StopRobot()

    def Close(self):
        self.StopRobot()
        #TODO: robot function to close? self.connection.close()
