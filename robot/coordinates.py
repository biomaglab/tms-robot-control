import time

import numpy as np
import socketio

import elfin_processing as elfin_process


class RobotCoordinates:
    """
    Class to set/get robot coordinates. Robot coordinates are acquired in ControlRobot thread.
    The class is required to avoid acquisition conflict with different threads (coordinates and navigation)
    """
    def __init__(self, __sio):
        self.coord = None
        self.__sio = __sio


    def SetRobotCoordinates(self, coord):
        try:
            self.__sio.emit('from_robot', {'topic': 'Update Robot Coordinates', 'data': {'coord': coord.tolist()}})
            time.sleep(0.1)
        except socketio.exceptions.BadNamespaceError:
            print("skip")
        self.coord = coord

    def GetRobotCoordinates(self):
        return self.coord

class TrackerCoordinates:
    """
    Class to set/get coordinates. Tracker coordinates are acquired in InVesalius.
    The class is required to avoid acquisition conflict with different threads
    """
    def __init__(self):
        self.coord = [None, None, None]
        self.markers_flag = [False, False, False]
        self.m_tracker_to_robot = np.array([])

    def SetTrackerToRobotMatrix(self, m_tracker_to_robot):
        self.m_tracker_to_robot = np.array(m_tracker_to_robot)

    def SetCoordinates(self, coord, markers_flag):
        self.coord = coord
        self.markers_flag = markers_flag

    def GetCoordinates(self):
        if self.m_tracker_to_robot.any():
            coord = elfin_process.transform_tracker_to_robot(self.m_tracker_to_robot, self.coord)
        else:
            coord = self.coord

        return coord, self.markers_flag
