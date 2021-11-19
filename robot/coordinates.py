import time

import numpy as np
import socketio

import robot.elfin_processing as elfin_process


class RobotCoordinates:
    """
    Class to set/send robot coordinates.
    The class is required to avoid acquisition conflict with different threads (coordinates and navigation)
    """
    def __init__(self, rc):
        self.rc = rc
        self.robot_coord = None

    def SetRobotCoordinates(self, coord):
        self.robot_coord = coord
        coord_robot = np.array(coord)
        #coord_robot[3], coord_robot[5] = coord_robot[5], coord_robot[3]

    def GetRobotCoordinates(self):
        return self.robot_coord


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
