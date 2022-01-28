
import numpy as np

import robot.control.elfin_processing as elfin_process


class RobotCoordinates:
    """
    Class to set/send robot coordinates.
    The class is required to avoid acquisition conflict with different threads (coordinates and navigation)
    """
    def __init__(self, rc):
        self.rc = rc
        self.robot_coord = [None]*6
        self.robot_coord_list = np.zeros((4, 4))
        self.robot_coord_list = self.robot_coord_list[np.newaxis]

    def SetRobotCoordinates(self, coord):
        coord_robot = np.array(coord)
        coord_robot[3], coord_robot[5] = coord_robot[5], coord_robot[3]
        self.robot_coord = coord_robot
        new_robot_coord_list = elfin_process.coordinates_to_transformation_matrix(
            position=coord_robot[:3],
            orientation=coord_robot[3:],
            axes='rzyx',
        )
        self.robot_coord_list = np.vstack([self.robot_coord_list.copy(), new_robot_coord_list[np.newaxis]])
        if len(self.robot_coord_list)>20:
            self.robot_coord_list = np.delete(self.robot_coord_list.copy(), 0, axis=0)

    def GetRobotCoordinates(self):
        return self.robot_coord, self.robot_coord_list


class TrackerCoordinates:
    """
    Class to set/get coordinates. Tracker coordinates are acquired in InVesalius.
    The class is required to avoid acquisition conflict with different threads
    """
    def __init__(self):
        self.coord = [None]*6
        self.coord_coil_list = np.zeros((4, 4))
        self.coord_coil_list = self.coord_coil_list[np.newaxis]
        self.markers_flag = [False, False, False]
        self.m_tracker_to_robot = np.array([])

    def SetTrackerToRobotMatrix(self, m_tracker_to_robot):
        self.m_tracker_to_robot = np.array(m_tracker_to_robot)

    def SetCoordinates(self, coord, markers_flag):
        self.coord = coord
        self.markers_flag = markers_flag
        new_coord_coil_list = np.array(elfin_process.coordinates_to_transformation_matrix(
            position=coord[2][:3],
            orientation=coord[2][3:],
            axes='rzyx',
        ))

        self.coord_coil_list = np.vstack([self.coord_coil_list.copy(), new_coord_coil_list[np.newaxis]])
        if len(self.coord_coil_list)>20:
            self.coord_coil_list = np.delete(self.coord_coil_list.copy(), 0, axis=0)

    def GetCoordinates(self):
        if self.m_tracker_to_robot.any():
            coord = elfin_process.transform_tracker_to_robot(self.m_tracker_to_robot, self.coord)
        else:
            coord = self.coord

        return coord, self.markers_flag, self.coord_coil_list
