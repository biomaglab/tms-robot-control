import numpy as np

import robot.transformations as tr
import robot.control.robot_processing as robot_process


class RobotCoordinates:
    """
    Class to set/send robot coordinates.
    The class is required to avoid acquisition conflict with different threads (coordinates and navigation)
    """
    def __init__(self):
        self.robot_coord = [None]*6

    def SetRobotCoordinates(self, coord):
        coord_robot = np.array(coord)
        self.robot_coord = coord_robot

    def GetRobotCoordinates(self):
        return self.robot_coord


class TrackerCoordinates:
    """
    Class to set/get tracker coordinates and do tracker-related transformations.
    Tracker coordinates are acquired in InVesalius.
    The class is required to avoid acquisition conflict with different threads
    """
    def __init__(self):
        self.coord = [None, None, None]
        self.markers_flag = [False, False, False]
        self.m_tracker_to_robot = None

    def SetTrackerToRobotMatrix(self, m_tracker_to_robot):
        self.m_tracker_to_robot = m_tracker_to_robot

    def SetCoordinates(self, coord, markers_flag):
        self.coord = coord
        self.markers_flag = markers_flag

    def GetCoordinates(self):
        return self.coord, self.markers_flag

    def transform_matrix_to_robot_space(self, M, axes='rzyx'):
        X, Y, affine = self.m_tracker_to_robot

        M_in_robot_space = Y @ M @ tr.inverse_matrix(X)
        M_affine_in_robot_space = affine @ M

        _, angles_as_deg = robot_process.transformation_matrix_to_coordinates(M_in_robot_space, axes=axes)
        translation, _ = robot_process.transformation_matrix_to_coordinates(M_affine_in_robot_space, axes=axes)

        pose_in_robot_space = list(translation) + list(angles_as_deg)

        return pose_in_robot_space

    def transform_pose_to_robot_space(self, pose):
        M = robot_process.coordinates_to_transformation_matrix(
            position=pose[:3],
            orientation=pose[3:6],
            axes='rzyx',
        )
        pose_in_robot_space = self.transform_matrix_to_robot_space(M)

        if pose_in_robot_space is None:
            pose_in_robot_space = pose

        return pose_in_robot_space
