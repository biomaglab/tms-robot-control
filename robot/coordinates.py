import numpy as np

import robot.elfin_processing as elfin_process


class RobotCoordinates:
    """
    Class to set/get robot coordinates. Robot coordinates are acquired in ControlRobot thread.
    The class is required to avoid acquisition conflict with different threads (coordinates and navigation)
    """
    def __init__(self):
        self.coord = None

    def SetRobotCoordinates(self, sio, coord):
        #relay_server._to_neuronavigation({'topic': 'Update Robot Coordinates', 'data': coord})
        # sio.emit(
        #     event="to_neuronavigation",
        #     data={
        #         "topic": 'Update Robot Coordinates',
        #         "data": '',
        #     })
        #Publisher.sendMessage_no_hook('Update Robot Coordinates', coord=coord)
        self.coord = coord

    def GetRobotCoordinates(self):
        return self.coord

class TrackerCoordinates:
    """
    Class to set/get coordinates. Tracker coordinates are acquired in InVesalius.
    The class is required to avoid acquisition conflict with different threads
    """
    def __init__(self):
        self.coord = [None, None]
        self.coord = [False, False, False]
        self.m_tracker_to_robot = np.array([])

    def SetTrackerToRobotMatrix(self, m_tracker_to_robot):
        self.m_tracker_to_robot = np.array(m_tracker_to_robot)

    def SetCoordinates(self, coord, markers_flag):
        self.coord = coord
        self.markers_flag = markers_flag

    def GetCoordinates(self):
        if self.m_tracker_to_robot.any():
            self.coord = elfin_process.transform_tracker_to_robot(self.m_tracker_to_robot, self.coord)
        return self.coord, self.markers_flag
