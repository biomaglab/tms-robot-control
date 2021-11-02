#--------------------------------------------------------------------------
# Software:     InVesalius - Software de Reconstrucao 3D de Imagens Medicas
# Copyright:    (C) 2001  Centro de Pesquisas Renato Archer
# Homepage:     http://www.softwarepublico.gov.br
# Contact:      invesalius@cti.gov.br
# License:      GNU - GPL 2 (LICENSE.txt/LICENCA.txt)
#--------------------------------------------------------------------------
#    Este programa e software livre; voce pode redistribui-lo e/ou
#    modifica-lo sob os termos da Licenca Publica Geral GNU, conforme
#    publicada pela Free Software Foundation; de acordo com a versao 2
#    da Licenca.
#
#    Este programa eh distribuido na expectativa de ser util, mas SEM
#    QUALQUER GARANTIA; sem mesmo a garantia implicita de
#    COMERCIALIZACAO ou de ADEQUACAO A QUALQUER PROPOSITO EM
#    PARTICULAR. Consulte a Licenca Publica Geral GNU para obter mais
#    detalhes.
#--------------------------------------------------------------------------
import numpy as np
import wx
import queue
import threading
from time import sleep

import constants as const
import pub as Publisher

import robot.elfin as elfin
import robot.coordinates as coordinates
import robot.elfin_processing as elfin_process


class QueueCustom(queue.Queue):
    """
    A custom queue subclass that provides a :meth:`clear` method.
    https://stackoverflow.com/questions/6517953/clear-all-items-from-the-queue
    Modified to a LIFO Queue type (Last-in-first-out). Seems to make sense for the navigation
    threads, as the last added coordinate should be the first to be processed.
    In the first tests in a short run, seems to increase the coord queue size considerably,
    possibly limiting the queue size is good.
    """

    def clear(self):
        """
        Clears all items from the queue.
        """

        with self.mutex:
            unfinished = self.unfinished_tasks - len(self.queue)
            if unfinished <= 0:
                if unfinished < 0:
                    raise ValueError('task_done() called too many times')
                self.all_tasks_done.notify_all()
            self.unfinished_tasks = unfinished
            self.queue.clear()
            self.not_full.notify_all()


class Robot:
    def __init__(self, sio):
        """
        Class to establish the connection between the robot and InVesalius
        """
        self.sio = sio
        self.trk_init = None
        self.process_tracker = elfin_process.TrackerProcessing()

        self.thread_robot = None
        self.matrix_tracker_fiducials = None
        self.robot_target_queue = QueueCustom(maxsize=1)
        self.event_robot = threading.Event()

        self.robot_coordinates = coordinates.RobotCoordinates()
        self.tracker_coordinates = coordinates.TrackerCoordinates()

        self.__bind_events()

    def __bind_events(self):
        Publisher.subscribe(self.OnRobotConnection, 'Connect to robot')
        Publisher.subscribe(self.OnUpdateRobotTransformationMatrix, 'Update robot transformation matrix')
        Publisher.subscribe(self.OnUpdateRobotTargetMatrix, 'Robot target matrix')
        Publisher.subscribe(self.OnResetProcessTracker, 'Reset robot process')
        Publisher.subscribe(self.OnUpdateCoordinates, 'Update tracker coordinates')
        Publisher.subscribe(self.OnUpdateTrackerFiducialsMatrix, 'Update tracker fiducials matrix')

    def OnRobotConnection(self, data):
        robot_IP = data["robot_IP"]
        self.trck_init_robot = self.ElfinRobot(robot_IP)
        self.StartRobotThreadNavigation()

    def OnUpdateRobotTransformationMatrix(self, data):
        m_tracker_to_robot = data["m_tracker_to_robot"]
        self.tracker_coordinates.SetTrackerToRobotMatrix(m_tracker_to_robot)

    def OnUpdateRobotTargetMatrix(self, data):
        robot_tracker_flag = data["robot_tracker_flag"]
        m_change_robot_to_head = np.array(data["m_change_robot_to_head"])

        try:
            self.robot_target_queue.put_nowait([robot_tracker_flag, m_change_robot_to_head])
        except queue.Full:
            pass

    def OnResetProcessTracker(self, data):
        self.process_tracker.__init__()

    def OnUpdateCoordinates(self, data):
        coord = data["coord"]
        markers_flag = data["markers_flag"]
        self.tracker_coordinates.SetCoordinates(np.vstack([coord[0], coord[1], coord[2]]), markers_flag)

    def OnUpdateTrackerFiducialsMatrix(self, data):
        matrix_tracker_fiducials = np.array(data["matrix_tracker_fiducials"])
        self.process_tracker.SetMatrixTrackerFiducials(matrix_tracker_fiducials)

    def ElfinRobot(self, robot_IP):
        print("Trying to connect Robot via: ", robot_IP)
        trck_init = elfin.Elfin_Server(robot_IP, const.ROBOT_ElFIN_PORT)
        trck_init.Initialize()
        print('Connect to elfin robot tracking device.')

        # return tracker initialization variable and type of connection
        return trck_init

    def StartRobotThreadNavigation(self):
        if self.event_robot.is_set():
            self.event_robot.clear()
        self.thread_robot = ControlRobot(self.robot_coordinates, self.tracker_coordinates, self.process_tracker,
                                         self.trck_init_robot, self.robot_target_queue, self.event_robot, self.sio)
        self.thread_robot.start()

    def StopRobotThreadNavigation(self):
        self.thread_robot.join()
        self.OnResetProcessTracker()


class ControlRobot(threading.Thread):
    def __init__(self, robot_coordinates, tracker_coordinates, process_tracker,
                 trck_init_robot, robot_target_queue, event_robot, sio):
        """Class (threading) to perform the robot control.
        A distinguish thread is required to increase perform and separate robot control from navigation thread
        (safetly layer for robot positining).

        There is no GUI involved, them no Sleep is required

        :param trck_init: Initialization variable of tracking device and connection type. See tracker.py.
        :param tracker: tracker.py  instance
        :param robot_coordinates: RobotCoordinates() instance
        :param queues: Queue instance that manage coordinates and robot target
        :param process_tracker: TrackerProcessing() instance from elfin_process
        :param event_robot: Threading event to ControlRobot when tasks is done
        """
        threading.Thread.__init__(self, name='ControlRobot')

        self.robotcoordinates = robot_coordinates
        self.coordinates = tracker_coordinates
        self.robot_tracker_flag = False
        self.target_flag = False
        self.m_change_robot_to_head = None
        self.coord_inv_old = None

        self.process_tracker = process_tracker
        self.trck_init_robot = trck_init_robot

        self.robot_target_queue = robot_target_queue
        self.event_robot = event_robot
        self.sio = sio

        self.arc_motion_flag = False
        self.arc_motion_step_flag = None
        self.target_linear_out = None
        self.target_linear_in = None
        self.target_arc = None
        self.previous_robot_status = False

    def get_coordinates_from_tracker_devices(self):
        coord_robot_raw = self.trck_init_robot.Run()
        coord_robot = np.array(coord_robot_raw)
        coord_robot[3], coord_robot[5] = coord_robot[5], coord_robot[3]
        self.robotcoordinates.SetRobotCoordinates(self.sio, coord_robot)

        coord_raw, markers_flag = self.coordinates.GetCoordinates()

        return coord_raw, coord_robot_raw, markers_flag

    def robot_motion_reset(self):
        self.trck_init_robot.StopRobot()
        self.arc_motion_flag = False
        self.arc_motion_step_flag = const.ROBOT_MOTIONS["normal"]

    def robot_move_decision(self, distance_target, new_robot_coordinates, current_robot_coordinates, current_head_filtered):
        """
        There are two types of robot movements.
        We can imagine in two concentric spheres of different sizes. The inside sphere is to compensate for small head movements.
         It was named "normal" moves.
        The outside sphere is for the arc motion. The arc motion is a safety feature for long robot movements.
         Even for a new target or a sudden huge head movement.
        1) normal:
            A linear move from the actual position until the target position.
            This movement just happens when move distance is below a threshold (const.ROBOT_ARC_THRESHOLD_DISTANCE)
        2) arc motion:
            It can be divided into three parts.
                The first one represents the movement from the inner sphere to the outer sphere.
                 The robot moves back using a radial move (it use the center of the head as a reference).
                The second step is the actual arc motion (along the outer sphere).
                 A middle point, between the actual position and the target, is required.
                The last step is to make a linear move until the target (goes to the inner sphere)

        """
        #Check if the target is inside the working space
        if self.process_tracker.estimate_robot_target_length(new_robot_coordinates) < const.ROBOT_WORKING_SPACE:
            #Check the target distance to define the motion mode
            if distance_target < const.ROBOT_ARC_THRESHOLD_DISTANCE and not self.arc_motion_flag:
                self.trck_init_robot.SendCoordinates(new_robot_coordinates, const.ROBOT_MOTIONS["normal"])

            elif distance_target >= const.ROBOT_ARC_THRESHOLD_DISTANCE or self.arc_motion_flag:
                actual_point = current_robot_coordinates
                if not self.arc_motion_flag:
                    head_center_coordinates = self.process_tracker.estimate_head_center(current_head_filtered).tolist()

                    self.target_linear_out, self.target_arc = self.process_tracker.compute_arc_motion(current_robot_coordinates, head_center_coordinates,
                                                                                                      new_robot_coordinates)
                    self.arc_motion_flag = True
                    self.arc_motion_step_flag = const.ROBOT_MOTIONS["linear out"]

                if self.arc_motion_flag and self.arc_motion_step_flag == const.ROBOT_MOTIONS["linear out"]:
                    coord = self.target_linear_out
                    if np.allclose(np.array(actual_point), np.array(self.target_linear_out), 0, 1):
                        self.arc_motion_step_flag = const.ROBOT_MOTIONS["arc"]
                        coord = self.target_arc

                elif self.arc_motion_flag and self.arc_motion_step_flag == const.ROBOT_MOTIONS["arc"]:
                    head_center_coordinates = self.process_tracker.estimate_head_center(current_head_filtered).tolist()

                    _, new_target_arc = self.process_tracker.compute_arc_motion(current_robot_coordinates, head_center_coordinates,
                                                                                new_robot_coordinates)
                    if np.allclose(np.array(new_target_arc[3:-1]), np.array(self.target_arc[3:-1]), 0, 1):
                        None
                    else:
                        if self.process_tracker.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates) >= \
                                const.ROBOT_ARC_THRESHOLD_DISTANCE*0.8:
                            self.target_arc = new_target_arc

                    coord = self.target_arc

                    if np.allclose(np.array(actual_point), np.array(self.target_arc[3:-1]), 0, 10):
                        self.arc_motion_flag = False
                        self.arc_motion_step_flag = const.ROBOT_MOTIONS["normal"]
                        coord = new_robot_coordinates

                self.trck_init_robot.SendCoordinates(coord, self.arc_motion_step_flag)
            robot_status = True
        else:
            print("Head is too far from the robot basis")
            robot_status = False

        return robot_status

    def robot_control(self, current_tracker_coordinates_in_robot, current_robot_coordinates, markers_flag):
        coord_head_tracker_in_robot = current_tracker_coordinates_in_robot[1]
        marker_head_flag = markers_flag[1]
        coord_obj_tracker_in_robot = current_tracker_coordinates_in_robot[2]
        marker_obj_flag = markers_flag[2]
        robot_status = False

        if self.robot_tracker_flag:
            current_head = coord_head_tracker_in_robot
            if current_head is not None and marker_head_flag:
                current_head_filtered = self.process_tracker.kalman_filter(current_head)
                if self.process_tracker.compute_head_move_threshold(current_head_filtered):
                    new_robot_coordinates = self.process_tracker.compute_head_move_compensation(current_head_filtered,
                                                                                    self.m_change_robot_to_head)
                    robot_status = True
                    if self.coord_inv_old is None:
                       self.coord_inv_old = new_robot_coordinates

                    if np.allclose(np.array(new_robot_coordinates), np.array(current_robot_coordinates), 0, 0.01):
                        #avoid small movements (0.01 mm)
                        pass
                    elif not np.allclose(np.array(new_robot_coordinates), np.array(self.coord_inv_old), 0, 5):
                        #if the head moves (>5mm) before the robot reach the target
                        self.trck_init_robot.StopRobot()
                        self.coord_inv_old = new_robot_coordinates
                    else:
                        distance_target = self.process_tracker.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates)
                        robot_status = self.robot_move_decision(distance_target, new_robot_coordinates, current_robot_coordinates, current_head_filtered)
                        self.coord_inv_old = new_robot_coordinates
            else:
                self.trck_init_robot.StopRobot()

        return robot_status

    def run(self):
        while not self.event_robot.is_set():
            current_tracker_coordinates, current_robot_coordinates, markers_flag = self.get_coordinates_from_tracker_devices()

            if not self.robot_target_queue.empty():
                self.robot_tracker_flag, self.m_change_robot_to_head = self.robot_target_queue.get_nowait()
                self.robot_motion_reset()
                self.robot_target_queue.task_done()

            robot_status = self.robot_control(current_tracker_coordinates, current_robot_coordinates, markers_flag)

            if self.previous_robot_status != robot_status:
                ##wx.CallAfter(Publisher.sendMessage, 'Update robot status', robot_status=robot_status)
                self.previous_robot_status = robot_status

            sleep(const.SLEEP_ROBOT)
