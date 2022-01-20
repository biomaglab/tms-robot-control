
import dataclasses
import numpy as np

import robot.constants as const
import robot.transformations as tr

import robot.control.elfin as elfin
import robot.control.coordinates as coordinates
import robot.control.elfin_processing as elfin_process


class RobotControl:
    def __init__(self, rc):
        self.rc = rc

        self.trk_init = None
        self.process_tracker = elfin_process.TrackerProcessing()

        self.robot_coordinates = coordinates.RobotCoordinates(rc)
        self.tracker_coordinates = coordinates.TrackerCoordinates()

        self.trck_init_robot = None
        self.robot_mode_status = False

        self.tracker_coord = []
        self.tracker_angles = []
        self.robot_coord = []
        self.robot_angles = []
        self.matrix_tracker_to_robot = []
        self.new_force_sensor_data = 0
        self.target_force_sensor_data = 0
        self.compensate_force_flag = False

        self.robot_tracker_flag = False
        self.target_index = None
        self.target_flag = False
        self.m_change_robot_to_head = [None] * 9
        self.coord_inv_old = None

        self.arc_motion_flag = False
        self.arc_motion_step_flag = None
        self.target_linear_out = None
        self.target_linear_in = None
        self.target_arc = None
        self.previous_robot_status = False

        self.robot_markers = []

    @dataclasses.dataclass
    class Robot_Marker:
        """Class for storing robot target."""
        m_robot_target : list = None
        target_force_sensor_data : float = 0

        @property
        def robot_target_matrix(self):
            return self.m_robot_target

        @robot_target_matrix.setter
        def robot_target_matrix(self, new_m_robot_target):
            self.m_robot_target = new_m_robot_target

        @property
        def robot_force_sensor_data(self):
            return self.target_force_sensor_data

        @robot_force_sensor_data.setter
        def robot_force_sensor_data(self, new_force_sensor_data):
            self.target_force_sensor_data = new_force_sensor_data

    def OnRobotConnection(self, data):
        robot_IP = data["robot_IP"]
        self.ElfinRobot(robot_IP)

    def OnUpdateRobotNavigationMode(self, data):
        self.robot_mode_status = data["robot_mode"]

    def OnUpdateRobotTargetMatrix(self, data):
        self.robot_tracker_flag = data["robot_tracker_flag"]
        self.target_index = data["target_index"]
        if self.robot_tracker_flag:
            self.m_change_robot_to_head = self.robot_markers[self.target_index].robot_target_matrix
            self.target_force_sensor_data = self.robot_markers[self.target_index].robot_force_sensor_data
            print("Setting robot target")
        else:
            self.m_change_robot_to_head = [None] * 9
            print("Invalid robot target")

    def OnResetProcessTracker(self, data):
        self.process_tracker.__init__()

    def OnUpdateCoordinates(self, data):
        if len(data) > 1:
            coord = data["coord"]
            markers_flag = data["markers_flag"]
            self.tracker_coordinates.SetCoordinates(np.vstack([coord[0], coord[1], coord[2]]), markers_flag)

    def OnUpdateTrackerFiducialsMatrix(self, data):
        self.matrix_tracker_fiducials = np.array(data["matrix_tracker_fiducials"])
        self.process_tracker.SetMatrixTrackerFiducials(self.matrix_tracker_fiducials)

    def OnCreatePoint(self, data):
        coord_raw, markers_flag = self.tracker_coordinates.GetCoordinates()
        coord_raw_robot = self.robot_coordinates.GetRobotCoordinates()
        coord_raw_tracker_obj = coord_raw[2]

        if markers_flag[2] and not any(coord is None for coord in coord_raw_robot):
            self.tracker_coord.append(coord_raw_tracker_obj[:3])
            self.tracker_angles.append(coord_raw_tracker_obj[3:])
            self.robot_coord.append(coord_raw_robot[:3])
            self.robot_angles.append(coord_raw_robot[3:])
            topic = 'Coordinates for the robot transformation matrix collected'
            data = {}
            self.rc.send_message(topic, data)
        else:
            print('Cannot collect the coil markers, please try again')

    def OnResetRobotMatrix(self, data):
        self.tracker_coord = []
        self.tracker_angles = []
        self.robot_coord = []
        self.robot_angles = []
        self.matrix_tracker_to_robot = []

    def OnRobotMatrixEstimation(self, data):
        tracker_coord = np.array(self.tracker_coord)
        robot_coord = np.array(self.robot_coord)

        matrix_robot_to_tracker = elfin_process.AffineTransformation(tracker_coord, robot_coord)
        matrix_tracker_to_robot = tr.inverse_matrix(matrix_robot_to_tracker)

        self.matrix_tracker_to_robot = matrix_tracker_to_robot
        self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

        topic = 'Update robot transformation matrix'
        data = {'data': self.matrix_tracker_to_robot.tolist()}
        self.rc.send_message(topic, data)

    def OnLoadRobotMatrix(self, data):
        self.matrix_tracker_to_robot = np.array(data["data"])
        self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

    def OnAddRobotMarker(self, data):
        if data["data"]:
            head_coordinates = self.tracker_coordinates.GetCoordinates()[0][1]
            robot_coordinates = self.robot_coordinates.GetRobotCoordinates()
            current_robot_target_matrix = elfin_process.compute_robot_to_head_matrix(head_coordinates, robot_coordinates)
        else:
            current_robot_target_matrix = [None]

        new_robot_marker = self.Robot_Marker()
        new_robot_marker.robot_target_matrix = current_robot_target_matrix
        new_robot_marker.target_force_sensor_data = self.new_force_sensor_data

        self.robot_markers.append(new_robot_marker)

    def OnDeleteAllRobotMarker(self, data):
        self.robot_markers = []

    def OnDeleteRobotMarker(self, data):
        index = data["index"]
        for i in reversed(index):
            del self.robot_markers[i]

    def ElfinRobot(self, robot_IP):
        print("Trying to connect Robot via: ", robot_IP)
        self.trck_init_robot = elfin.Elfin_Server(robot_IP, const.ROBOT_ElFIN_PORT)
        status_connection = self.trck_init_robot.Initialize()
        if status_connection:
            print('Connect to elfin robot tracking device.')
        else:
            topic = 'Dialog robot destroy'
            data = {}
            self.rc.send_message(topic, data)
            self.trck_init_robot = None
            print('Not possible to connect to elfin robot.')

        topic = 'Robot connection status'
        data = {'data': status_connection}
        self.rc.send_message(topic, data)

    def update_robot_coordinates(self):
        coord_robot_raw = self.trck_init_robot.Run()
        self.robot_coordinates.SetRobotCoordinates(coord_robot_raw)

    def robot_motion_reset(self):
        self.trck_init_robot.StopRobot()
        self.arc_motion_flag = False
        self.arc_motion_step_flag = const.ROBOT_MOTIONS["normal"]

    def check_robot_tracker_registration(self, current_tracker_coordinates_in_robot, coord_obj_tracker_in_robot,
                                         marker_obj_flag):
        if marker_obj_flag:
            if not np.allclose(np.array(current_tracker_coordinates_in_robot), np.array(coord_obj_tracker_in_robot), 0,
                               const.ROBOT_TRANSFORMATION_MATRIX_THRESHOLD):
                topic = 'Request new robot transformation matrix'
                data = {}
                self.rc.send_message(topic, data)
                print('Request new robot transformation matrix')

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

    def robot_control(self):
        current_tracker_coordinates_in_robot, markers_flag = self.tracker_coordinates.GetCoordinates()
        current_robot_coordinates = self.robot_coordinates.GetRobotCoordinates()
        self.new_force_sensor_data = self.trck_init_robot.GetForceSensorData()

        coord_head_tracker_in_robot = current_tracker_coordinates_in_robot[1]
        marker_head_flag = markers_flag[1]
        coord_obj_tracker_in_robot = current_tracker_coordinates_in_robot[2]
        marker_obj_flag = markers_flag[2]
        robot_status = False

        self.check_robot_tracker_registration(current_tracker_coordinates_in_robot, coord_obj_tracker_in_robot, marker_obj_flag)
        if self.new_force_sensor_data < const.ROBOT_FORCE_SENSOR_THRESHOLD:
            if self.robot_tracker_flag and np.all(self.m_change_robot_to_head[:3]):
                current_head = coord_head_tracker_in_robot
                if current_head is not None and marker_head_flag:
                    current_head_filtered = self.process_tracker.kalman_filter(current_head)
                    if self.process_tracker.compute_head_move_threshold(current_head_filtered):
                        new_robot_coordinates = self.process_tracker.compute_head_move_compensation(current_head_filtered,
                                                                                        self.m_change_robot_to_head)
                        robot_status = True
                        if self.coord_inv_old is None:
                           self.coord_inv_old = new_robot_coordinates

                        #if np.allclose(np.array(new_robot_coordinates)[:3], np.array(current_robot_coordinates)[:3], 0, 0.1):
                           # if self.target_force_sensor * 0.8 < force_sensor_data < self.target_force_sensor * 1.2:
                        if self.new_force_sensor_data >= (self.target_force_sensor_data + np.abs(self.target_force_sensor_data * 0.2)):
                            self.trck_init_robot.CompensateForce(self.compensate_force_flag)
                            self.compensate_force_flag = True
                        else:
                            self.compensate_force_flag = False
                            print("force OK")
                        if np.allclose(np.array(new_robot_coordinates), np.array(current_robot_coordinates), 0, 0.01):
                            #avoid small movements (0.01 mm)
                            pass
                        elif not np.allclose(np.array(new_robot_coordinates), np.array(self.coord_inv_old), 0, 5):
                            #if the head moves (>5mm) before the robot reach the target
                            self.trck_init_robot.StopRobot()
                            self.coord_inv_old = new_robot_coordinates
                        else:
                            distance_target = self.process_tracker.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates)
                            if not self.compensate_force_flag:
                                robot_status = self.robot_move_decision(distance_target, new_robot_coordinates, current_robot_coordinates, current_head_filtered)
                            self.coord_inv_old = new_robot_coordinates
                else:
                    print("Head marker is not visible")
                    self.trck_init_robot.StopRobot()
        else:
            print("Force sensor data higher than the upper threshold")
            self.trck_init_robot.StopRobot()

        return robot_status
