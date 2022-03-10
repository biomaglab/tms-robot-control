
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

        self.motion_step_flag = const.ROBOT_MOTIONS["normal"]
        self.target_linear_out = None
        self.target_linear_in = None
        self.target_arc = None
        self.previous_robot_status = False

        self.robot_markers = []

        self.robot_coord_list = np.zeros((4, 4))[np.newaxis]
        self.coord_coil_list = np.zeros((4, 4))[np.newaxis]

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
        target = data["target"]
        if target is not None:
            # coord = self.process_tracker.align_coil_with_head_center(self.tracker_coordinates,
            #                                                                          self.robot_coordinates)
            # self.trck_init_robot.SendCoordinates(coord)
            self.m_change_robot_to_head = self.process_tracker.estimate_robot_target(self.trck_init_robot, self.tracker_coordinates,
                                                                                     self.robot_coordinates, target)
            self.target_force_sensor_data = self.robot_markers[self.target_index].robot_force_sensor_data
            print("Setting robot target")
        else:
            if self.robot_tracker_flag:
                self.m_change_robot_to_head = self.robot_markers[self.target_index].robot_target_matrix
                self.target_force_sensor_data = self.robot_markers[self.target_index].robot_force_sensor_data
                print("Setting robot target for head move compensation")
            else:
                self.m_change_robot_to_head = [None] * 9
                self.target_force_sensor_data = 0
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
        self.create_calibration_point()
        topic = 'Coordinates for the robot transformation matrix collected'
        data = {}
        self.rc.send_message(topic, data)

    def OnResetRobotMatrix(self, data):
        self.robot_coord_list = np.zeros((4, 4))[np.newaxis]
        self.coord_coil_list = np.zeros((4, 4))[np.newaxis]
        self.matrix_tracker_to_robot = []

    def OnRobotMatrixEstimation(self, data=None):
        try:
            robot_coord_list = np.stack(self.robot_coord_list[1:], axis=2)
            coord_coil_list = np.stack(self.coord_coil_list[1:], axis=2)
            X_est, Y_est, Y_est_check, ErrorStats = elfin_process.Transformation_matrix.matrices_estimation(robot_coord_list, coord_coil_list)
            print(robot_coord_list[:4, :4, -1][:3, 3].T - (Y_est @ coord_coil_list[:4, :4, -1] @ tr.inverse_matrix(X_est))[:3, 3].T)
            # print(X_est)
            # print(Y_est)
            # print(Y_est_check)
            print(ErrorStats)
            self.matrix_tracker_to_robot = X_est, Y_est
            self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

            topic = 'Update robot transformation matrix'
            data = {'data': np.hstack(np.concatenate((X_est, Y_est))).tolist()}
            self.rc.send_message(topic, data)

        except np.linalg.LinAlgError:
            print("numpy.linalg.LinAlgError")
            print("Try a new acquisition")

    def OnLoadRobotMatrix(self, data):
        X_est, Y_est = np.split(np.array(data["data"]).reshape(8, 4), 2, axis=0)
        self.matrix_tracker_to_robot = X_est, Y_est
        self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

    def OnAddRobotMarker(self, data):
        if data["data"]:
            coordinates = self.tracker_coordinates.GetCoordinates()[0]
            if self.tracker_coordinates.m_tracker_to_robot is not None:
                head_coordinates_in_robot = elfin_process.transform_tracker_to_robot(self.tracker_coordinates.m_tracker_to_robot, coordinates[1])
            else:
                head_coordinates_in_robot = coordinates[1]
            robot_coordinates = self.robot_coordinates.GetRobotCoordinates()
            current_robot_target_matrix = elfin_process.compute_robot_to_head_matrix(head_coordinates_in_robot, robot_coordinates)
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
        self.motion_step_flag = const.ROBOT_MOTIONS["normal"]

    def create_calibration_point(self):
        coord_raw, markers_flag = self.tracker_coordinates.GetCoordinates()
        coord_raw_robot = self.robot_coordinates.GetRobotCoordinates()
        coord_raw_tracker_obj = coord_raw[2]

        if markers_flag[2] and not any(coord is None for coord in coord_raw_robot):
            new_robot_coord_list = elfin_process.coordinates_to_transformation_matrix(
                position=coord_raw_robot[:3],
                orientation=coord_raw_robot[3:],
                axes='rzyx',
            )
            new_coord_coil_list = np.array(elfin_process.coordinates_to_transformation_matrix(
                position=coord_raw_tracker_obj[:3],
                orientation=coord_raw_tracker_obj[3:],
                axes='rzyx',
            ))

            self.robot_coord_list = np.vstack([self.robot_coord_list.copy(), new_robot_coord_list[np.newaxis]])
            self.coord_coil_list = np.vstack([self.coord_coil_list.copy(), new_coord_coil_list[np.newaxis]])
        else:
            print('Cannot collect the coil markers, please try again')

    def check_robot_tracker_registration(self, current_tracker_coordinates_in_robot, coord_obj_tracker_in_robot,
                                         marker_obj_flag):
        if marker_obj_flag:
            if not np.allclose(np.array(current_tracker_coordinates_in_robot)[:3], np.array(coord_obj_tracker_in_robot)[:3], 0,
                               const.ROBOT_TRANSFORMATION_MATRIX_THRESHOLD):
                topic = 'Request new robot transformation matrix'
                data = {}
                self.rc.send_message(topic, data)
                print('Request new robot transformation matrix')

    def robot_motion(self, current_robot_coordinates, new_robot_coordinates, coord_head_tracker):
        robot_status = True
        if self.coord_inv_old is None:
            self.coord_inv_old = new_robot_coordinates

        # if np.allclose(np.array(new_robot_coordinates)[:3], np.array(current_robot_coordinates)[:3], 0, 0.1):
        if np.allclose(np.array(new_robot_coordinates), np.array(current_robot_coordinates), 0, 10):
            # avoid small movements (1 mm)
            pass
        elif not np.allclose(np.array(new_robot_coordinates), np.array(self.coord_inv_old), 0, 10):
            # if the head moves (>10mm) before the robot reach the target
            self.trck_init_robot.StopRobot()
            self.coord_inv_old = new_robot_coordinates
        else:
            distance_target = elfin_process.correction_distance_calculation_target(new_robot_coordinates,
                                                                                   current_robot_coordinates)
            robot_status = self.robot_move_decision(distance_target, new_robot_coordinates,
                                                    current_robot_coordinates, coord_head_tracker)
            self.coord_inv_old = new_robot_coordinates

        return robot_status

    def robot_move_decision(self, distance_target, new_robot_coordinates, current_robot_coordinates, coord_head_tracker):
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
        if elfin_process.estimate_robot_target_length(new_robot_coordinates) < const.ROBOT_WORKING_SPACE:
            #Check the target distance to define the motion mode
            if distance_target >= const.ROBOT_ARC_THRESHOLD_DISTANCE:
                head_center_coordinates = self.process_tracker.estimate_head_center_in_robot(self.tracker_coordinates.m_tracker_to_robot, coord_head_tracker).tolist()
                target_linear_out, target_arc = elfin_process.compute_arc_motion(current_robot_coordinates,
                                                                                 head_center_coordinates,
                                                                                 new_robot_coordinates)
                if self.motion_step_flag == const.ROBOT_MOTIONS["normal"]:
                    self.target_linear_out = target_linear_out
                    self.motion_step_flag = const.ROBOT_MOTIONS["linear out"]

                if self.motion_step_flag == const.ROBOT_MOTIONS["linear out"]:
                    new_robot_coordinates = self.target_linear_out
                    if np.allclose(np.array(current_robot_coordinates)[:3], np.array(self.target_linear_out)[:3], 0, 1):
                        self.motion_step_flag = const.ROBOT_MOTIONS["arc"]
                        self.target_arc = target_arc
                        new_robot_coordinates = self.target_arc

                elif self.motion_step_flag == const.ROBOT_MOTIONS["arc"]:
                    if not np.allclose(np.array(target_arc[3:-1]), np.array(self.target_arc[3:-1]), 0, 20):
                        if elfin_process.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates) >= \
                                const.ROBOT_ARC_THRESHOLD_DISTANCE:
                            self.target_arc = target_arc
                    new_robot_coordinates = self.target_arc

                    if np.allclose(np.array(current_robot_coordinates)[:3], np.array(self.target_arc[3:-1])[:3], 0, 20):
                        self.motion_step_flag = const.ROBOT_MOTIONS["normal"]

            else:
                self.motion_step_flag = const.ROBOT_MOTIONS["normal"]

            self.trck_init_robot.SendCoordinatesControl(new_robot_coordinates, self.motion_step_flag)

            robot_status = True
        else:
            print("Head is too far from the robot basis")
            robot_status = False

        return robot_status

    def robot_control(self):
        current_tracker_coordinates, markers_flag = self.tracker_coordinates.GetCoordinates()
        marker_head_flag = markers_flag[1]
        coord_head_tracker_filtered = self.process_tracker.kalman_filter(current_tracker_coordinates[1])

        current_robot_coordinates = self.robot_coordinates.GetRobotCoordinates()

        self.new_force_sensor_data = self.trck_init_robot.GetForceSensorData()

        if self.tracker_coordinates.m_tracker_to_robot is not None:
            coord_head_tracker_in_robot = elfin_process.transform_tracker_to_robot(self.tracker_coordinates.m_tracker_to_robot, coord_head_tracker_filtered)
        else:
            coord_head_tracker_in_robot = coord_head_tracker_filtered

        robot_status = False

        #CHECK IF TARGET FROM INVESALIUS
        if self.robot_tracker_flag and np.all(self.m_change_robot_to_head[:3]):
            #self.check_robot_tracker_registration(current_robot_coordinates, coord_obj_tracker_in_robot, marker_obj_flag)
            #CHECK FORCE SENSOR
            if self.new_force_sensor_data < const.ROBOT_FORCE_SENSOR_THRESHOLD:
                # if self.target_force_sensor * 0.8 < force_sensor_data < self.target_force_sensor * 1.2:
                if self.new_force_sensor_data <= (self.target_force_sensor_data + np.abs(self.target_force_sensor_data * (const.ROBOT_FORCE_SENSOR_SCALE_THRESHOLD / 100))):
                    self.compensate_force_flag = False
                    #CHECK IF HEAD IS VISIBLE
                    if coord_head_tracker_in_robot is not None and marker_head_flag:
                        #CHECK HEAD VELOCITY
                        if self.process_tracker.compute_head_move_threshold(coord_head_tracker_in_robot):
                            new_robot_coordinates = elfin_process.compute_head_move_compensation(coord_head_tracker_in_robot, self.m_change_robot_to_head)
                            robot_status = self.robot_motion(current_robot_coordinates, new_robot_coordinates, coord_head_tracker_filtered)
                        else:
                            print("Head is moving too much")
                            self.trck_init_robot.StopRobot()
                    else:
                        print("Head marker is not visible")
                        self.trck_init_robot.StopRobot()
                else:
                    print("Compensating Force")
                    self.trck_init_robot.CompensateForce(self.compensate_force_flag)
                    self.compensate_force_flag = True
            else:
                print("Force sensor data higher than the upper threshold")
                self.trck_init_robot.StopRobot()
        else:
            print("InVesalius stopped navigation")
            self.trck_init_robot.StopRobot()

        return robot_status
