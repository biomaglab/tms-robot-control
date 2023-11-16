
import dataclasses
import numpy as np
from collections import deque
from pynput import keyboard
import time

import robot.constants as const
import robot.transformations as tr

import robot.robot_api.elfin_api as elfin
import robot.robot_api.elfin_linux_api as elfin_linux
import robot.robot_api.dobot_api as dobot
import robot.control.coordinates as coordinates
import robot.control.ft as ft
import robot.control.robot_processing as robot_process


class RobotControl:
    def __init__(self, rc):
        self.rc = rc

        self.trk_init = None
        self.process_tracker = robot_process.TrackerProcessing()

        self.robot_coordinates = coordinates.RobotCoordinates(rc)
        self.tracker_coordinates = coordinates.TrackerCoordinates()

        self.trck_init_robot = None
        self.robot_mode_status = False

        self.tracker_coord_list = []
        self.robot_coord_list = []
        self.matrix_tracker_to_robot = []

        self.new_force_sensor_data = 0
        self.target_force_sensor_data = 5
        self.compensate_force_flag = False

        # reference force and moment values
        self.force_ref = np.array([0.0, 0.0, 0.0])
        self.moment_ref = np.array([0.0, 0.0, 0.0])
        self.inside_circle = False
        self.prev_state_flag = 0  # 0 for inside circle, 1 for outer circle
        self.F_dq = deque(maxlen=6)
        self.M_dq = deque(maxlen=6)

        self.REF_FLAG = False
        listener = keyboard.Listener(on_press=self._on_press)
        listener.start()
        #self.status = True

        self.robot_tracker_flag = False
        self.target_index = None
        self.target_flag = False
        self.m_change_robot_to_head = [None] * 9

        self.motion_step_flag = const.ROBOT_MOTIONS["normal"]
        self.target_linear_out = None
        self.target_linear_in = None
        self.target_arc = None
        self.previous_robot_status = False

        self.coil_at_target_state = False
        self.distance_to_target = [0]*6
        self.ft_distance_offset = [0, 0]

        self.robot_coord_matrix_list = np.zeros((4, 4))[np.newaxis]
        self.coord_coil_matrix_list = np.zeros((4, 4))[np.newaxis]

    def OnRobotConnection(self, data):
        robot_IP = data["robot_IP"]
        robot_model = data["robot_model"]
        self.RobotConnection(robot_IP, robot_model)

    def OnUpdateRobotNavigationMode(self, data):
        self.robot_mode_status = data["robot_mode"]

    def OnUpdateRobotTargetMatrix(self, data):
        if self.trck_init_robot:
            self.robot_tracker_flag = data["robot_tracker_flag"]
            self.target_index = data["target_index"]
            target = data["target"]
            if not self.robot_tracker_flag:
                self.trck_init_robot.ForceStopRobot()
                self.m_change_robot_to_head = [None] * 9
                self.target_force_sensor_data = 5
                print("Removing robot target")
            else:
                target = np.array(target).reshape(4, 4)
                self.m_change_robot_to_head = self.process_tracker.estimate_robot_target(self.tracker_coordinates, target)
                self.target_force_sensor_data = self.new_force_sensor_data
                print("Setting robot target")

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
        if self.create_calibration_point():
            topic = 'Coordinates for the robot transformation matrix collected'
            data = {}
            self.rc.send_message(topic, data)

    def OnResetRobotMatrix(self, data):
        self.robot_coord_matrix_list = np.zeros((4, 4))[np.newaxis]
        self.coord_coil_matrix_list = np.zeros((4, 4))[np.newaxis]
        self.tracker_coord_list = []
        self.robot_coord_list = []
        self.matrix_tracker_to_robot = []

    def OnRobotMatrixEstimation(self, data=None):
        try:
            affine_matrix_robot_to_tracker = robot_process.AffineTransformation(np.array(self.tracker_coord_list), np.array(self.robot_coord_list))
            affine_matrix_tracker_to_robot = tr.inverse_matrix(affine_matrix_robot_to_tracker)

            robot_coord_list = np.stack(self.robot_coord_matrix_list[1:], axis=2)
            coord_coil_list = np.stack(self.coord_coil_matrix_list[1:], axis=2)
            X_est, Y_est, Y_est_check, ErrorStats = robot_process.Transformation_matrix.matrices_estimation(robot_coord_list, coord_coil_list)
            print(robot_coord_list[:4, :4, -1][:3, 3].T - (Y_est @ coord_coil_list[:4, :4, -1] @ tr.inverse_matrix(X_est))[:3, 3].T)
            print(robot_coord_list[:4, :4, -1][:3, 3].T - (affine_matrix_tracker_to_robot @ coord_coil_list[:4, :4, -1])[:3, 3].T)
            # print(X_est)
            # print(Y_est)
            # print(Y_est_check)
            print(ErrorStats)
            self.matrix_tracker_to_robot = X_est, Y_est, affine_matrix_tracker_to_robot
            self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

            topic = 'Update robot transformation matrix'
            data = {'data': np.hstack(np.concatenate((X_est, Y_est, affine_matrix_tracker_to_robot))).tolist()}
            self.rc.send_message(topic, data)

        except np.linalg.LinAlgError:
            print("numpy.linalg.LinAlgError")
            print("Try a new acquisition")

    def OnLoadRobotMatrix(self, data):
        X_est, Y_est, affine_matrix_tracker_to_robot = np.split(np.array(data["data"]).reshape(12, 4), 3, axis=0)
        self.matrix_tracker_to_robot = X_est, Y_est, affine_matrix_tracker_to_robot
        self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

    def OnCoilToRobotAlignment(self, distance):
        xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]
        Rx = tr.rotation_matrix(const.ROBOT_RX_OFFSET, xaxis)
        Ry = tr.rotation_matrix(const.ROBOT_RY_OFFSET, yaxis)
        Rz = tr.rotation_matrix(const.ROBOT_RZ_OFFSET, zaxis)
        rotation_alignment_matrix = tr.concatenate_matrices(Rx, Ry, Rz)

        fix_axis = -distance[0], distance[1], distance[2], -distance[3], distance[4], distance[5]
        m_offset = robot_process.coordinates_to_transformation_matrix(
            position=fix_axis[:3],
            orientation=fix_axis[3:],
            axes='sxyz',
        )
        distance_matrix = np.linalg.inv(rotation_alignment_matrix) @ m_offset @ rotation_alignment_matrix

        return robot_process.transformation_matrix_to_coordinates(distance_matrix, axes='sxyz')

    def OnTuneTCP(self):
        robot_coord = self.robot_coordinates.GetRobotCoordinates()
        robot_coord_flip = robot_coord.copy()
        m_robot = robot_process.coordinates_to_transformation_matrix(
            position=robot_coord_flip[:3],
            orientation=robot_coord_flip[3:],
            axes='rzyx',
        )
        result_frame_X = m_robot[0, 0] * self.distance_to_target[0] + m_robot[0, 1] * self.distance_to_target[1] + m_robot[0, 2] * self.distance_to_target[2] + m_robot[0, 3]
        result_frame_Y = m_robot[1, 0] * self.distance_to_target[0] + m_robot[1, 1] * self.distance_to_target[1] + m_robot[1, 2] * self.distance_to_target[2] + m_robot[1, 3]
        result_frame_Z = m_robot[2, 0] * self.distance_to_target[0] + m_robot[2, 1] * self.distance_to_target[1] + m_robot[2, 2] * self.distance_to_target[2] + m_robot[2, 3]

        m_offset = robot_process.coordinates_to_transformation_matrix(
            position=self.distance_to_target[:3],
            orientation=self.distance_to_target[3:],
            axes='sxyz',
        )
        m_final = m_robot @ m_offset
        m_robot_offset = m_final.copy()
        m_robot_offset[:3, -1] = [result_frame_X, result_frame_Y, result_frame_Z]
        translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_robot_offset, axes='sxyz')

        return list(translation) + list(angles_as_deg)

    def OnDistanceToTarget(self, data):
        distance = data["distance"]
        translation, angles_as_deg = self.OnCoilToRobotAlignment(distance)
        translation[0] += self.ft_distance_offset[0]
        translation[1] += self.ft_distance_offset[1]
        self.distance_to_target = list(translation) + list(angles_as_deg)

    def OnCoilAtTarget(self, data):
        self.coil_at_target_state = data["state"]

    def RobotConnection(self, robot_IP, robot_model):
        print("Trying to connect Robot via: ", robot_IP, robot_model)
        if robot_model == "elfin":
            self.trck_init_robot = elfin.Server(robot_IP, const.ROBOT_ElFIN_PORT, self.rc)
            status_connection = self.trck_init_robot.Initialize()
        elif robot_model == "elfin_linux":
            self.trck_init_robot = elfin_linux.Server(robot_IP, const.ROBOT_ElFIN_PORT, self.rc)
            status_connection = self.trck_init_robot.Initialize()
        elif robot_model == "dobot":
            self.trck_init_robot = dobot.Server(robot_IP, self.rc)
            status_connection = self.trck_init_robot.Initialize()
        else:
            print('Unknown robot model.')
            status_connection = False

        if status_connection:
            print('Connect to robot tracking device.')
        else:
            topic = 'Dialog robot destroy'
            data = {}
            self.rc.send_message(topic, data)
            self.trck_init_robot = None
            print('Not possible to connect to robot.')

        topic = 'Robot connection status'
        data = {'data': status_connection}
        self.rc.send_message(topic, data)

    def SensorUpdateTarget(self, distance, status):
        topic = 'Update target from FT values'
        data = {'data' : (distance, status)}

        self.status = False

        self.rc.send_message(topic, data)

    def _on_press(self, key):
        '''
        Listen to keyboard press while the loop to display
        F - T values is running.

        Returns: None - regular actions
                    False - stop listener

        '''
        if key == keyboard.Key.esc:
            return False  # stop listener
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
        if k == 'n':
            print('Normalising . . . .')
            self.REF_FLAG = True

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
            new_robot_coord_list = robot_process.coordinates_to_transformation_matrix(
                position=coord_raw_robot[:3],
                orientation=coord_raw_robot[3:],
                axes='rzyx',
            )
            new_coord_coil_list = np.array(robot_process.coordinates_to_transformation_matrix(
                position=coord_raw_tracker_obj[:3],
                orientation=coord_raw_tracker_obj[3:],
                axes='rzyx',
            ))

            self.robot_coord_matrix_list = np.vstack([self.robot_coord_matrix_list.copy(), new_robot_coord_list[np.newaxis]])
            self.coord_coil_matrix_list = np.vstack([self.coord_coil_matrix_list.copy(), new_coord_coil_list[np.newaxis]])
            self.tracker_coord_list.append(coord_raw_tracker_obj[:3])
            self.robot_coord_list.append(coord_raw_robot[:3])

            return True
        else:
            print('Cannot collect the coil markers, please try again')
            return False

    def check_robot_tracker_registration(self, current_tracker_coordinates_in_robot, coord_obj_tracker_in_robot,
                                         marker_obj_flag):
        if marker_obj_flag:
            if not np.allclose(np.array(current_tracker_coordinates_in_robot)[:3], np.array(coord_obj_tracker_in_robot)[:3], 0,
                               const.ROBOT_TRANSFORMATION_MATRIX_THRESHOLD):
                topic = 'Request new robot transformation matrix'
                data = {}
                self.rc.send_message(topic, data)
                print('Request new robot transformation matrix')

    def robot_move_decision(self, new_robot_coordinates, current_robot_coordinates, coord_head_tracker, tunning_to_target, ft_values=False):
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
        current_robot_coordinates_flip_angle = current_robot_coordinates.copy()
        current_robot_coordinates_flip_angle[3], current_robot_coordinates_flip_angle[4], current_robot_coordinates_flip_angle[5] = current_robot_coordinates_flip_angle[5], current_robot_coordinates_flip_angle[4], current_robot_coordinates_flip_angle[3]
        distance_target = robot_process.correction_distance_calculation_target(tunning_to_target,
                                                                               current_robot_coordinates)
        distance_angle_target = robot_process.correction_distance_calculation_target(tunning_to_target[3:],
                                                                                     current_robot_coordinates_flip_angle[3:])
        #Check if the target is inside the working space
        if robot_process.estimate_robot_target_length(new_robot_coordinates) < const.ROBOT_WORKING_SPACE:
            #Check the target distance to define the motion mode

            if distance_target >= const.ROBOT_ARC_THRESHOLD_DISTANCE or distance_angle_target >= const.ROBOT_ARC_THRESHOLD_DISTANCE_ANGLE:
                head_center_coordinates = self.process_tracker.estimate_head_center_in_robot(self.tracker_coordinates.m_tracker_to_robot, coord_head_tracker).tolist()
                target_linear_out, middle_arc_point, target_arc = robot_process.compute_arc_motion(
                                                                            current_robot_coordinates,
                                                                            head_center_coordinates,
                                                                            new_robot_coordinates) #needs to be new_robot_coordinates!!
                if self.motion_step_flag == const.ROBOT_MOTIONS["normal"]:
                    self.target_linear_out = target_linear_out
                    self.motion_step_flag = const.ROBOT_MOTIONS["linear out"]

                if self.motion_step_flag == const.ROBOT_MOTIONS["linear out"]:
                    new_robot_target = self.target_linear_out
                    if np.allclose(np.array(current_robot_coordinates_flip_angle), np.array(self.target_linear_out), 0, 1):
                        self.motion_step_flag = const.ROBOT_MOTIONS["arc"]
                        self.target_arc = target_arc
                        new_robot_target = current_robot_coordinates_flip_angle, middle_arc_point, self.target_arc

                elif self.motion_step_flag == const.ROBOT_MOTIONS["arc"]:
                    #UPDATE arc motion target
                    if not np.allclose(np.array(target_arc), np.array(self.target_arc), 0, const.ROBOT_THRESHOLD_TO_UPDATE_TARGET_ARC):
                        if robot_process.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates_flip_angle) >= const.ROBOT_ARC_THRESHOLD_DISTANCE:
                            self.target_arc = target_arc
                            #Avoid small arc motion
                        elif robot_process.correction_distance_calculation_target(target_arc, current_robot_coordinates_flip_angle) < const.ROBOT_ARC_THRESHOLD_DISTANCE/2:
                            self.motion_step_flag = const.ROBOT_MOTIONS["normal"]
                            self.target_arc = tunning_to_target
                    new_robot_target = current_robot_coordinates_flip_angle, middle_arc_point, self.target_arc

                    if np.allclose(np.array(current_robot_coordinates)[:3], np.array(self.target_arc)[:3], 0, 20):
                       self.motion_step_flag = const.ROBOT_MOTIONS["normal"]
                       new_robot_target = tunning_to_target
                else:
                    self.motion_step_flag = const.ROBOT_MOTIONS["normal"]
                    new_robot_target = tunning_to_target
            else:
                self.motion_step_flag = const.ROBOT_MOTIONS["normal"]
                new_robot_target = tunning_to_target

            if not self.coil_at_target_state:
                if (np.sqrt(
                        np.sum(np.square(self.distance_to_target[:3]))) < const.ROBOT_TARGET_TUNING_THRESHOLD_DISTANCE or
                    np.sqrt(
                        np.sum(np.square(self.distance_to_target[3:]))) < const.ROBOT_TARGET_TUNING_THRESHOLD_ANGLE) \
                        and self.motion_step_flag != const.ROBOT_MOTIONS["arc"]:
                    # tunes the robot position based on neuronavigation
                    new_robot_target = self.distance_to_target
                    self.motion_step_flag = const.ROBOT_MOTIONS["tunning"]
                    self.inside_circle = True
                    #print('Distance to target: ', self.distance_to_target)
                    if const.FORCE_TORQUE_SENSOR and self.inside_circle and self.prev_state_flag == 1:
                        self.force_ref = ft_values[0:3]
                        self.moment_ref = ft_values[3:6]
                        print('Normalised!')
                        # print(self.force_ref)
                    self.prev_state_flag = 0
                else:
                    self.inside_circle = False
                    self.prev_state_flag = 1

                self.trck_init_robot.SendTargetToControl(new_robot_target, self.motion_step_flag)

            self.trck_init_robot.coil_at_target_state(self.coil_at_target_state)

            robot_status = True
        else:
            print("Head is too far from the robot basis")
            robot_status = False

        return robot_status

    def robot_control(self):
        robot_status = False
        current_tracker_coordinates, markers_flag = self.tracker_coordinates.GetCoordinates()
        if current_tracker_coordinates[1] is None:
            return robot_status
        marker_head_flag = markers_flag[1]
        marker_coil_flag = markers_flag[2]
        coord_head_tracker_filtered = self.process_tracker.kalman_filter(current_tracker_coordinates[1])

        current_robot_coordinates = self.robot_coordinates.GetRobotCoordinates()
        # TODO: condition for const.FORCE_TORQUE_SENSOR
        ft_values = np.array(self.trck_init_robot.GetForceSensorData())
        if const.FORCE_TORQUE_SENSOR:
            # true f-t value
            current_F = ft_values[0:3]
            current_M = ft_values[3:6]
            # normalised f-t value
            F_normalised = current_F - self.force_ref
            M_normalised = current_M - self.moment_ref
            self.F_dq.append(F_normalised)
            self.M_dq.append(M_normalised)

            if self.REF_FLAG:
                self.force_ref = current_F
                self.moment_ref = current_M
                self.REF_FLAG = False

            # smoothed f-t value, to increase size of filter increase deque size
            F_avg = np.mean(self.F_dq, axis=0)
            M_avg = np.mean(self.M_dq, axis=0)
            point_of_application = ft.find_r(F_avg, M_avg)

            point_of_application[0], point_of_application[1] = point_of_application[1], point_of_application[0] #change in axis, relevant for only aalto robot

            if const.DISPLAY_POA and len(point_of_application) == 3:
                with open(const.TEMP_FILE, 'a') as tmpfile:
                    tmpfile.write(f'{point_of_application}\n')

            self.new_force_sensor_data = -F_normalised[2]
            # Calculate vector of point of application vs centre
            distance = [point_of_application[0], point_of_application[1]]
            # self.ft_distance_offset = [point_of_application[0], point_of_application[1]]
            # TODO: Change this entire part to compensate force properly in a feedback loop

        if self.tracker_coordinates.m_tracker_to_robot is not None:
            coord_head_tracker_in_robot = robot_process.transform_tracker_to_robot(self.tracker_coordinates.m_tracker_to_robot, coord_head_tracker_filtered)
        else:
            coord_head_tracker_in_robot = coord_head_tracker_filtered

        #CHECK IF TARGET FROM INVESALIUS
        if self.robot_tracker_flag and np.all(self.m_change_robot_to_head[:3]):
            #self.check_robot_tracker_registration(current_robot_coordinates, coord_obj_tracker_in_robot, marker_obj_flag)
            #CHECK FORCE SENSOR
            if self.new_force_sensor_data <= const.ROBOT_FORCE_SENSOR_THRESHOLD or self.new_force_sensor_data <= (self.target_force_sensor_data + np.abs(self.target_force_sensor_data * (const.ROBOT_FORCE_SENSOR_SCALE_THRESHOLD / 100))):
                self.compensate_force_flag = False
                #CHECK IF HEAD IS VISIBLE
                if coord_head_tracker_in_robot is not None and marker_head_flag and marker_coil_flag:
                    #CHECK HEAD VELOCITY
                    if self.process_tracker.compute_head_move_threshold(coord_head_tracker_in_robot):
                        new_robot_coordinates = robot_process.compute_head_move_compensation(coord_head_tracker_in_robot, self.m_change_robot_to_head)
                        # if const.FORCE_TORQUE_SENSOR and np.sqrt(np.sum(np.square(self.distance_to_target[:3]))) < 10: # check if coil is 20mm from target and look for ft readout
                        #     if np.sqrt(np.sum(np.square(point_of_application[:2]))) > 0.5:
                        #         if self.status:
                        #             self.SensorUpdateTarget(distance, self.status)
                        #             self.status = False
                        tunning_to_target = self.OnTuneTCP()
                        robot_status = self.robot_move_decision(new_robot_coordinates, current_robot_coordinates,
                                                                coord_head_tracker_filtered, tunning_to_target, ft_values)
                    else:
                        print("Head is moving too much")
                        self.trck_init_robot.StopRobot()
                else:
                    print("Head marker is not visible")
                    self.trck_init_robot.StopRobot()
            else:
                #print("Compensating Force")
                print(self.new_force_sensor_data)
                self.compensate_force_flag = True
                self.trck_init_robot.CompensateForce(self.compensate_force_flag)
                time.sleep(0.5)
        else:
            #print("Navigation is off")
            pass

        return robot_status
