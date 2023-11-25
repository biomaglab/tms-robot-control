
import dataclasses
import numpy as np
from collections import deque
from pynput import keyboard
import time

import robot.constants as const
from robot.constants import MotionType
import robot.transformations as tr

import robot.robots.elfin.elfin as elfin
import robot.robots.dobot.dobot as dobot
import robot.control.coordinates as coordinates
import robot.control.ft as ft
import robot.control.robot_processing as robot_process


class RobotControl:
    def __init__(self, robot_type, remote_control, site_config, robot_config):
        self.robot_type = robot_type
        self.remote_control = remote_control

        self.site_config = site_config
        self.robot_config = robot_config

        self.process_tracker = robot_process.TrackerProcessing(
            robot_config=robot_config,
        )

        self.robot_coordinates = coordinates.RobotCoordinates()
        self.tracker_coordinates = coordinates.TrackerCoordinates()

        self.robot = None
        self.robot_mode_status = False

        self.tracker_coord_list = []
        self.robot_coord_list = []
        self.matrix_tracker_to_robot = []

        self.new_force_sensor_data = 0
        self.target_force_sensor_data = 5

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

        self.motion_type = MotionType.NORMAL
        self.linear_target = None
        self.target_arc = None
        self.previous_robot_status = False

        self.target_reached = False
        self.displacement_to_target = [0]*6
        self.ft_distance_offset = [0, 0]

        self.robot_coord_matrix_list = np.zeros((4, 4))[np.newaxis]
        self.coord_coil_matrix_list = np.zeros((4, 4))[np.newaxis]

    def OnRobotConnection(self, data):
        robot_IP = data["robot_IP"]
        self.ConnectToRobot(robot_IP)

    def OnUpdateRobotNavigationMode(self, data):
        self.robot_mode_status = data["robot_mode"]

    def OnUpdateRobotTargetMatrix(self, data):
        if self.robot:
            self.robot_tracker_flag = data["robot_tracker_flag"]
            self.target_index = data["target_index"]
            target = data["target"]
            if not self.robot_tracker_flag:
                self.robot.force_stop_robot()
                self.m_change_robot_to_head = [None] * 9
                self.target_force_sensor_data = 5
                print("Removing robot target")
            else:
                target = np.array(target).reshape(4, 4)
                self.m_change_robot_to_head = self.process_tracker.estimate_robot_target(self.tracker_coordinates, target)
                self.target_force_sensor_data = self.new_force_sensor_data
                print("Setting robot target")

    def OnResetProcessTracker(self, data):
        # TODO: This shouldn't call the constructor again but instead a separate reset method.
        self.process_tracker.__init__(
            robot_config=self.robot_config
        )

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
            self.remote_control.send_message(topic, data)

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
            self.remote_control.send_message(topic, data)

        except np.linalg.LinAlgError:
            print("numpy.linalg.LinAlgError")
            print("Try a new acquisition")

    def OnLoadRobotMatrix(self, data):
        X_est, Y_est, affine_matrix_tracker_to_robot = np.split(np.array(data["data"]).reshape(12, 4), 3, axis=0)
        self.matrix_tracker_to_robot = X_est, Y_est, affine_matrix_tracker_to_robot
        self.tracker_coordinates.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

    def OnCoilToRobotAlignment(self, distance):
        xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]

        rx_offset = self.site_config['rx_offset']
        ry_offset = self.site_config['ry_offset']
        rz_offset = self.site_config['rz_offset']

        Rx = tr.rotation_matrix(rx_offset, xaxis)
        Ry = tr.rotation_matrix(ry_offset, yaxis)
        Rz = tr.rotation_matrix(rz_offset, zaxis)

        rotation_alignment_matrix = tr.multiply_matrices(Rx, Ry, Rz)

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
        m_robot = robot_process.coordinates_to_transformation_matrix(
            position=robot_coord[:3],
            orientation=robot_coord[3:],
            axes='sxyz',
        )
        result_frame_X = m_robot[0, 0] * self.displacement_to_target[0] + m_robot[0, 1] * self.displacement_to_target[1] + m_robot[0, 2] * self.displacement_to_target[2] + m_robot[0, 3]
        result_frame_Y = m_robot[1, 0] * self.displacement_to_target[0] + m_robot[1, 1] * self.displacement_to_target[1] + m_robot[1, 2] * self.displacement_to_target[2] + m_robot[1, 3]
        result_frame_Z = m_robot[2, 0] * self.displacement_to_target[0] + m_robot[2, 1] * self.displacement_to_target[1] + m_robot[2, 2] * self.displacement_to_target[2] + m_robot[2, 3]

        m_offset = robot_process.coordinates_to_transformation_matrix(
            position=self.displacement_to_target[:3],
            orientation=self.displacement_to_target[3:],
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
        self.displacement_to_target = list(translation) + list(angles_as_deg)

    def OnCoilAtTarget(self, data):
        self.target_reached = data["state"]

    def ConnectToRobot(self, robot_IP):
        robot_type = self.robot_type
        print("Trying to connect to robot '{}' with IP: {}".format(robot_type, robot_IP))

        if robot_type == "elfin":
            self.robot = elfin.Elfin(robot_IP)
            success = self.robot.connect()

        elif robot_type == "elfin_new_api":
            self.robot = elfin.Elfin(robot_IP, use_new_api=True)
            success = self.robot.connect()

        elif robot_type == "dobot":
            self.robot = dobot.Dobot(robot_IP, robot_config=self.robot_config)
            success = self.robot.connect()

        elif robot_type == "ur":
            # TODO: Add Universal Robots robot here.
            pass

        elif robot_type == "test":
            # TODO: Add 'test' robot here.
            pass

        else:
            assert False, "Unknown robot model"

        if success:
            print('Connected to robot tracking device.')
        else:
            # Send message to neuronavigation to close the robot dialog.
            topic = 'Dialog robot destroy'
            data = {}
            self.remote_control.send_message(topic, data)

            self.robot = None
            print('Unable to connect to robot.')

        # Send message to neuronavigation indicating the state of connection.
        topic = 'Robot connection status'
        data = {'data': success}
        self.remote_control.send_message(topic, data)

    # TODO: Unused for now; all robots should have a similar logic and interface for reconnecting,
    #   that's why this function was moved here from Elfin robot class. However, how reconnecting
    #   works exactly needs to be thought through later.
    def ReconnectToRobot(self):
        topic = 'Update robot status'
        data = {'robot_status': False}
        self.remote_control.send_message(topic, data)

        while True:
            print("Trying to reconnect to robot...")
            success = self.robot.connect()
            if success:
                break

            time.sleep(1)

        self.robot.stop_robot()
        time.sleep(0.1)
        print("Reconnected!")

    def SensorUpdateTarget(self, distance, status):
        topic = 'Update target from FT values'
        data = {'data' : (distance, status)}

        self.status = False

        self.remote_control.send_message(topic, data)

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
        coord_robot_raw = self.robot.get_coordinates()
        self.robot_coordinates.SetRobotCoordinates(coord_robot_raw)

    def robot_motion_reset(self):
        self.robot.stop_robot()
        self.arc_motion_flag = False
        self.motion_type = MotionType.NORMAL

    def create_calibration_point(self):
        coord_raw, markers_flag = self.tracker_coordinates.GetCoordinates()
        coord_raw_robot = self.robot_coordinates.GetRobotCoordinates()
        coord_raw_tracker_obj = coord_raw[2]

        if markers_flag[2] and not any(coord is None for coord in coord_raw_robot):
            new_robot_coord_list = robot_process.coordinates_to_transformation_matrix(
                position=coord_raw_robot[:3],
                orientation=coord_raw_robot[3:],
                axes='sxyz',
            )

            # XXX: It would be preferable to use the same coordinate system everywhere - however, below
            #   the Euler angles are applied in the order z, y, x in a rotating frame ('r'). These rotation
            #   angles come from the neuronavigation, so ideally they would need to be changed there. Keeping it
            #   like it is for now.
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
            transformation_matrix_threshold = self.robot_config['transformation_matrix_threshold']
            if not np.allclose(np.array(current_tracker_coordinates_in_robot)[:3], np.array(coord_obj_tracker_in_robot)[:3], 0,
                               transformation_matrix_threshold):
                topic = 'Request new robot transformation matrix'
                data = {}
                self.remote_control.send_message(topic, data)
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
            This movement just happens when move distance is below a threshold ("robot_arc_threshold_distance" in robot config)

        2) arc motion:
            It can be divided into three parts.
                The first one represents the movement from the inner sphere to the outer sphere.
                 The robot moves back using a radial move (it use the center of the head as a reference).
                The second step is the actual arc motion (along the outer sphere).
                 A middle point, between the actual position and the target, is required.
                The last step is to make a linear move until the target (goes to the inner sphere)

        """
        distance_target = robot_process.correction_distance_calculation_target(tunning_to_target,
                                                                               current_robot_coordinates)
        distance_angle_target = robot_process.correction_distance_calculation_target(tunning_to_target[3:],
                                                                                     current_robot_coordinates[3:])
        # Check if the target is outside the working space. If so, return early.
        working_space = self.robot_config['working_space']
        if robot_process.estimate_robot_target_length(new_robot_coordinates) >= working_space:
            print("Head is too far from the robot basis")
            return False

        # Check the target distance to define the motion mode.
        arc_threshold_distance = self.robot_config['arc_threshold_distance']
        arc_threshold_angle = self.robot_config['arc_threshold_angle']
        if distance_target >= arc_threshold_distance or \
           distance_angle_target >= arc_threshold_angle:

            head_center_coordinates = self.process_tracker.estimate_head_center_in_robot(self.tracker_coordinates.m_tracker_to_robot, coord_head_tracker).tolist()

            versor_scale_factor = self.robot_config['versor_scale_factor']
            middle_arc_scale_factor = self.robot_config['middle_arc_scale_factor']
            linear_target, middle_arc_point, target_arc = robot_process.compute_arc_motion(
                current_robot_coordinates=current_robot_coordinates,
                head_center_coordinates=head_center_coordinates,
                new_robot_coordinates=new_robot_coordinates,  #needs to be new_robot_coordinates!!
                versor_scale_factor=versor_scale_factor,
                middle_arc_scale_factor=middle_arc_scale_factor,
            )
            if self.motion_type == MotionType.NORMAL:
                self.linear_target = linear_target
                self.motion_type = MotionType.LINEAR_OUT

            if self.motion_type == MotionType.LINEAR_OUT:
                if np.allclose(np.array(current_robot_coordinates), np.array(self.linear_target), 0, 10):
                    self.motion_type = MotionType.ARC
                    self.target_arc = target_arc

            elif self.motion_type == MotionType.ARC:
                #UPDATE arc motion target
                threshold_to_update_target_arc = self.robot_config['threshold_to_update_target_arc']
                if not np.allclose(np.array(target_arc), np.array(self.target_arc), 0, threshold_to_update_target_arc):
                    if robot_process.correction_distance_calculation_target(new_robot_coordinates, current_robot_coordinates) >= arc_threshold_distance:
                        self.target_arc = target_arc
                        #Avoid small arc motion
                    elif robot_process.correction_distance_calculation_target(target_arc, current_robot_coordinates) < arc_threshold_distance / 2:
                        self.motion_type = MotionType.NORMAL
                        self.target_arc = tunning_to_target

                if np.allclose(np.array(current_robot_coordinates)[:3], np.array(self.target_arc)[:3], 0, 20):
                    self.motion_type = MotionType.NORMAL
                    linear_target = tunning_to_target
            else:
                self.motion_type = MotionType.NORMAL
                linear_target = tunning_to_target
        else:
            self.motion_type = MotionType.NORMAL
            linear_target = tunning_to_target

        # Check if the robot is already in the target position. If so, return early.
        if self.target_reached:
            self.robot.set_target_reached(self.target_reached)
            return True

        target_tuning_threshold_angle = self.robot_config['target_tuning_threshold_angle']
        target_tuning_threshold_distance = self.robot_config['target_tuning_threshold_distance']
        if (np.sqrt(
                np.sum(np.square(self.displacement_to_target[:3]))) < target_tuning_threshold_distance or
            np.sqrt(
                np.sum(np.square(self.displacement_to_target[3:]))) < target_tuning_threshold_angle) \
                and self.motion_type != MotionType.ARC:
            # tunes the robot position based on neuronavigation
            self.motion_type = MotionType.TUNING
            self.inside_circle = True
            #print('Displacement to target: ', self.displacement_to_target)
            if const.FORCE_TORQUE_SENSOR and self.inside_circle and self.prev_state_flag == 1:
                self.force_ref = ft_values[0:3]
                self.moment_ref = ft_values[3:6]
                print('Normalised!')
                # print(self.force_ref)
            self.prev_state_flag = 0
        else:
            self.inside_circle = False
            self.prev_state_flag = 1

        # Branch to different movement functions, depending on the motion type.
        if self.motion_type == MotionType.NORMAL or \
           self.motion_type == MotionType.LINEAR_OUT:

            self.robot.move_linear(linear_target)

        elif self.motion_type == MotionType.ARC:
            start_position = current_robot_coordinates
            waypoint = middle_arc_point
            target = self.target_arc

            self.robot.move_circular(start_position, waypoint, target)

        elif self.motion_type == MotionType.TUNING:
            self.robot.tune_robot(self.displacement_to_target)

        elif self.motion_type == MotionType.FORCE_LINEAR_OUT:
            # TODO: Should this be implemented?
            pass

        self.robot.set_target_reached(self.target_reached)

        return True

    def read_force_sensor(self):
        success, force_sensor_values = self.robot.read_force_sensor()

        # If force sensor could not be read, return early.
        if not success:
            print("Error: Could not read the force sensor.")
            return

        # TODO: condition for const.FORCE_TORQUE_SENSOR
        ft_values = np.array(force_sensor_values)

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

        return ft_values

    def get_robot_status(self):
        robot_status = False
        current_tracker_coordinates, markers_flag = self.tracker_coordinates.GetCoordinates()
        if current_tracker_coordinates[1] is None:
            return robot_status
        marker_head_flag = markers_flag[1]
        marker_coil_flag = markers_flag[2]
        coord_head_tracker_filtered = self.process_tracker.kalman_filter(current_tracker_coordinates[1])

        current_robot_coordinates = self.robot_coordinates.GetRobotCoordinates()

        if const.FORCE_TORQUE_SENSOR:
            force_sensor_values = self.read_force_sensor()
        else:
            force_sensor_values = False

        if self.tracker_coordinates.m_tracker_to_robot is not None:
            coord_head_tracker_in_robot = robot_process.transform_tracker_to_robot(self.tracker_coordinates.m_tracker_to_robot, coord_head_tracker_filtered)
        else:
            coord_head_tracker_in_robot = coord_head_tracker_filtered

        #CHECK IF TARGET FROM INVESALIUS
        if self.robot_tracker_flag and np.all(self.m_change_robot_to_head[:3]):
            #self.check_robot_tracker_registration(current_robot_coordinates, coord_obj_tracker_in_robot, marker_obj_flag)
            #CHECK FORCE SENSOR
            force_sensor_threshold = self.robot_config['force_sensor_threshold']
            force_sensor_scale_threshold = self.robot_config['force_sensor_scale_threshold']
            if self.new_force_sensor_data <= force_sensor_threshold or \
               self.new_force_sensor_data <= (self.target_force_sensor_data + np.abs(self.target_force_sensor_data * (force_sensor_scale_threshold / 100))):

                #CHECK IF HEAD IS VISIBLE
                if coord_head_tracker_in_robot is not None and marker_head_flag and marker_coil_flag:
                    #CHECK HEAD VELOCITY
                    if self.process_tracker.compute_head_move_threshold(coord_head_tracker_in_robot):
                        new_robot_coordinates = robot_process.compute_head_move_compensation(coord_head_tracker_in_robot, self.m_change_robot_to_head)
                        # if const.FORCE_TORQUE_SENSOR and np.sqrt(np.sum(np.square(self.displacement_to_target[:3]))) < 10: # check if coil is 20mm from target and look for ft readout
                        #     if np.sqrt(np.sum(np.square(point_of_application[:2]))) > 0.5:
                        #         if self.status:
                        #             self.SensorUpdateTarget(distance, self.status)
                        #             self.status = False
                        tunning_to_target = self.OnTuneTCP()
                        robot_status = self.robot_move_decision(new_robot_coordinates, current_robot_coordinates,
                                                                coord_head_tracker_filtered, tunning_to_target, force_sensor_values)
                    else:
                        print("Head is moving too much")
                        self.robot.stop_robot()
                else:
                    print("Head marker is not visible")
                    self.robot.stop_robot()
            else:
                #print("Compensating Force")
                print(self.new_force_sensor_data)
                self.robot.compensate_force()
                time.sleep(0.5)
        else:
            #print("Navigation is off")
            pass

        return robot_status
