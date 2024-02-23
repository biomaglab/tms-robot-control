import numpy as np
from collections import deque
from pynput import keyboard
import time

import robot.constants as const
import robot.transformations as tr

import robot.robots.elfin.elfin as elfin
import robot.robots.dobot.dobot as dobot
import robot.control.coordinates as coordinates
import robot.control.ft as ft
import robot.control.robot_processing as robot_process

from robot.control.robot_state_controller import RobotStateController, RobotState
from robot.control.algorithms.radially_outward import RadiallyOutwardAlgorithm
from robot.control.algorithms.directly_upward import DirectlyUpwardAlgorithm


class RobotControl:
    def __init__(self, remote_control, config, site_config, robot_config):
        self.remote_control = remote_control

        self.config = config
        self.site_config = site_config
        self.robot_config = robot_config

        self.process_tracker = robot_process.TrackerProcessing(
            robot_config=robot_config,
        )

        self.robot_pose_storage = coordinates.RobotPoseStorage()
        self.tracker = coordinates.Tracker()

        self.robot = None

        self.tracker_coordinates = []
        self.robot_coordinates = []
        self.matrix_tracker_to_robot = []

        self.new_force_sensor_data = 0
        self.target_force_sensor_data = 5

        # reference force and moment values
        self.force_ref = np.array([0.0, 0.0, 0.0])
        self.moment_ref = np.array([0.0, 0.0, 0.0])
        self.tuning_ongoing = False
        self.F_dq = deque(maxlen=6)
        self.M_dq = deque(maxlen=6)

        self.REF_FLAG = False
        listener = keyboard.Listener(on_press=self._on_press)
        listener.start()
        #self.status = True

        self.target_set = False
        self.m_target_to_head = [None] * 9

        self.linear_out_target = None
        self.arc_motion_target = None
        self.previous_robot_status = False

        self.target_reached = False
        self.displacement_to_target = 6 * [0]
        self.ft_displacement_offset = [0, 0]

        self.robot_coord_matrix_list = np.zeros((4, 4))[np.newaxis]
        self.coord_coil_matrix_list = np.zeros((4, 4))[np.newaxis]

        self.last_displacement_update_time = None
        self.last_robot_status_logging_time = time.time()

    def OnRobotConnection(self, data):
        robot_IP = data["robot_IP"]
        self.ConnectToRobot(robot_IP)

    def OnUpdateRobotTargetMatrix(self, data):
        if self.robot:
            # XXX: The variable received from neuronavigation is called 'robot_tracker_flag', which is confusing.
            self.target_set = data["robot_tracker_flag"]
            target = data["target"]
            if not self.target_set:
                # If target is removed mid-movement, the current movement is rendered invalid; hence, stop the robot.
                self.robot.stop_robot()

                self.m_target_to_head = [None] * 9
                self.target_force_sensor_data = 5
                print("Target removed")
            else:
                target = np.array(target).reshape(4, 4)
                self.m_target_to_head = self.process_tracker.compute_transformation_target_to_head(self.tracker, target)
                self.target_force_sensor_data = self.new_force_sensor_data

                # If target changes mid-movement, the current movement is rendered invalid; hence, stop the robot and reset the state
                # of the movement algorithm to ensure that the next movement starts from a known, well-defined state.
                self.robot.stop_robot()
                self.movement_algorithm.reset_state()

                print("Target set")

    def OnResetProcessTracker(self, data):
        # TODO: This shouldn't call the constructor again but instead a separate reset method.
        self.process_tracker.__init__(
            robot_config=self.robot_config
        )

    def OnUpdateCoordinates(self, data):
        if len(data) > 1:
            coord = data["coord"]
            markers_flag = data["markers_flag"]
            self.tracker.SetCoordinates(np.vstack([coord[0], coord[1], coord[2]]), markers_flag)

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
        self.tracker_coordinates = []
        self.robot_coordinates = []
        self.matrix_tracker_to_robot = []

    def OnRobotMatrixEstimation(self, data=None):
        try:
            affine_matrix_robot_to_tracker = robot_process.AffineTransformation(
                np.array(self.tracker_coordinates),
                np.array(self.robot_coordinates)
            )
            affine_matrix_tracker_to_robot = tr.inverse_matrix(affine_matrix_robot_to_tracker)

            robot_coordinates = np.stack(self.robot_coord_matrix_list[1:], axis=2)
            coord_coil_list = np.stack(self.coord_coil_matrix_list[1:], axis=2)

            # Estimating the transformation matrix between the tracker and the robot includes randomness; ensure that the
            # results are reproducible.
            np.random.seed(1)

            X_est, Y_est, Y_est_check, ErrorStats = robot_process.Transformation_matrix.matrices_estimation(robot_coordinates, coord_coil_list)
            print(robot_coordinates[:4, :4, -1][:3, 3].T - (Y_est @ coord_coil_list[:4, :4, -1] @ tr.inverse_matrix(X_est))[:3, 3].T)
            print(robot_coordinates[:4, :4, -1][:3, 3].T - (affine_matrix_tracker_to_robot @ coord_coil_list[:4, :4, -1])[:3, 3].T)
            # print(X_est)
            # print(Y_est)
            # print(Y_est_check)
            print(ErrorStats)
            self.matrix_tracker_to_robot = X_est, Y_est, affine_matrix_tracker_to_robot
            self.tracker.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

            topic = 'Update robot transformation matrix'
            data = {'data': np.hstack(np.concatenate((X_est, Y_est, affine_matrix_tracker_to_robot))).tolist()}
            self.remote_control.send_message(topic, data)

        except np.linalg.LinAlgError:
            print("numpy.linalg.LinAlgError")
            print("Try a new acquisition")

    def OnLoadRobotMatrix(self, data):
        X_est, Y_est, affine_matrix_tracker_to_robot = np.split(np.array(data["data"]).reshape(12, 4), 3, axis=0)
        self.matrix_tracker_to_robot = X_est, Y_est, affine_matrix_tracker_to_robot
        self.tracker.SetTrackerToRobotMatrix(self.matrix_tracker_to_robot)

    def OnCoilToRobotAlignment(self, displacement):
        # XXX: Why does the transformation done by this function exist in the first place? The displacement is received
        #   from neuronavigation in terms of the TCP coordinate system, and it is used to estimate the target position in robot
        #   space using the current robot pose and the displacement.
        #
        #   This function essentially transforms the displacement to the end effector coordinate system. However, it is only needed
        #   if robot pose was received from the robot in terms of the end effector coordinate system, but at least in case of Elfin
        #   the robot pose is in terms of TCP coordinate system, making the transformation done here seemingly incorrect (if the
        #   rx, ry, and rz offsets differ from zero - if they are zero, this transformation does not do anything.)
        xaxis, yaxis, zaxis = [1, 0, 0], [0, 1, 0], [0, 0, 1]

        rx_offset = self.site_config['rx_offset']
        ry_offset = self.site_config['ry_offset']
        rz_offset = self.site_config['rz_offset']

        Rx = tr.rotation_matrix(np.radians(rx_offset), xaxis)
        Ry = tr.rotation_matrix(np.radians(ry_offset), yaxis)
        Rz = tr.rotation_matrix(np.radians(rz_offset), zaxis)

        rotation_alignment_matrix = tr.multiply_matrices(Rx, Ry, Rz)

        m_offset = robot_process.coordinates_to_transformation_matrix(
            position=displacement[:3],
            orientation=displacement[3:],
            axes='sxyz',
        )
        displacement_matrix = np.linalg.inv(rotation_alignment_matrix) @ m_offset @ rotation_alignment_matrix

        return robot_process.transformation_matrix_to_coordinates(displacement_matrix, axes='sxyz')

    def compute_target_in_robot_space(self):
        robot_pose = self.robot_pose_storage.GetRobotPose()
        m_robot = robot_process.coordinates_to_transformation_matrix(
            position=robot_pose[:3],
            orientation=robot_pose[3:],
            axes='sxyz',
        )

        # XXX: The code below essentially copies the code from coordinates_to_transformation_matrix function, except that the order
        #   of rotation and translation is reversed (in the code below it is: rotation first, then translation). However,
        #   we should stick with one convention, hence the coordinates_to_transformation_matrix is not modified to allow
        #   using two different conventions. The correct solution would be to change neuronavigation so that the displacement
        #   received from the there follows the same convention as used elsewhere in this code and most likely in neuronavigation
        #   as well.
        a, b, g = np.radians(self.displacement_to_target[3:])

        # XXX: The order of axis rotations in the displacement received from neuronavigation is: rx, ry, rz in a rotating frame ('rxyz').
        #   Hence, use that when generating the rotation matrix, even though 'sxyz' (equivalent to 'rzyx') is the convention used in the
        #   rest of the code.
        r_ref = tr.euler_matrix(a, b, g, axes='rxyz')
        t_ref = tr.translation_matrix(self.displacement_to_target[:3])

        # XXX: First rotate, then translate. This is done because displacement received from neuronavigation uses that order.
        m_offset = tr.multiply_matrices(r_ref, t_ref)

        m_final = m_robot @ m_offset
        translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

        return list(translation) + list(angles_as_deg)

    def OnDistanceToTarget(self, data):
        # For the displacement received from the neuronavigation, the following is true:
        #
        # - It is a vector from the current robot pose to the target pose.
        # - It is represented as the vector (Dx, Dy, Dz, Da, Db, Dc) in the coordinate system of the tool center point (TCP), where
        #   Dx, Dy, and Dz are the differences along x-, y-, and z-axes, and Da, Db, Dc are the differences in Euler angles between the
        #   current pose and the target pose.
        # - The Euler angles are applied in the order of rx, ry, rz, i.e., the rotation around x-axis is performed first, followed by
        #   rotation around y-axis, and finally rotation around z-axis.
        # - The rotations are applied in this order in a rotating frame ('rxyz'), not a static frame.
        # - Contrary to the common convention and the convention used elsewhere in the code, the rotation is applied _before_ translation.

        # XXX: The variable received from neuronavigation is called 'distance', but displacement is a more accurate name.
        displacement = data["distance"]

        # XXX: The handedness of the coordinate system used by neuronavigation is different from the one used by the robot,
        #   hence the sign reversal below.
        #
        # The exact axes (x- and rx-axes) that are reversed are determined using the observations below:
        #
        # Starting from the target (i.e., displacement = [0, 0, 0, 0, 0, 0]), move the robot in the positive x-direction by 10 mm.
        # The displacement is a vector from the current robot position to the target position, so the displacement should
        # become [-10, 0, 0, 0, 0, 0]. However, the displacement received from neuronavigation is [10, 0, 0, 0, 0, 0].
        # The same applies for rx-axis (but not for the other axes) - hence the sign reversal for x- and rx-axes.
        #
        displacement[0] = -displacement[0]
        displacement[3] = -displacement[3]

        translation, angles_as_deg = self.OnCoilToRobotAlignment(displacement)
        translation[0] += self.ft_displacement_offset[0]
        translation[1] += self.ft_displacement_offset[1]
        self.displacement_to_target = list(translation) + list(angles_as_deg)

        if self.last_displacement_update_time is not None:
            print("Displacement received: {} (time since last: {:.2f} s)".format(
                ", ".join(["{:.2f}".format(x) for x in self.displacement_to_target]),
                time.time() - self.last_displacement_update_time,
            ))

        self.last_displacement_update_time = time.time()

    def OnCoilAtTarget(self, data):
        self.target_reached = data["state"]
        if self.robot is not None:
            self.robot.set_target_reached(self.target_reached)

    def ConnectToRobot(self, robot_IP):
        robot_type = self.config['robot']
        print("Trying to connect to robot '{}' with IP: {}".format(robot_type, robot_IP))

        if robot_type == "elfin":
            robot = elfin.Elfin(robot_IP, config=self.config)
            success = robot.connect()

        elif robot_type == "elfin_new_api":
            robot = elfin.Elfin(robot_IP, config=self.config, use_new_api=True)
            success = robot.connect()

        elif robot_type == "dobot":
            robot = dobot.Dobot(robot_IP, config=self.config, robot_config=self.robot_config)
            success = robot.connect()

        elif robot_type == "ur":
            # TODO: Add Universal Robots robot here.
            pass

        elif robot_type == "test":
            # TODO: Add 'test' robot here.
            pass

        else:
            assert False, "Unknown robot model"

        if success:
            print('Connected to robot.')

            # Initialize the robot.
            robot.initialize()
            print('Robot initialized.')

            # Initialize the robot state controller.
            dwell_time = self.config['dwell_time']
            self.robot_state_controller = RobotStateController(
                robot=robot,
                dwell_time=dwell_time,
            )

            self.robot = robot

            movement_algorithm_name = self.config['movement_algorithm']
            if movement_algorithm_name == 'radially_outward':
                self.movement_algorithm = RadiallyOutwardAlgorithm(
                    robot=robot,
                    config=self.config,
                    robot_config=self.robot_config,
                )

            elif movement_algorithm_name == 'directly_upward':
                self.movement_algorithm = DirectlyUpwardAlgorithm(
                    robot=robot,
                    config=self.config,
                    robot_config=self.robot_config,
                )

            else:
                assert False, "Unknown movement algorithm"

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

    def update_robot_pose(self):
        success, robot_pose = self.robot.get_pose()

        # Only update the robot pose if the robot pose could be read.
        if success:
            self.robot_pose_storage.SetRobotPose(robot_pose)

    def create_calibration_point(self):
        coil_visible = self.tracker.coil_visible
        coil_pose = self.tracker.coil_pose

        robot_pose = self.robot_pose_storage.GetRobotPose()

        if coil_visible and not any(coord is None for coord in robot_pose):
            new_robot_coordinates = robot_process.coordinates_to_transformation_matrix(
                position=robot_pose[:3],
                orientation=robot_pose[3:],
                axes='sxyz',
            )
            new_coord_coil_list = np.array(robot_process.coordinates_to_transformation_matrix(
                position=coil_pose[:3],
                orientation=coil_pose[3:],
                axes='sxyz',
            ))

            self.robot_coord_matrix_list = np.vstack([self.robot_coord_matrix_list.copy(), new_robot_coordinates[np.newaxis]])
            self.coord_coil_matrix_list = np.vstack([self.coord_coil_matrix_list.copy(), new_coord_coil_list[np.newaxis]])

            self.tracker_coordinates.append(coil_pose[:3])
            self.robot_coordinates.append(robot_pose[:3])

            return True
        else:
            print('Cannot collect the coil markers, please try again')
            return False

    # Unused and bitrotten for now.
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

    def read_force_sensor(self):
        success, force_sensor_values = self.robot.read_force_sensor()

        # If force sensor could not be read, return early.
        if not success:
            print("Error: Could not read the force sensor.")
            return

        # TODO: condition for self.config['use_force_sensor']

        # true f-t value
        current_F = force_sensor_values[0:3]
        current_M = force_sensor_values[3:6]
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
        # self.ft_displacement.offset = [point_of_application[0], point_of_application[1]]
        # TODO: Change this entire part to compensate force properly in a feedback loop

        return force_sensor_values

    def compensate_force(self):
        """
        Compensate the force by moving the robot in the negative z-direction by 2 mm.
        """
        # TODO: Are these checks actually needed?
        if self.robot.is_moving() or self.robot.is_error_state():
            return

        # Get the current robot pose.
        success, robot_pose = self.robot.get_pose()
        if not success:
            print("Error: Could not read the robot pose.")
            return

        # Create the transformation matrix for the robot pose.
        m_robot = robot_process.coordinates_to_transformation_matrix(
            position=robot_pose[:3],
            orientation=robot_pose[3:],
            axes='sxyz',
        )

        # Create the compensation vector that points 2 mm to the negative z-direction.
        compensation = [0, 0, -2, 0, 0, 0]

        # Create the transformation matrix for the compensation.
        m_compensation = robot_process.coordinates_to_transformation_matrix(
            position=compensation[:3],
            orientation=compensation[3:],
            axes='sxyz',
        )

        # Compute the final transformation matrix.
        m_final = m_robot @ m_compensation

        # Convert the transformation matrix to coordinates.
        translation, angles_as_deg = robot_process.transformation_matrix_to_coordinates(m_final, axes='sxyz')

        target_pose = list(translation) + list(angles_as_deg)

        # Move the robot to the target pose.
        success = self.robot.move_linear(target_pose)

        if not success:
            print("Error: Could not compensate the force.")
            return

        # Wait for the compensation to finish.
        time.sleep(0.5)
        return

    def get_robot_status(self):
        # Update the robot state.
        self.robot_state_controller.update()

        # Print the robot state once every 0.1 seconds.
        if self.last_robot_status_logging_time is not None and time.time() - self.last_robot_status_logging_time > 0.1:
            self.robot_state_controller.print_state()
            self.last_robot_status_logging_time = time.time()

        head_pose_in_tracker_space = self.tracker.head_pose
        if head_pose_in_tracker_space is None:
            return False

        head_visible = self.tracker.head_visible
        coil_visible = self.tracker.coil_visible

        head_pose_in_tracker_space_filtered = self.process_tracker.kalman_filter(head_pose_in_tracker_space)

        robot_pose = self.robot_pose_storage.GetRobotPose()

        if self.config['use_force_sensor']:
            force_sensor_values = self.read_force_sensor()
        else:
            force_sensor_values = False

        if self.tracker.m_tracker_to_robot is not None:
            head_pose_in_robot_space = self.tracker.transform_pose_to_robot_space(head_pose_in_tracker_space_filtered)
        else:
            # XXX: This doesn't seem correct: if the transformation to robot space is not available, we should not
            #   claim that the head pose in tracker space is in robot space and use it as such.
            head_pose_in_robot_space = head_pose_in_tracker_space_filtered

        # If target has not been received, return early.
        if not self.target_set or not np.all(self.m_target_to_head[:3]):
            #print("Navigation is off")
            return False

        #self.check_robot_tracker_registration(robot_pose, coord_obj_tracker_in_robot, marker_obj_flag)

        # Check force sensor.
        force_sensor_threshold = self.robot_config['force_sensor_threshold']
        force_sensor_scale_threshold = self.robot_config['force_sensor_scale_threshold']
        if self.new_force_sensor_data > force_sensor_threshold and \
            self.new_force_sensor_data > (self.target_force_sensor_data + np.abs(self.target_force_sensor_data * (force_sensor_scale_threshold / 100))):

            #print("Compensating Force")
            print(self.new_force_sensor_data)

            self.compensate_force()

            return False

        # Check if head is visible.
        if head_pose_in_robot_space is None or not head_visible or not coil_visible:
            if self.config['stop_robot_if_head_not_visible']:
                print("Warning: Head marker is not visible, stopping the robot")

                # Stop the robot. This is done because if the head marker is not visible, we cannot trust that the ongoing
                # movement does not collide with the head.
                self.robot.stop_robot()

                # Reset the state of the movement algorithm. This is done because the movement algorithm may have trouble
                # resuming the state after the robot is stopped - this is the case for 'directly upward' algorithm - hence,
                # it's better to continue from a known, well-defined state.
                self.movement_algorithm.reset_state()

                return False

            print("Warning: Head marker is not visible")

        # Check that the head is not moving too fast.
        if self.process_tracker.is_head_moving_too_fast(head_pose_in_robot_space):
            print("Warning: Head is moving too fast, stopping the robot")

            # Stop the robot. This is done because if the head is moving too fast, we cannot trust that the ongoing
            # movement does not collide with the head.
            self.robot.stop_robot()

            # Reset the state of the movement algorithm. This is done because the movement algorithm may have trouble
            # resuming the state after the robot is stopped - this is the case for 'directly upward' algorithm - hence,
            # it's better to continue from a known, well-defined state.
            self.movement_algorithm.reset_state()

            return False

        # Check if the robot is ready to move.
        if self.robot_state_controller.get_state() != RobotState.READY:

            # Return True even if the robot is not ready to move; the return value is used to indicate
            # that the robot is generally in a good state.
            return True

        # Check if the robot is already in the target position.
        if self.target_reached:
            # Return True if the robot is already in the target position; the return value is used to indicate
            # that the robot is in a good state.
            return True

        # Check if the displacement to the target is available.
        if self.displacement_to_target is None:

            # Even though a recent displacement should be always available, it turns out that the 0.2 second time limit
            # is quite strict. Hence, interpret the lack of displacement as a "good state".
            return True

        # Ensure that the displacement to target has been updated recently.
        if time.time() > self.last_displacement_update_time + 0.2:
            print("No displacement update received for 0.2 seconds")
            self.displacement_to_target = None
            return True

        target_pose_in_robot_space_estimated_from_head_pose = robot_process.compute_head_move_compensation(head_pose_in_robot_space, self.m_target_to_head)

        # Check if the target is outside the working space. If so, return early.
        working_space = self.robot_config['working_space']
        if np.linalg.norm(target_pose_in_robot_space_estimated_from_head_pose[:3]) >= working_space:
            print("Head is too far from the robot basis")
            return False

        # if self.config['use_force_sensor'] and np.sqrt(np.sum(np.square(self.displacement_to_target[:3]))) < 10: # check if coil is 20mm from target and look for ft readout
        #     if np.sqrt(np.sum(np.square(point_of_application[:2]))) > 0.5:
        #         if self.status:
        #             self.SensorUpdateTarget(distance, self.status)
        #             self.status = False

        # Compute the target pose in robot space.
        target_pose_in_robot_space_estimated_from_displacement = self.compute_target_in_robot_space()

        # Compute the head center in robot space.
        head_center = self.process_tracker.estimate_head_center_in_robot_space(
            self.tracker.m_tracker_to_robot,
            head_pose_in_tracker_space_filtered).tolist()

        # Move the robot.
        success, normalize_force_sensor = self.movement_algorithm.move_decision(
            displacement_to_target=self.displacement_to_target,
            target_pose_in_robot_space_estimated_from_head_pose=target_pose_in_robot_space_estimated_from_head_pose,
            target_pose_in_robot_space_estimated_from_displacement=target_pose_in_robot_space_estimated_from_displacement,
            robot_pose=robot_pose,
            head_center=head_center,
        )

        # Normalize force sensor values if needed.
        use_force_sensor = self.config['use_force_sensor']
        if use_force_sensor and normalize_force_sensor:
            self.force_ref = force_sensor_values[0:3]
            self.moment_ref = force_sensor_values[3:6]
            print('Normalised!')

        # TODO: Dobot has internal methods to check and control if the robot is moving and the controller was making
        #  dobot have weird behavior. The best approach would be to use the controller instead of the internal methods.
        #  By now, for dobot, its requires that the self.config['dwell_time'] is equal to zero.

        # Set the robot state to moving if the movement was successful and the dwell_time is different from zero.
        if success and self.config['dwell_time']:
            self.robot_state_controller.set_state_to_moving()

        return success
