
import numpy as np
import cv2
from time import time

import robot.transformations as tr
import robot.constants as const

def coordinates_to_transformation_matrix(position, orientation, axes='sxyz'):
    """
    Transform vectors consisting of position and orientation (in Euler angles) in 3d-space into a 4x4
    transformation matrix that combines the rotation and translation.
    :param position: A vector of three coordinates.
    :param orientation: A vector of three Euler angles in degrees.
    :param axes: The order in which the rotations are done for the axes. See transformations.py for details. Defaults to 'sxyz'.
    :return: The transformation matrix (4x4).
    """
    a, b, g = np.radians(orientation)

    r_ref = tr.euler_matrix(a, b, g, axes=axes)
    t_ref = tr.translation_matrix(position)

    m_img = tr.concatenate_matrices(t_ref, r_ref)

    return m_img

def transformation_matrix_to_coordinates(matrix, axes='sxyz'):
    """
    Given a matrix that combines the rotation and translation, return the position and the orientation
    determined by the matrix. The orientation is given as three Euler angles.
    The inverse of coordinates_of_transformation_matrix when the parameter 'axes' matches.
    :param matrix: A 4x4 transformation matrix.
    :param axes: The order in which the rotations are done for the axes. See transformations.py for details. Defaults to 'sxyz'.
    :return: The position (a vector of length 3) and Euler angles for the orientation in degrees (a vector of length 3).
    """
    angles = tr.euler_from_matrix(matrix, axes=axes)
    angles_as_deg = np.degrees(angles)

    translation = tr.translation_from_matrix(matrix)

    return translation, angles_as_deg

def compute_marker_transformation(coord_raw, obj_ref_mode):
    m_probe = coordinates_to_transformation_matrix(
        position=coord_raw[obj_ref_mode, :3],
        orientation=coord_raw[obj_ref_mode, 3:],
        axes='rzyx',
    )
    return m_probe

def transformation_tracker_to_robot(m_tracker_to_robot, M_tracker_coord):
    X, Y, affine = m_tracker_to_robot

    M_tracker_in_robot = Y @ M_tracker_coord @ tr.inverse_matrix(X)
    M_affine_tracker_in_robot = affine @ M_tracker_coord

    _, angles_as_deg = transformation_matrix_to_coordinates(M_tracker_in_robot, axes='rzyx')
    translation, _ = transformation_matrix_to_coordinates(M_affine_tracker_in_robot, axes='rzyx')
    tracker_in_robot = list(translation) + list(angles_as_deg)

    return tracker_in_robot

def transform_tracker_to_robot(m_tracker_to_robot, coord_tracker):
    M_tracker_coord = coordinates_to_transformation_matrix(
        position=coord_tracker[:3],
        orientation=coord_tracker[3:6],
        axes='rzyx',
    )
    tracker_in_robot = transformation_tracker_to_robot(m_tracker_to_robot, M_tracker_coord)

    if tracker_in_robot is None:
        tracker_in_robot = coord_tracker

    return tracker_in_robot

def compute_robot_to_head_matrix(head_coordinates, robot_coordinates):
    """
    :param head: nx6 array of head coordinates from tracking device in robot space
    :param robot: nx6 array of robot coordinates

    :return: target_robot_matrix: 3x3 array representing change of basis from robot to head in robot system
    """
    # compute head target matrix
    m_head_target = coordinates_to_transformation_matrix(
        position=head_coordinates[:3],
        orientation=head_coordinates[3:],
        axes='rzyx',
    )

    # compute robot target matrix
    m_robot_target = coordinates_to_transformation_matrix(
        position=robot_coordinates[:3],
        orientation=robot_coordinates[3:],
        axes='rzyx',
    )
    robot_to_head_matrix = np.linalg.inv(m_head_target) @ m_robot_target

    return robot_to_head_matrix

def AffineTransformation(tracker, robot):
    m_change = tr.affine_matrix_from_points(robot[:].T, tracker[:].T,
                                            shear=False, scale=False, usesvd=False)
    return m_change

def estimate_head_velocity(coord_vel, timestamp):
    coord_vel = np.vstack(np.array(coord_vel))
    coord_init = coord_vel[:int(len(coord_vel) / 2)].mean(axis=0)
    coord_final = coord_vel[int(len(coord_vel) / 2):].mean(axis=0)
    velocity = (coord_final - coord_init)/(timestamp[-1] - timestamp[0])
    distance = (coord_final - coord_init)

    return velocity, distance

def compute_versors(init_point, final_point, scale=const.ROBOT_VERSOR_SCALE_FACTOR):
    init_point = np.array(init_point)
    final_point = np.array(final_point)
    norm = (sum((final_point - init_point) ** 2)) ** 0.5
    versor_factor = (((final_point-init_point) / norm) * scale).tolist()

    return versor_factor

def compute_arc_motion(current_robot_coordinates, head_center_coordinates, new_robot_coordinates):
    head_center = head_center_coordinates[0], head_center_coordinates[1], head_center_coordinates[2]

    versor_factor_move_out = compute_versors(head_center, current_robot_coordinates[:3])
    init_move_out_point = current_robot_coordinates[0] + versor_factor_move_out[0], \
                          current_robot_coordinates[1] + versor_factor_move_out[1], \
                          current_robot_coordinates[2] + versor_factor_move_out[2], \
                          current_robot_coordinates[5], current_robot_coordinates[4], current_robot_coordinates[3]

    middle_point = ((new_robot_coordinates[0] + current_robot_coordinates[0]) / 2,
                    (new_robot_coordinates[1] + current_robot_coordinates[1]) / 2,
                    (new_robot_coordinates[2] + current_robot_coordinates[2]) / 2)

    versor_factor_middle_arc = (np.array(compute_versors(head_center, middle_point))) * 1.5
    middle_arc_point = middle_point[0] + versor_factor_middle_arc[0], \
                       middle_point[1] + versor_factor_middle_arc[1], \
                       middle_point[2] + versor_factor_middle_arc[2]

    versor_factor_arc = compute_versors(head_center, new_robot_coordinates[:3])
    final_ext_arc_point = new_robot_coordinates[0] + versor_factor_arc[0], \
                          new_robot_coordinates[1] + versor_factor_arc[1], \
                          new_robot_coordinates[2] + versor_factor_arc[2], \
                          new_robot_coordinates[3], new_robot_coordinates[4], new_robot_coordinates[5], 0

    target_arc = middle_arc_point + final_ext_arc_point

    return init_move_out_point, target_arc

def correction_distance_calculation_target(coord_inv, actual_point):
    """
    Estimates the Euclidean distance between the actual position and the target
    """
    correction_distance_compensation = np.sqrt((coord_inv[0]-actual_point[0]) ** 2 +
                                               (coord_inv[1]-actual_point[1]) ** 2 +
                                               (coord_inv[2]-actual_point[2]) ** 2)

    return correction_distance_compensation

def compute_head_move_compensation(current_head, m_change_robot_to_head):
    """
    Estimates the new robot position to reach the target
    """
    M_current_head = coordinates_to_transformation_matrix(
        position=current_head[:3],
        orientation=current_head[3:6],
        axes='rzyx',
    )
    m_robot_new = M_current_head @ m_change_robot_to_head

    translation, angles_as_deg = transformation_matrix_to_coordinates(m_robot_new, axes='sxyz')
    new_robot_position = list(translation) + list(angles_as_deg)

    return new_robot_position

def estimate_robot_target_length(robot_target):
    """
    Estimates the length of the 3D vector of the robot target
    """
    robot_target_length = np.sqrt(robot_target[0] ** 2 + robot_target[1] ** 2 + robot_target[2] ** 2)

    return robot_target_length

#the class Transformation_matrix is based on elif.ayvali code @ https://github.com/eayvali/Pose-Estimation-for-Sensor-Calibration
class Transformation_matrix:
    def matrices_estimation(A, B):
        n = A.shape[2];
        T = np.zeros([9, 9]);
        X_est = np.eye(4)
        Y_est = np.eye(4)

        # Permutate A and B to get gross motions
        idx = np.random.permutation(n)
        A = A[:, :, idx];
        B = B[:, :, idx];

        for ii in range(n - 1):
            Ra = A[0:3, 0:3, ii]
            Rb = B[0:3, 0:3, ii]
            #  K[9*ii:9*(ii+1),:] = np.concatenate((np.kron(Rb,Ra), -np.eye(9)),axis=1)
            T = T + np.kron(Rb, Ra);

        U, S, Vt = np.linalg.svd(T)
        xp = Vt.T[:, 0]
        yp = U[:, 0]
        X = np.reshape(xp, (3, 3), order="F")
        Xn = (np.sign(np.linalg.det(X)) / np.abs(np.linalg.det(X)) ** (1 / 3)) * X
        # re-orthogonalize to guarantee that they are indeed rotations.
        U_n, S_n, Vt_n = np.linalg.svd(Xn)
        X = np.matmul(U_n, Vt_n)

        Y = np.reshape(yp, (3, 3), order="F")
        Yn = (np.sign(np.linalg.det(Y)) / np.abs(np.linalg.det(Y)) ** (1 / 3)) * Y
        U_yn, S_yn, Vt_yn = np.linalg.svd(Yn)
        Y = np.matmul(U_yn, Vt_yn)

        A_est = np.zeros([3 * n, 6])
        b_est = np.zeros([3 * n, 1])
        for ii in range(n - 1):
            A_est[3 * ii:3 * ii + 3, :] = np.concatenate((-A[0:3, 0:3, ii], np.eye(3)), axis=1)
            b_est[3 * ii:3 * ii + 3, :] = np.transpose(
                A[0:3, 3, ii] - np.matmul(np.kron(B[0:3, 3, ii].T, np.eye(3)), np.reshape(Y, (9, 1), order="F")).T)

        t_est_np = np.linalg.lstsq(A_est, b_est, rcond=None)
        if t_est_np[2] < A_est.shape[1]:  # A_est.shape[1]=6
            print('Rank deficient')
        t_est = t_est_np[0]
        X_est[0:3, 0:3] = X
        X_est[0:3, 3] = t_est[0:3].T
        Y_est[0:3, 0:3] = Y
        Y_est[0:3, 3] = t_est[3:6].T
        # verify Y_est using rigid_registration
        Y_est_check, ErrorStats = Transformation_matrix.__rigid_registration(A, X_est, B)
        return X_est, Y_est, Y_est_check, ErrorStats

    def __rigid_registration(A, X, B):
        # nxnx4
        """solves for Y in YB=AX
        A: (4x4xn)
        B: (4x4xn)
        X= (4x4)
        Y= (4x4)
        n number of measurements
        ErrorStats: Registration error (mean,std)
        """
        n = A.shape[2];
        AX = np.zeros(A.shape)
        AXp = np.zeros(A.shape)
        Bp = np.zeros(B.shape)
        pAX = np.zeros(B[0:3, 3, :].shape)  # To calculate reg error
        pYB = np.zeros(B[0:3, 3, :].shape)  # To calculate reg error
        Y_est = np.eye(4)

        ErrorStats = np.zeros((2, 1))

        for ii in range(n):
            AX[:, :, ii] = np.matmul(A[:, :, ii], X)

            # Centroid of transformations t and that
        t = 1 / n * np.sum(AX[0:3, 3, :], 1);
        that = 1 / n * np.sum(B[0:3, 3, :], 1);
        AXp[0:3, 3, :] = AX[0:3, 3, :] - np.tile(t[:, np.newaxis], (1, n))
        Bp[0:3, 3, :] = B[0:3, 3, :] - np.tile(that[:, np.newaxis], (1, n))

        [i, j, k] = AX.shape;  # 4x4xn
        # Convert AX and B to 2D arrays
        AXp_2D = AXp.reshape((i, j * k))  # now it is 4x(4xn)
        Bp_2D = Bp.reshape((i, j * k))  # 4x(4xn)
        # %Calculates the best rotation
        U, S, Vt = np.linalg.svd(np.matmul(Bp_2D[0:3, :], AXp_2D[0:3, :].T))  # v is v' in matlab
        R_est = np.matmul(Vt.T, U.T)
        # special reflection case
        if np.linalg.det(R_est) < 0:
            print('Warning: Y_est returned a reflection')
            R_est = np.matmul(Vt.T, np.matmul(np.diag([1, 1, -1]), U.T))
            # Calculates the best transformation
        t_est = t - np.dot(R_est, that)
        Y_est[0:3, 0:3] = R_est
        Y_est[0:3, 3] = t_est
        # Calculate registration error
        pYB = np.matmul(R_est, B[0:3, 3, :]) + np.tile(t_est[:, np.newaxis], (1, n))  # 3xn
        pAX = AX[0:3, 3, :]
        Reg_error = np.linalg.norm(pAX - pYB, axis=0)  # 1xn
        ErrorStats[0] = np.mean(Reg_error)
        ErrorStats[1] = np.std(Reg_error)
        return Y_est, ErrorStats

class KalmanTracker:
    """
    Kalman filter to avoid sudden fluctuation from tracker device.
    The filter strength can be set by the cov_process, and cov_measure parameter
    It is required to create one instance for each variable (x, y, z, a, b, g)
    """
    def __init__(self,
                 state_num=2,
                 covariance_process=0.001,
                 covariance_measure=0.1):

        self.state_num = state_num
        measure_num = 1

        # The filter itself.
        self.filter = cv2.KalmanFilter(state_num, measure_num, 0)

        self.state = np.zeros((state_num, 1), dtype=np.float32)
        self.measurement = np.array((measure_num, 1), np.float32)
        self.prediction = np.zeros((state_num, 1), np.float32)


        self.filter.transitionMatrix = np.array([[1, 1],
                                                 [0, 1]], np.float32)
        self.filter.measurementMatrix = np.array([[1, 1]], np.float32)
        self.filter.processNoiseCov = np.array([[1, 0],
                                                [0, 1]], np.float32) * covariance_process
        self.filter.measurementNoiseCov = np.array([[1]], np.float32) * covariance_measure

    def update_kalman(self, measurement):
        self.prediction = self.filter.predict()
        self.measurement = np.array([[np.float32(measurement[0])]])

        self.filter.correct(self.measurement)
        self.state = self.filter.statePost


class TrackerProcessing:
    def __init__(self):
        self.coord_vel = []
        self.timestamp = []
        self.velocity_vector = []
        self.kalman_coord_vector = []
        self.velocity_std = 0
        self.matrix_tracker_fiducials = 3*[None]

        self.tracker_stabilizers = [KalmanTracker(
            state_num=2,
            covariance_process=0.001,
            covariance_measure=0.1) for _ in range(6)]

    def SetMatrixTrackerFiducials(self, matrix_tracker_fiducials):
        self.matrix_tracker_fiducials = matrix_tracker_fiducials

    def kalman_filter(self, coord_tracker):
        kalman_array = []
        pose_np = np.array((coord_tracker[:3], coord_tracker[3:])).flatten()
        for value, ps_stb in zip(pose_np, self.tracker_stabilizers):
            ps_stb.update_kalman([value])
            kalman_array.append(ps_stb.state[0])
        coord_kalman = np.hstack(kalman_array)

        self.kalman_coord_vector.append(coord_kalman[:3])
        if len(self.kalman_coord_vector) < 20: #avoid initial fluctuations
            coord_kalman = coord_tracker
            print('initializing filter')
        else:
            del self.kalman_coord_vector[0]

        return coord_kalman

    def compute_head_move_threshold(self, current_ref):
        """
        Checks if the head velocity is bellow the threshold
        """
        self.coord_vel.append(current_ref)
        self.timestamp.append(time())
        if len(self.coord_vel) >= 10:
            head_velocity, head_distance = estimate_head_velocity(self.coord_vel, self.timestamp)
            self.velocity_vector.append(head_velocity)

            del self.coord_vel[0]
            del self.timestamp[0]

            if len(self.velocity_vector) >= 15:
                self.velocity_std = np.std(self.velocity_vector)
                del self.velocity_vector[0]

            if self.velocity_std > const.ROBOT_HEAD_VELOCITY_THRESHOLD:
                return False
            else:
                return True

        return True

    def estimate_head_center_in_robot(self, m_tracker_to_robot, current_head):
        """
        Estimates the actual head center position using fiducials
        """
        m_probe_head_left, m_probe_head_right, m_probe_head_nasion = self.matrix_tracker_fiducials
        m_current_head = compute_marker_transformation(np.array([current_head]), 0)

        m_ear_left_new = m_current_head @ m_probe_head_left
        m_ear_right_new = m_current_head @ m_probe_head_right

        X, Y, affine = m_tracker_to_robot
        m_ear_left_new_in_robot = affine @ m_ear_left_new
        m_ear_right_new_in_robot = affine @ m_ear_right_new

        center_head_in_robot = (m_ear_left_new_in_robot[:3, -1] + m_ear_right_new_in_robot[:3, -1])/2

        return center_head_in_robot

    def estimate_head_anterior_posterior_versor(self, m_tracker_to_robot, current_head, center_head_in_robot):
        """
        Estimates the actual anterior-posterior versor using nasion fiducial
        """
        _, _, m_probe_head_nasion = self.matrix_tracker_fiducials
        m_current_head = compute_marker_transformation(np.array([current_head]), 0)

        m_nasion_new = m_current_head @ m_probe_head_nasion

        X, Y, affine = m_tracker_to_robot
        m_nasion_new_in_robot = affine @ m_nasion_new

        head_anterior_posterior_versor = compute_versors(center_head_in_robot, m_nasion_new_in_robot[:3, -1], scale=1)

        return head_anterior_posterior_versor

    def estimate_head_left_right_versor(self, m_tracker_to_robot, current_head):
        """
        Estimates the actual left-right versor using fiducials
        """
        m_probe_head_left, m_probe_head_right, _ = self.matrix_tracker_fiducials
        m_current_head = compute_marker_transformation(np.array([current_head]), 0)

        m_ear_left_new = m_current_head @ m_probe_head_left
        m_ear_right_new = m_current_head @ m_probe_head_right

        X, Y, affine = m_tracker_to_robot
        m_ear_left_new_in_robot = affine @ m_ear_left_new
        m_ear_right_new_in_robot = affine @ m_ear_right_new

        head_left_right_versor = compute_versors(m_ear_left_new_in_robot[:3, -1], m_ear_right_new_in_robot[:3, -1], scale=1)

        return head_left_right_versor

    def estimate_robot_target(self,  tracker_coordinates,  m_target):
        coord_raw, markers_flag = tracker_coordinates.GetCoordinates()
        head_coordinates_in_tracker = coord_raw[1]

        target_in_robot = transformation_tracker_to_robot(tracker_coordinates.m_tracker_to_robot, m_target)
        head_coordinates_in_robot = transform_tracker_to_robot(tracker_coordinates.m_tracker_to_robot, head_coordinates_in_tracker)

        print("new target:", target_in_robot)

        return compute_robot_to_head_matrix(head_coordinates_in_robot, target_in_robot)

    def align_coil_with_head_center(self, tracker_coordinates, robot_coordinates):
        coord_raw, markers_flag = tracker_coordinates.GetCoordinates()
        coord_raw_robot = robot_coordinates.GetRobotCoordinates()
        head_coordinates_in_tracker = coord_raw[1]

        head_center_coordinates = self.estimate_head_center_in_robot(tracker_coordinates.m_tracker_to_robot,
                                                                     head_coordinates_in_tracker).tolist()

        initaxis = [0,0,1]
        newaxis = compute_versors(head_center_coordinates[:3], coord_raw_robot[:3], scale=1)
        crossvec = np.cross(initaxis, newaxis)
        angle = np.rad2deg(np.arccos(np.dot(initaxis, newaxis)))

        target_in_robot = coord_raw_robot.copy()
        target_in_robot[5] = target_in_robot[5] - (angle * crossvec[0])
        target_in_robot[4] = target_in_robot[4] - (angle * crossvec[1])
        target_in_robot[3] = target_in_robot[3] - (angle * crossvec[2])

        target_in_robot[3], target_in_robot[5] = target_in_robot[5], target_in_robot[3]

        return target_in_robot
