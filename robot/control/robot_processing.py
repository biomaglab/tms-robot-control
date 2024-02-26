
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

    m_img = tr.multiply_matrices(t_ref, r_ref)

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
        axes='sxyz',
    )
    return m_probe

def compute_transformation_to_head_space(pose, head_pose):
    """
    :param pose: nx6 array of a pose in robot space
    :param head: nx6 array of a head pose in robot space

    :return: 4x4 array representing change of basis from the given pose to head's coordinate system.
    """
    m_head_pose = coordinates_to_transformation_matrix(
        position=head_pose[:3],
        orientation=head_pose[3:],
        axes='sxyz',
    )
    m_pose = coordinates_to_transformation_matrix(
        position=pose[:3],
        orientation=pose[3:],
        axes='sxyz',
    )
    to_head = np.linalg.inv(m_head_pose) @ m_pose

    return to_head

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

def compute_versor(init_point, final_point, scale):
    init_point = np.array(init_point)
    final_point = np.array(final_point)
    norm = (sum((final_point - init_point) ** 2)) ** 0.5
    versor_factor = (((final_point-init_point) / norm) * scale).tolist()

    return versor_factor

def compute_arc_motion(robot_pose, head_center, target_pose, versor_scale_factor, middle_arc_scale_factor):
    # XXX: Is this actually needed? Head center should already be a three-element list.
    head_center_ = head_center[0], head_center[1], head_center[2]

    versor_factor_move_out = compute_versor(
        init_point=head_center_,
        final_point=robot_pose[:3],
        scale=versor_scale_factor,
    )
    init_move_out_point = robot_pose[0] + versor_factor_move_out[0], \
                          robot_pose[1] + versor_factor_move_out[1], \
                          robot_pose[2] + versor_factor_move_out[2], \
                          robot_pose[3], robot_pose[4], robot_pose[5]

    middle_point = ((target_pose[0] + robot_pose[0]) / 2,
                    (target_pose[1] + robot_pose[1]) / 2,
                    (target_pose[2] + robot_pose[2]) / 2)

    versors = compute_versor(
        init_point=head_center_,
        final_point=middle_point,
        scale=versor_scale_factor,
    )
    versor_factor_middle_arc = np.array(versors) * middle_arc_scale_factor
    middle_arc_point = middle_point[0] + versor_factor_middle_arc[0], \
                       middle_point[1] + versor_factor_middle_arc[1], \
                       middle_point[2] + versor_factor_middle_arc[2], \
                       target_pose[3], target_pose[4], target_pose[5]

    versor_factor_arc = compute_versor(
        init_point=head_center_,
        final_point=target_pose[:3],
        scale=versor_scale_factor,
    )
    final_ext_arc_point = target_pose[0] + versor_factor_arc[0], \
                          target_pose[1] + versor_factor_arc[1], \
                          target_pose[2] + versor_factor_arc[2], \
                          target_pose[3], target_pose[4], target_pose[5]

    return init_move_out_point, middle_arc_point, final_ext_arc_point

def compute_head_move_compensation(head_pose_in_robot_space, m_target_to_head):
    """
    Estimates the new robot position to reach the target
    """
    m_head = coordinates_to_transformation_matrix(
        position=head_pose_in_robot_space[:3],
        orientation=head_pose_in_robot_space[3:6],
        axes='sxyz',
    )
    m_target_new = m_head @ m_target_to_head

    translation, angles_as_deg = transformation_matrix_to_coordinates(m_target_new, axes='sxyz')
    new_robot_position = list(translation) + list(angles_as_deg)

    return new_robot_position

# Unused for now.
def compute_transformation_tcp_to_head(tracker, robot_pose_storage):
    head_pose_in_tracker_space = tracker.get_head_pose()
    robot_pose = robot_pose_storage.GetRobotPose()

    head_pose_in_robot_space = tracker.transform_pose_to_robot_space(head_pose_in_tracker_space)

    return compute_transformation_to_head_space(
        pose=robot_pose,
        head_pose=head_pose_in_robot_space
    )

def bezier_curve(points, step):
    """
    Code based on https://github.com/torresjrjr/Bezier.py
    Returns a point interpolated by the Bezier process for arc motion
    INPUTS:
        t_values     list of floats/ints
        points       list of numpy arrays
    OUTPUTS:
        curve        list of numpy arrays
    """
    # Defines the number of points along the path.
    t_values = np.arange(0, 1, step)
    t_values = np.append(t_values, 1)
    curve = []
    init_angles = points[0, 3:]
    target_angles = points[2, 3:]

    for t in t_values:
        newpoints = np.array(points)
        while len(newpoints) > 1:
            newpoints_aux = (1 - t) * newpoints[:-1] + t * newpoints[1:]
            newpoints = newpoints_aux
        if t <= 0.7:
            newpoints[0][3], newpoints[0][4], newpoints[0][5] = init_angles[0], init_angles[1], init_angles[2]
        # elif t <= 0.5:
        #     newpoints[0][3], newpoints[0][4], newpoints[0][5] = init_angles[0], target_angles[1], target_angles[2]
        else:
            newpoints[0][3], newpoints[0][4], newpoints[0][5] = target_angles[0], target_angles[1], target_angles[2]
        curve.append(newpoints[0])

    return np.array(curve)

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
    def __init__(self, robot_config):
        self.robot_config = robot_config

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
        else:
            del self.kalman_coord_vector[0]

        return coord_kalman

    def is_head_moving_too_fast(self, current_ref):
        """
        Check if the head velocity is above the threshold. If yes, return True, otherwise False.
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

            head_velocity_threshold = self.robot_config['head_velocity_threshold']
            if self.velocity_std > head_velocity_threshold:
                self.coord_vel = []
                self.timestamp = []
                return True
            else:
                self.coord_vel = []
                self.timestamp = []
                return False

        return False

    def estimate_head_center_in_robot_space(self, m_tracker_to_robot, head_pose_in_tracker_space):
        """
        Estimates the actual head center position in robot space as the average of the positions of
        the left ear and right ear, using the fiducials registered in neuronavigation.
        """
        m_probe_head_left, m_probe_head_right, m_probe_head_nasion = self.matrix_tracker_fiducials

        # Check that all fiducials are available.
        if None in self.matrix_tracker_fiducials:
            return None

        m_head = compute_marker_transformation(np.array([head_pose_in_tracker_space]), 0)

        m_ear_left_new = m_head @ m_probe_head_left
        m_ear_right_new = m_head @ m_probe_head_right

        X, Y, affine = m_tracker_to_robot
        m_ear_left_new_in_robot_space = affine @ m_ear_left_new
        m_ear_right_new_in_robot_space = affine @ m_ear_right_new

        center_head_in_robot_space = (m_ear_left_new_in_robot_space[:3, -1] + m_ear_right_new_in_robot_space[:3, -1])/2

        return center_head_in_robot_space.tolist()

    def estimate_head_anterior_posterior_versor(self, m_tracker_to_robot, current_head, center_head_in_robot):
        """
        Estimates the actual anterior-posterior versor using nasion fiducial
        """
        _, _, m_probe_head_nasion = self.matrix_tracker_fiducials
        m_current_head = compute_marker_transformation(np.array([current_head]), 0)

        m_nasion_new = m_current_head @ m_probe_head_nasion

        X, Y, affine = m_tracker_to_robot
        m_nasion_new_in_robot = affine @ m_nasion_new

        head_anterior_posterior_versor = compute_versor(center_head_in_robot, m_nasion_new_in_robot[:3, -1], scale=1)

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

        head_left_right_versor = compute_versor(m_ear_left_new_in_robot[:3, -1], m_ear_right_new_in_robot[:3, -1], scale=1)

        return head_left_right_versor

    def compute_transformation_target_to_head(self, tracker, m_target):
        head_pose_in_tracker_space = tracker.get_head_pose()

        target_pose_in_robot_space = tracker.transform_matrix_to_robot_space(m_target)
        head_pose_in_robot_space = tracker.transform_pose_to_robot_space(head_pose_in_tracker_space)

        print("Update target based on InVesalius:", target_pose_in_robot_space)

        return compute_transformation_to_head_space(
            pose=target_pose_in_robot_space,
            head_pose=head_pose_in_robot_space,
        )

    # Unused for now.
    def align_coil_with_head_center(self, tracker, robot_pose_storage):
        head_pose_in_tracker_space = tracker.get_head_pose()
        robot_pose = robot_pose_storage.GetRobotPose()

        head_center_coordinates = self.estimate_head_center_in_robot_space(
            tracker.m_tracker_to_robot,
            head_pose_in_tracker_space).tolist()

        initaxis = [0,0,1]
        newaxis = compute_versor(head_center_coordinates[:3], robot_pose[:3], scale=1)
        crossvec = np.cross(initaxis, newaxis)
        angle = np.rad2deg(np.arccos(np.dot(initaxis, newaxis)))

        target_pose_in_robot_space = robot_pose.copy()
        target_pose_in_robot_space[3] = target_pose_in_robot_space[3] - (angle * crossvec[0])
        target_pose_in_robot_space[4] = target_pose_in_robot_space[4] - (angle * crossvec[1])
        target_pose_in_robot_space[5] = target_pose_in_robot_space[5] - (angle * crossvec[2])

        return target_pose_in_robot_space
