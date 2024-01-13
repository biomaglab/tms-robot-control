from enum import Enum

import numpy as np

import robot.control.robot_processing as robot_process


class MotionType(Enum):
    NORMAL = 0
    LINEAR_OUT = 1
    ARC = 2
    FORCE_LINEAR_OUT = 3
    TUNING = 4


class RadiallyOutwardsAlgorithm:
    def __init__(self, robot, robot_config):
        self.robot = robot
        self.robot_config = robot_config

        self.motion_type = MotionType.NORMAL
        self.tuning_ongoing = False

        self.linear_out_target = None
        self.arc_motion_target = None

    def reset(self):
        self.motion_type = MotionType.NORMAL
        self.tuning_ongoing = False

        self.linear_out_target = None
        self.arc_motion_target = None

    def move_decision(self,
                      displacement_to_target,
                      target_pose_in_robot_space_estimated_from_head_pose,
                      target_pose_in_robot_space_estimated_from_displacement,
                      robot_pose,
                      head_center):
        """
        There are two types of robot movements.

        We can imagine in two concentric spheres of different sizes. The inside sphere is to compensate for small head movements.
         It was named "normal" moves.

        The outside sphere is for the arc motion. The arc motion is a safety feature for long robot movements.
         Even for a new target or a sudden huge head movement.

        1) normal:
            A linear move from the actual position until the target position.
            This movement just happens when move distance is below a threshold ("distance_threshold_for_arc_motion" in robot config)

        2) arc motion:
            It can be divided into three parts.
                The first one represents the movement from the inner sphere to the outer sphere.
                 The robot moves back using a radial move (it use the center of the head as a reference).
                The second step is the actual arc motion (along the outer sphere).
                 A middle point, between the actual position and the target, is required.
                The last step is to make a linear move until the target (goes to the inner sphere)

        """
        # Initialize the return values.
        success = False
        normalize_force_sensor = False

        # Check the distance to target to determine the motion mode.
        distance_to_target = np.linalg.norm(target_pose_in_robot_space_estimated_from_displacement[:3] - robot_pose[:3])
        angular_distance_to_target = np.linalg.norm(target_pose_in_robot_space_estimated_from_displacement[3:] - robot_pose[3:])

        distance_threshold_for_arc_motion = self.robot_config['distance_threshold_for_arc_motion']
        angular_distance_threshold_for_arc_motion = self.robot_config['angular_distance_threshold_for_arc_motion']

        if distance_to_target >= distance_threshold_for_arc_motion or \
           angular_distance_to_target >= angular_distance_threshold_for_arc_motion:

            versor_scale_factor = self.robot_config['versor_scale_factor']
            middle_arc_scale_factor = self.robot_config['middle_arc_scale_factor']
            linear_out_target, waypoint, arc_motion_target = robot_process.compute_arc_motion(
                robot_pose=robot_pose,
                head_center=head_center,
                target_pose=target_pose_in_robot_space_estimated_from_head_pose,  #needs to be target_pose_in_robot_space_estimated_from_head_pose!!
                versor_scale_factor=versor_scale_factor,
                middle_arc_scale_factor=middle_arc_scale_factor,
            )
            if self.motion_type == MotionType.NORMAL:
                self.linear_out_target = linear_out_target
                self.motion_type = MotionType.LINEAR_OUT

            if self.motion_type == MotionType.LINEAR_OUT:
                if np.allclose(np.array(robot_pose), np.array(self.linear_out_target), 0, 10):
                    self.motion_type = MotionType.ARC
                    self.arc_motion_target = arc_motion_target

            elif self.motion_type == MotionType.ARC:
                # Check if the target of arc motion has changed enough; if so, update the target.
                threshold_to_update_arc_motion_target = self.robot_config['threshold_to_update_arc_motion_target']
                if not np.allclose(np.array(arc_motion_target), np.array(self.arc_motion_target), 0, threshold_to_update_arc_motion_target):
                    if np.linalg.norm(target_pose_in_robot_space_estimated_from_head_pose[:3] - robot_pose[:3]) >= distance_threshold_for_arc_motion:
                        self.arc_motion_target = arc_motion_target

                    # Avoid small arc motion; in that case, it is better to use linear movement.
                    elif np.linalg.norm(arc_motion_target[:3] - robot_pose[:3]) < distance_threshold_for_arc_motion / 2:
                        self.motion_type = MotionType.NORMAL

                if np.allclose(np.array(robot_pose)[:3], np.array(self.arc_motion_target)[:3], 0, 20):
                    self.motion_type = MotionType.NORMAL
            else:
                self.motion_type = MotionType.NORMAL
        else:
            self.motion_type = MotionType.NORMAL

        # Check the conditions for tuning.
        angular_distance_threshold_for_tuning = self.robot_config['angular_distance_threshold_for_tuning']
        distance_threshold_for_tuning = self.robot_config['distance_threshold_for_tuning']

        close_to_target = np.linalg.norm(displacement_to_target[:3]) < distance_threshold_for_tuning or \
                          np.linalg.norm(displacement_to_target[3:]) < angular_distance_threshold_for_tuning

        if close_to_target and self.motion_type != MotionType.ARC:
            self.motion_type = MotionType.TUNING

            if not self.tuning_ongoing:
                normalize_force_sensor = True

            self.tuning_ongoing = True
        else:
            self.tuning_ongoing = False

        # Branch to different movement functions depending on the motion type.
        if self.motion_type == MotionType.NORMAL:
            success = self.robot.move_linear(target_pose_in_robot_space_estimated_from_displacement)

        elif self.motion_type == MotionType.LINEAR_OUT:
            success = self.robot.move_linear(self.linear_out_target)

        elif self.motion_type == MotionType.ARC:
            success = self.robot.move_circular(
                start_position=robot_pose,
                waypoint=waypoint,
                target=self.arc_motion_target
            )

        elif self.motion_type == MotionType.TUNING:
            success = self.robot.tune_robot(displacement_to_target)

        elif self.motion_type == MotionType.FORCE_LINEAR_OUT:
            # TODO: Should this be implemented?
            pass

        return success, normalize_force_sensor