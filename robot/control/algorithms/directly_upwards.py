from enum import Enum

import numpy as np


class MotionType(Enum):
    UPWARDS = 0
    ORIENT = 1
    PLANE_MOVEMENT = 2
    DOWNWARDS = 3


class DirectlyUpwardsAlgorithm:
    def __init__(self, robot, robot_config):
        self.robot = robot
        self.robot_config = robot_config

        self.motion_type = MotionType.UPWARDS

    def reset(self):
        self.motion_type = MotionType.UPWARDS

    def move_decision(self,
                      displacement_to_target,
                      target_pose_in_robot_space_estimated_from_head_pose,
                      target_pose_in_robot_space_estimated_from_displacement,
                      robot_pose,
                      head_center):
        # TODO: Implement this.

        # Initialize the return values.
        success = False
        normalize_force_sensor = False

        return success, normalize_force_sensor
