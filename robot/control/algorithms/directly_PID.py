from enum import Enum

import numpy as np

from robot.control.color import Color
from robot.control.PID import PID


class MotionSequenceState(Enum):
    NOT_INITIATED = 0
    MOVE_UPWARD = 1
    MOVE_TO_TARGET = 2

    def next(self):
        """Returns the next state in the sequence, and remains in the last state."""
        members = list(MotionSequenceState)
        index = members.index(self)

        # Check if current state is the last state
        if index == len(members) - 1:
            return self

        # Otherwise, proceed to the next state
        return members[index + 1]


class DirectlyPIDAlgorithm:
    def __init__(self, robot, config, robot_config):
        self.robot = robot
        self.config = config

        self.default_speed_ratio = config['default_speed_ratio']
        self.tuning_speed_ratio = config['tuning_speed_ratio']
        self.translation_threshold = config['translation_threshold']
        self.rotation_threshold = config['rotation_threshold']

        # Unused for now.
        self.robot_config = robot_config

        self.reset_state()

    def reset_state(self):
        self.motion_sequence_state = MotionSequenceState.NOT_INITIATED

    def move_decision(self,
                      displacement_to_target,
                      target_pose_in_robot_space_estimated_from_head_pose,
                      target_pose_in_robot_space_estimated_from_displacement,
                      robot_pose,
                      head_center):

        # Compute the maximum translation and rotation to the target.
        max_translation = np.max(np.abs(displacement_to_target[:3]))
        max_rotation = np.max(np.abs(displacement_to_target[3:]))

        if round(robot_pose[2]) < round(self.config['safe_height']):
            # If the maximum translation or rotation to the target is larger than the threshold, initiate the motion sequence. If motion sequence is not initiated, check if it should be.
            if max_translation > self.translation_threshold or max_rotation > self.rotation_threshold:
                if max_translation > self.translation_threshold:
                    print("Translation ({:.2f} mm) exceeds the threshold ({:.2f} mm)".format(max_translation, self.translation_threshold))
                if max_rotation > self.rotation_threshold:
                    print("Rotation ({:.2f} deg) exceeds the threshold ({:.2f} deg)".format(max_rotation, self.rotation_threshold))
                if self.motion_sequence_state == MotionSequenceState.NOT_INITIATED:
                    print("Initiating motion sequence")
                    # Start the motion sequence by moving upward.
                    self.motion_sequence_state = MotionSequenceState.MOVE_UPWARD
        else:
            self.motion_sequence_state = MotionSequenceState.MOVE_TO_TARGET

        # If motion sequence is initiated, continue the sequence, otherwise perform tuning motion.
        if self.motion_sequence_state == MotionSequenceState.MOVE_UPWARD:
            success = self._move_to_safe_height(target_pose_in_robot_space_estimated_from_displacement)
        else:
            success = self._tune(target_pose_in_robot_space_estimated_from_displacement)

        # TODO: The force sensor is not normalized for now - add some logic here.
        normalize_force_sensor = False

        return success, normalize_force_sensor

    def _tune(self, target_pose_in_robot_space):
        print("Initiating tuning motion")

        success = self.robot.move_linear(target_pose_in_robot_space, self.tuning_speed_ratio)
        return success

    def _move_to_safe_height(self, target_pose_in_robot_space=None):
        print("")
        print("{}Moving upward to a safe height{}".format(Color.BOLD, Color.END))

        success, pose = self.robot.get_pose()
        if not success:
            return False

        # Set the safe height
        actual_pose_z = pose[2] 
        pose[2] = self.config['safe_height']
        # If a target pose is provided, modify Z pose for orientation
        if target_pose_in_robot_space:
            pose[3] = target_pose_in_robot_space[3]
            pose[4] = target_pose_in_robot_space[4]
            pose[5] = target_pose_in_robot_space[5]
        success = self.robot.move_linear(pose, self.default_speed_ratio)

        # Transition to the next state if needed
        if target_pose_in_robot_space and success and (round(actual_pose_z) >= round(self.config['safe_height'])):
            self.motion_sequence_state = self.motion_sequence_state.next()

        return success

    def move_away_from_head(self):
        success = self._move_to_safe_height()
        return success
