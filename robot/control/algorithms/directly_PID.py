from enum import Enum
import numpy as np
import time
from collections import deque

from robot.control.color import Color
from robot.control.PID import ImpedancePIDController


class MotionSequenceState(Enum):
    NOT_INITIATED = 0
    MOVE_UPWARD = 1
    CHARACTERIZE_CABLE = 2  # New state for cable characterization
    MOVE_TO_TARGET = 3

    def next(self):
        """Returns the next state in the sequence, and remains in the last state."""
        members = list(MotionSequenceState)
        index = members.index(self)

        # Check if current state is the last state
        if index == len(members) - 1:
            return self

        # Otherwise, proceed to the next state
        return members[index + 1]


class CableCharacterizer:
    def __init__(self, safe_height, min_samples=100):
        self.safe_height = safe_height
        self.min_samples = min_samples
        self.force_samples = []
        self.characterization_complete = False
        self.static_offset = 0.0
        self.last_update_time = time.time()

    def apply_dynamic_compensation(self, force_z, velocity, acceleration):
        """Apply additional dynamic compensation to Z-axis force"""
        if not self.characterization_complete:
            return force_z

        # Velocity-dependent compensation
        vel_z = velocity[2]  # Z-axis velocity
        vel_compensation = vel_z * 0.2  # 0.2 N per m/s

        # Acceleration-dependent compensation
        accel_z = acceleration[2]
        accel_compensation = accel_z * 0.05  # 0.05 N per m/s²

        # Direction-based compensation factors
        if vel_z > 0:  # Moving upward
            compensation_factor = 1.3
        elif vel_z < 0:  # Moving downward
            compensation_factor = 0.8
        else:
            compensation_factor = 1.0

        # Apply compensation
        compensated_force = force_z - (vel_compensation + accel_compensation) * compensation_factor

        # Clamp to prevent over-compensation
        max_compensation = 5.0  # Max 5N compensation
        return np.clip(compensated_force, force_z - max_compensation, force_z + max_compensation)

    def update(self, force, pose):
        """Collect data for cable characterization"""
        # Only characterize at safe height
        if abs(pose[2] - self.safe_height) < 1.0:  # Within 1mm of safe height
            self.force_samples.append(force[2])  # Only Z-axis force

            if len(self.force_samples) >= self.min_samples:
                self._complete_characterization()

    def _complete_characterization(self):
        """Finalize cable characterization"""
        if not self.force_samples:
            return

        # Use median to avoid outliers
        self.static_offset = np.median(self.force_samples)
        self.characterization_complete = True
        print(f"Cable characterization complete. Static offset: {self.static_offset:.2f}N")

    def get_compensated_force(self, raw_force, velocity, acceleration):
        """Apply cable compensation to force reading"""
        if not self.characterization_complete:
            return raw_force

        # Static offset compensation
        compensated = raw_force - self.static_offset

        # Dynamic compensation based on movement
        velocity_factor = np.linalg.norm(velocity[:3]) * 0.15  # Linear velocity impact
        acceleration_factor = np.linalg.norm(acceleration[:3]) * 0.3  # Acceleration impact
        dynamic_compensation = velocity_factor + acceleration_factor

        # Direction-sensitive compensation
        if velocity[2] > 0:  # Moving upward
            dynamic_compensation *= 1.2  # Extra compensation for upward movement
        elif velocity[2] < 0:  # Moving downward
            dynamic_compensation *= 0.8  # Less compensation for downward movement

        return compensated - dynamic_compensation

    def reset(self):
        """Reset characterization data"""
        self.force_samples = []
        self.characterization_complete = False


class DirectlyPIDAlgorithm:
    def __init__(self, robot, config, robot_config):
        self.robot = robot
        self.config = config
        self.default_speed_ratio = config['default_speed_ratio']
        self.tuning_speed_ratio = config['tuning_speed_ratio']
        self.translation_threshold = config['translation_threshold']
        self.rotation_threshold = config['rotation_threshold']
        self.robot_config = robot_config

        self.reset_state()
        self.reset_force_normalized()

        # Motion control variables
        self.hold_start_time = 0
        self.hold_duration = 1.5  # Seconds to hold for characterization
        self.last_pose = None
        self.last_velocity = np.zeros(6)
        self.last_time = time.time()

    def reset_state(self):
        self.motion_sequence_state = MotionSequenceState.NOT_INITIATED

    def reset_force_normalized(self):
        self.force_normalized = False

    def move_decision(self,
                      displacement_to_target,
                      target_pose_in_robot_space_estimated_from_head_pose,
                      target_pose_in_robot_space_estimated_from_displacement,
                      robot_pose,
                      head_center):

        # Compute the maximum translation and rotation to the target.
        max_translation = np.max(np.abs(displacement_to_target[:3]))
        max_rotation = np.max(np.abs(displacement_to_target[3:]))

        # State transition logic
        if round(robot_pose[2]) < round(self.config['safe_height']):
            # If the maximum translation or rotation to the target is larger than the threshold, initiate the motion sequence. If motion sequence is not initiated, check if it should be.
            if max_translation > self.translation_threshold or max_rotation > self.rotation_threshold:
                if self.motion_sequence_state == MotionSequenceState.NOT_INITIATED:
                    self.motion_sequence_state = MotionSequenceState.MOVE_UPWARD
        else:
            # Only transition to CHARACTERIZE if we just arrived at safe height
            if self.motion_sequence_state == MotionSequenceState.MOVE_UPWARD:
                self.motion_sequence_state = MotionSequenceState.CHARACTERIZE_CABLE
                self.hold_start_time = time.time()
            elif self.motion_sequence_state == MotionSequenceState.CHARACTERIZE_CABLE:
                force_z = self.robot.get_force_sensor_only_Z()
                if force_z is not None:
                    self.robot.cable_char.update(force_z, robot_pose)
                # Check if hold duration has passed
                if time.time() - self.hold_start_time > self.hold_duration:
                    self.motion_sequence_state = MotionSequenceState.MOVE_TO_TARGET

        # State execution
        if self.motion_sequence_state == MotionSequenceState.MOVE_UPWARD:
            success = self._move_to_safe_height(target_pose_in_robot_space_estimated_from_displacement)
        elif self.motion_sequence_state == MotionSequenceState.CHARACTERIZE_CABLE:
            success = self._hold_for_characterization()
        else:
            success = self._tune(target_pose_in_robot_space_estimated_from_displacement)

        # Normalization logic remains the same
        close_translation = max_translation < self.translation_threshold / 10
        close_rotation = max_rotation < self.rotation_threshold / 10
        normalize_force_sensor = close_translation and close_rotation and not self.force_normalized
        if normalize_force_sensor:
            self.force_normalized = True
            print("Force normalization triggered.")

        return success, normalize_force_sensor

    def _tune(self, target_pose_in_robot_space):
        print("Initiating tuning motion")

        success = self.robot.dynamic_motion(target_pose_in_robot_space, self.tuning_speed_ratio)
        return success

    def _move_to_safe_height(self, target_pose_in_robot_space=None):
        print("\n{}Moving upward to safe height{}".format(Color.BOLD, Color.END))
        success, pose = self.robot.get_pose()
        if not success:
            return False

        # Move to safe height
        pose[2] = self.config['safe_height']
        if target_pose_in_robot_space:
            pose[3:] = target_pose_in_robot_space[3:]

        success = self.robot.dynamic_motion(pose, self.default_speed_ratio)
        return success

    def _hold_for_characterization(self):
        """Hold position to characterize cable forces"""
        # Get current pose and maintain position
        success, current_pose = self.robot.get_pose()
        if not success:
            return False

        # Hold position - send current pose as target
        hold_success = self.robot.dynamic_motion(current_pose, 0.1)  # Low speed ratio for holding

        # Check if hold duration complete
        if time.time() - self.hold_start_time > self.hold_duration:
            print("Cable characterization complete. Proceeding to target.")
            self.motion_sequence_state = self.motion_sequence_state.next()

        return hold_success

    def move_away_from_head(self):
        self.robot.stop_robot()
        success = self._move_to_safe_height()
        return success
