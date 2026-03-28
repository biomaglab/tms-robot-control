from time import sleep

from robot.robots.elfin.elfin_connection import (
    ElfinConnection,
    MotionState,
)
from robot.robots.robot import Robot


class Elfin(Robot):
    """
    The class for communicating with Elfin robot.
    """

    def __init__(self, ip, use_new_api=False):
        self.use_new_api = use_new_api
        self.connection = ElfinConnection(
            ip=ip,
            use_new_api=use_new_api,
        )
        self._last_dynamic_target = None

        # Smoothing and per-update step limits for new API servo updates.
        self._dynamic_target_alpha = 0.35
        self._max_translation_step_mm = 4.0
        self._max_rotation_step_deg = 1.5

    def _smooth_dynamic_target(self, target):
        if self._last_dynamic_target is None:
            self._last_dynamic_target = list(target)
            return list(target)

        previous = self._last_dynamic_target
        smoothed = []
        for idx, target_value in enumerate(target):
            previous_value = previous[idx]
            blended = previous_value + self._dynamic_target_alpha * (
                target_value - previous_value
            )

            max_step = (
                self._max_translation_step_mm if idx < 3 else self._max_rotation_step_deg
            )
            delta = blended - previous_value
            if delta > max_step:
                blended = previous_value + max_step
            elif delta < -max_step:
                blended = previous_value - max_step

            smoothed.append(blended)

        self._last_dynamic_target = smoothed
        return smoothed

    # Connection
    def connect(self):
        return self.connection.connect()

    def disconnect(self):
        return self.connection.disconnect()

    def is_connected(self):
        return self.connection.connected

    # Initialization
    def initialize(self):
        pass

    # Robot state
    def get_pose(self):
        return self.connection.get_pose()

    def is_moving(self):
        return self.connection.get_motion_state() == MotionState.IN_MOTION

    def is_error_state(self):
        return self.connection.get_motion_state() == MotionState.ERROR

    def read_force_sensor(self):
        return self.connection.read_force_sensor()

    # Movement
    def move_linear(self, target, speed_ratio):
        self._last_dynamic_target = None
        success = self.connection.set_speed_ratio(speed_ratio)
        if not success:
            return False

        # After sending the speed command, wait for a while, as it takes a moment for the robot to actually change speed.
        #
        # XXX: This seems to not be enough time for at least Elfin's new (Linux-based) version to fully change speed,
        # resulting in a speed gradient. However, it's better than not waiting at all, and the current design, where
        # there is a main loop that is periodically called, does not work that well with longer waiting times, as it
        # delays the main loop.
        sleep(0.1)

        return self.connection.move_linear(target)

    def dynamic_motion(self, target, speed_ratio):
        success = self.connection.set_speed_ratio(speed_ratio)
        if not success:
            return False

        if self.use_new_api:
            smoothed_target = self._smooth_dynamic_target(target)
            return self.connection.move_linear(smoothed_target)

        # Using MoveB in old API
        return self.connection.move_linear(target)

    def move_circular(self, start_position, waypoint, target, speed_ratio):
        success = self.connection.set_speed_ratio(speed_ratio)
        if not success:
            return False

        # After sending the speed command, wait for a while, as it takes a moment for the robot to actually change speed.
        #
        # XXX: This seems to not be enough time for at least Elfin's new (Linux-based) version to fully change speed,
        # resulting in a speed gradient. However, it's better than not waiting at all, and the current design, where
        # there is a main loop that is periodically called, does not work that well with longer waiting times, as it
        # delays the main loop.
        sleep(0.1)

        return self.connection.move_circular(start_position, waypoint, target)

    def stop_robot(self):
        success = self.connection.stop_robot()
        self._last_dynamic_target = None

        # After the stop command, it takes some milliseconds for the robot to stop. Wait for that time.
        sleep(0.05)

        return success

    def enable_free_drive(self):
        success = self.connection.enable_assistive_robot()
        return success

    def disable_free_drive(self):
        success = self.connection.disable_assistive_robot()
        return success

    # Destruction and cleanup
    def close(self):
        self.stop_robot()
        self.disconnect()
