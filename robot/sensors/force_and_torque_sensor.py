from collections import deque

import numpy as np


class BufferedForceTorqueSensor:
    def __init__(self, config, robot, buffer_size=50):
        """
        Monitors and analyzes Z-axis force data.

        :param config: Configuration dictionary, must contain 'use_force_sensor'
        :param robot: Robot instance with a `read_force_sensor()` method
        :param buffer_size: Maximum number of force readings to keep
        """
        self.config = config
        self.robot = robot
        self.force_buffer = deque(maxlen=buffer_size)
        self._last_z_offset_sent = None

    def read_force_sensor(self):
        if not self.config.get("use_force_sensor", False):
            print("Error: 'use_force_sensor' is not enabled in configuration.")
            return None

        success, values = self.robot.read_force_sensor()
        if not success:
            print("Error: Could not read the force sensor.")
            return None

        return values

    def update_force_buffer(self):
        force_values = self.read_force_sensor()
        if force_values is not None:
            self.force_buffer.append(force_values)

    def get_latest_value(self, axis=None):
        """
        Get the latest force reading or one axis value.

        :param axis: None (returns full 6D), or int index (0–5)
        :return: list[6] or float or None
        """
        if not self.force_buffer:
            return None

        latest = self.force_buffer[-1]
        if axis is None:
            return latest

        if isinstance(axis, int) and 0 <= axis < 6:
            return latest[axis]

        return None

    def get_buffer(self):
        """
        Returns a copy of the current Z-axis force buffer.

        :return: list of floats – All force readings in the buffer.
        """
        return list(self.force_buffer)

    def get_force_z_buffer(self):
        """
        Extracts the Z-axis (index 2) force values from the buffer.
        """
        return [reading[2] for reading in self.force_buffer if len(reading) == 6]

    def z_offset_changed(self, z_offset, tolerance=1.0):
        """
        Checks whether the new z_offset differs significantly from the last one.
        """
        return not np.isclose(self._last_z_offset_sent or 0.0, z_offset, atol=tolerance)

    def is_force_z_stable(
        self,
        force_setpoint,
        z_offset,
        setpoint_tolerance=1.5,
        threshold_std=0.1,
        min_samples=15,
        window_size=25,
        smoothing=True,
        z_offset_tolerance=1.0,
    ):
        """
        Determines if Z force is stable and z_offset has changed enough.
        """
        buffer = self.get_force_z_buffer()
        if len(buffer) < min_samples:
            return False

        recent = buffer[-window_size:] if len(buffer) > window_size else buffer

        if smoothing and len(recent) > 1:
            alpha = 0.5
            smoothed = [recent[0]]
            for val in recent[1:]:
                smoothed.append(alpha * val + (1 - alpha) * smoothed[-1])
            recent = smoothed

        std_dev = np.std(recent)
        mean_val = np.mean(recent)

        force_near_setpoint = abs(mean_val - force_setpoint) <= setpoint_tolerance
        force_stable = std_dev < threshold_std
        z_offset_changed = self.z_offset_changed(z_offset, z_offset_tolerance)

        is_stable = force_stable and force_near_setpoint and z_offset_changed
        if is_stable:
            self._last_z_offset_sent = z_offset

        return is_stable
