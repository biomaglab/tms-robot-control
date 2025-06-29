import time


class PIDControllerGroup:
    def __init__(self, use_force=False, use_pressure=False):
        self.translation_pids = [
            ImpedancePIDController(P=0.5),  # x
            ImpedancePIDController(P=0.5),  # y
        ]

        if use_pressure:
            self.stiffness = 0.05
            self.damping = 0.02
            pid_z = ImpedancePIDController(P=0.5, I=0.01, D=0.0, mode='impedance')
        elif use_force:
            self.stiffness = 0.1
            self.damping = 0
            pid_z = ImpedancePIDController(P=0.1, I=0.0001, D=0.0, stiffness=self.stiffness, damping=self.damping, mode='impedance')
        else:
            pid_z = ImpedancePIDController(P=0.1)

        self.translation_pids.append(pid_z)

        self.rotation_pids = [
            ImpedancePIDController(),  # rx
            ImpedancePIDController(),  # ry
            ImpedancePIDController(),  # rz
        ]

    def update_translation(self, translations, force_feedback=None):
        # Update x, y
        self.translation_pids[0].update(translations[0])
        self.translation_pids[1].update(translations[1])

        # Update z with optional force feedback
        if force_feedback is not None:
            self.dynamically_update_stiffness_and_damping(translations[2], force_feedback=force_feedback, min_stiffness=(self.stiffness*2)/10, max_stiffness=self.stiffness*2, damping_ratio=self.damping/self.stiffness)
            self.translation_pids[2].update(translations[2], force_feedback=force_feedback)
        else:
            self.translation_pids[2].update(translations[2])

    def update_rotation(self, angles):
        for pid, angle in zip(self.rotation_pids, angles):
            pid.update(angle)

    def dynamically_update_stiffness_and_damping(self, z_displacement, force_feedback,
                                    max_displacement=1.0,
                                    min_stiffness=0.01, max_stiffness=0.1,
                                    smoothing=0.9, release_force_threshold=0.1, damping_ratio=0.4):
        """
        Adjust stiffness based on displacement and force_feedback, with lockout if force is too high.
        Parameters:
            z_displacement (float): Z-axis displacement.
            force_feedback (float): Current measured force (typically negative in compression).
            max_displacement (float): Max considered displacement.
            min_stiffness (float): Minimum stiffness when force is high.
            max_stiffness (float): Maximum stiffness for large displacements.
            smoothing (float): EMA smoothing factor (0 = no smoothing).
            release_force_threshold (float): Force value below which stiffness lock is released.
            damping_ratio (float): Ratio used to compute damping relative to current stiffness.
        """

        # Compute absolute force limit
        max_force = abs(self.translation_pids[2].force_setpoint * 2)

        # Initialize lock flag if not present
        if not hasattr(self, "stiffness_locked_due_to_force"):
            self.stiffness_locked_due_to_force = False

        # Lock stiffness if force_feedback exceeds threshold
        if abs(force_feedback) > max_force:
            self.stiffness_locked_due_to_force = True

        # Unlock stiffness if force is close enough to zero
        if abs(force_feedback) < release_force_threshold:
            self.stiffness_locked_due_to_force = False

        if self.stiffness_locked_due_to_force:
            # Force too high → lock to minimum stiffness
            stiffness = min_stiffness
        else:
            # Scale stiffness with displacement
            normalized_disp = max(0.0, min(z_displacement / max_displacement, 1.0))
            target_stiffness = min_stiffness + normalized_disp * (max_stiffness - min_stiffness)

            # Apply smoothing
            current_stiffness = self.translation_pids[2].stiffness
            stiffness = smoothing * current_stiffness + (1 - smoothing) * target_stiffness

        # Set the stiffness and damping
        self.translation_pids[2].stiffness = stiffness
        self.translation_pids[2].damping = damping_ratio * stiffness

    def get_outputs(self):
        # Return two lists, negated outputs for translation and rotation respectively
        trans_out = [-pid.output for pid in self.translation_pids]
        rot_out = [-pid.output for pid in self.rotation_pids]
        return trans_out, rot_out

    def get_force_setpoint(self):
        return self.translation_pids[2].force_setpoint

    def clear(self):
        for pid in self.translation_pids + self.rotation_pids:
            pid.clear()


class ImpedancePIDController:
    def __init__(self, P=0.3, I=0.01, D=0.0, stiffness=0.05, damping=0.02, mode='pid'):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.stiffness = stiffness
        self.damping = damping

        self.mode = mode.lower()
        if self.mode not in ('pid', 'impedance'):
            raise ValueError("mode must be 'pid' or 'impedance'")

        self.sample_time = 0.0
        self.current_time = time.monotonic()
        self.last_time = self.current_time

        # Independent setpoints
        self.displacement_setpoint = 0.0        # Used in PID mode (desired displacement)
        self.force_setpoint = -5.0      # Used in Impedance mode (desired force)

        self.last_displacement_error = 0.0
        self.last_force_error = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.windup_guard = 20.0

        self.output = 0.0
        self.velocity = 0.0

        self.output_min = -float('inf')
        self.output_max = float('inf')
        self.enabled = True

    def clear(self):
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_displacement_error = 0.0
        self.last_force_error = 0.0
        self.velocity = 0.0
        self.output = 0.0
        self.current_time = time.monotonic()
        self.last_time = self.current_time

    def update(self, feedback_value, force_feedback=None):
        if not self.enabled:
            return self.output

        self.current_time = time.monotonic()
        delta_time_actual = self.current_time - self.last_time

        if self.sample_time > 0 and delta_time_actual < self.sample_time:
            return self.output

        delta_time = self.sample_time if self.sample_time > 0 else delta_time_actual
        # Compute displacement error (desired - actual)
        displacement_error = self.displacement_setpoint - feedback_value

        if self.mode == 'impedance' and force_feedback is not None:
            # Compute force error (desired - actual)
            force_error = self.force_setpoint - force_feedback

            # PID for force control (force feedback loop)
            self.PTerm = self.Kp * force_error
            self.ITerm += force_error * delta_time
            self.ITerm = max(-self.windup_guard, min(self.windup_guard, self.ITerm))  # Anti-windup
            # Derivative of force error
            delta_force_error = force_error - self.last_force_error
            self.DTerm = delta_force_error / delta_time if delta_time > 0 else 0.0

            # Impedance control: Calculate position correction based on displacement error
            impedance_position_correction = self.stiffness * displacement_error - self.damping * self.velocity

            # Update position correction using force control (PID) and impedance control
            unclamped_output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm) + impedance_position_correction

            # Update position based on velocity and acceleration
            acceleration = (unclamped_output - self.damping * self.velocity)
            self.velocity += acceleration * delta_time

            # Apply position output limits
            self.output = max(self.output_min, min(unclamped_output, self.output_max))
            self.last_force_error = force_error

        else:
            self.PTerm = self.Kp * displacement_error
            self.ITerm += displacement_error * delta_time
            self.ITerm = max(-self.windup_guard, min(self.windup_guard, self.ITerm))
            delta_error = displacement_error - self.last_displacement_error
            self.DTerm = delta_error / delta_time if delta_time > 0 else 0.0
            unclamped = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            self.output = max(self.output_min, min(unclamped, self.output_max))

            if unclamped != self.output and self.Ki != 0:
                self.ITerm = (self.output - self.PTerm - self.Kd * self.DTerm) / self.Ki
            self.last_displacement_error = displacement_error

        # Store last update time and error for future calculations
        self.last_time = self.current_time

        return self.output

    # --- Configuration Methods ---

    def set_mode(self, mode):
        if mode not in ('pid', 'impedance'):
            raise ValueError("mode must be 'pid' or 'impedance'")
        self.mode = mode

    def set_gains(self, P=None, I=None, D=None):
        if P is not None: self.Kp = P
        if I is not None: self.Ki = I
        if D is not None: self.Kd = D

    def set_impedance(self, stiffness=None, damping=None):
        if stiffness is not None: self.stiffness = stiffness
        if damping is not None: self.damping = damping

    def set_pid_setpoint(self, setpoint):
        """Used in PID mode."""
        self.displacement_setpoint = setpoint

    def set_force_setpoint(self, force_setpoint):
        """Used in impedance mode."""
        self.force_setpoint = force_setpoint

    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def set_output_limits(self, min_val, max_val):
        if min_val > max_val:
            raise ValueError("min_val must be <= max_val")
        self.output_min = min_val
        self.output_max = max_val

    def set_enabled(self, enabled):
        self.enabled = enabled
