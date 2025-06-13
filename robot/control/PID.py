import time

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
        self.force_setpoint = -3.0      # Used in Impedance mode (desired force)

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
        self.last_time = time.monotonic()

    def update(self, feedback_value, force_feedback=None):
        if not self.enabled:
            return self.output

        self.current_time = time.monotonic()
        delta_time_actual = self.current_time - self.last_time

        if self.sample_time > 0 and delta_time_actual < self.sample_time:
            return self.output

        delta_time = self.sample_time if self.sample_time > 0 else delta_time_actual

        displacement_error = self.displacement_setpoint - feedback_value

        if self.mode == 'pid':
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

        elif self.mode == 'impedance':
            if force_feedback is None:
                raise ValueError("force_feedback is required in impedance mode")

            # Compute force error (desired - actual)
            force_error = self.force_setpoint - force_feedback
            print("force_error", force_error)
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
