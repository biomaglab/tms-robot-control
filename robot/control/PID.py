import time

class PID:
    """
    A PID controller with anti-windup, output clamping, and sample time support.
    """

    def __init__(self, P=0.3, I=0.01, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00  # Default to update every call
        self.current_time = time.monotonic()
        self.last_time = self.current_time

        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        self.windup_guard = 20.0  # Windup guard for integral term sum

        self.output = 0.0
        self.output_min = -float('inf')
        self.output_max = float('inf')
        self.enabled = True  # Controller enable flag

    def clear(self):
        """
        Clears PID computations, resets controller state. Coefficients (Kp, Ki, Kd) remain unchanged.
        """
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.last_time = time.monotonic()
        self.current_time = self.last_time

    def update(self, feedback_value):
        """
        Computes PID output. Returns last output if disabled or sample time not met.
        """
        if not self.enabled:
            return self.output

        error = self.SetPoint - feedback_value
        self.current_time = time.monotonic()
        delta_time_actual = self.current_time - self.last_time

        if self.sample_time > 0 and delta_time_actual < self.sample_time:
            return self.output  # Exit if sample time not met

        # Calculate time delta for this update
        delta_time = self.sample_time if self.sample_time > 0 else delta_time_actual

        # Calculate PID terms
        self.PTerm = self.Kp * error
        self.ITerm += error * delta_time

        # Apply integral windup guard
        self.ITerm = max(-self.windup_guard, min(self.windup_guard, self.ITerm))

        delta_error = error - self.last_error
        self.DTerm = delta_error / delta_time if delta_time > 0 else 0.0

        # Store last update time and error
        self.last_time = self.current_time
        self.last_error = error

        # Compute unclamped output
        unclamped_output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

        # Apply output limits
        self.output = max(self.output_min, min(unclamped_output, self.output_max))

        # Back-calculate ITerm to prevent windup if clamped
        if unclamped_output != self.output and self.Ki != 0:
            self.ITerm = (self.output - self.PTerm - self.Kd * self.DTerm) / self.Ki

        return self.output

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        self.sample_time = sample_time

    def set_output_limits(self, min_val, max_val):
        if min_val > max_val:
            raise ValueError("min_val must be <= max_val")
        self.output_min = min_val
        self.output_max = max_val

    def set_enabled(self, enabled):
        self.enabled = enabled

    def set_setpoint(self, setpoint):
        self.SetPoint = setpoint


class PIDImpedanceDualFeedback:
    """
    A PID-based impedance controller for TMS coil positioning, using both force and displacement feedback.
    """

    def __init__(self, P=0.3, I=0.01, D=0.0, stiffness=0.05, damping=0.02):
        # PID gains
        self.Kp = P
        self.Ki = I
        self.Kd = D

        # Impedance model
        self.stiffness = stiffness  # Virtual stiffness (K)
        self.damping = damping      # Virtual damping (B)

        # Controller state
        self.sample_time = 0.00  # Default to update every call
        self.current_time = time.monotonic()
        self.last_time = self.current_time

        # Force and displacement feedback
        self.ForceSetPoint = 3.0  # Desired force
        self.position = 0.0       # Controlled z-position (displacement)
        self.velocity = 0.0       # Simulated z-velocity

        # PID terms
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup guard for integral term
        self.windup_guard = 20.0

        self.output = 0.0
        self.output_min = -float('inf')
        self.output_max = float('inf')
        self.enabled = True  # Controller enable flag

    def clear(self):
        """ Clears PID computations and resets controller state. """
        self.ForceSetPoint = 3.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.position = 0.0
        self.velocity = 0.0
        self.last_time = time.monotonic()
        self.current_time = self.last_time

    def update(self, displacement_feedback, force_feedback):
        """
        Computes the position correction based on both force and displacement feedback.
        """
        if not self.enabled:
            return self.position

        # Compute force error (desired - actual)
        force_error = self.ForceSetPoint - force_feedback

        # Compute displacement error (desired - actual)
        displacement_error = self.position - displacement_feedback

        # Current time and sample time calculation
        self.current_time = time.monotonic()
        delta_time_actual = self.current_time - self.last_time

        if self.sample_time > 0 and delta_time_actual < self.sample_time:
            return self.position  # Exit if sample time not met

        delta_time = self.sample_time if self.sample_time > 0 else delta_time_actual

        # PID for force control (force feedback loop)
        self.PTerm = self.Kp * force_error
        self.ITerm += force_error * delta_time
        self.ITerm = max(-self.windup_guard, min(self.windup_guard, self.ITerm))  # Anti-windup

        # Derivative of force error
        delta_force_error = force_error - self.last_error
        self.DTerm = delta_force_error / delta_time if delta_time > 0 else 0.0

        # Impedance control: Calculate position correction based on displacement error
        impedance_position_correction = self.stiffness * displacement_error - self.damping * self.velocity

        # Update position correction using force control (PID) and impedance control
        position_correction = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm) + impedance_position_correction

        # Update position based on velocity and acceleration
        acceleration = (position_correction - self.damping * self.velocity)
        self.velocity += acceleration * delta_time
        self.position += self.velocity * delta_time

        # Apply position output limits
        self.position = max(self.output_min, min(self.position, self.output_max))

        # Store last update time and error for future calculations
        self.last_time = self.current_time
        self.last_error = force_error

        return self.position

    def setGains(self, P, I, D):
        """ Set the PID gains for force control. """
        self.Kp = P
        self.Ki = I
        self.Kd = D

    def setImpedance(self, stiffness, damping):
        """ Set virtual stiffness and damping for impedance behavior. """
        self.stiffness = stiffness
        self.damping = damping

    def setSetpoint(self, force_setpoint):
        """ Set the desired force setpoint. """
        self.ForceSetPoint = force_setpoint

    def setSampleTime(self, sample_time):
        """ Set the sample time (update rate) for the controller. """
        self.sample_time = sample_time

    def setOutputLimits(self, min_val, max_val):
        """ Set position limits for safe movement. """
        if min_val > max_val:
            raise ValueError("min_val must be <= max_val")
        self.output_min = min_val
        self.output_max = max_val

    def setEnabled(self, enabled):
        """ Enable or disable the PID controller. """
        self.enabled = enabled
