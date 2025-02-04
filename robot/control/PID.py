import time

class PID:
    """
    A PID controller with anti-windup, output clamping, and sample time support.
    """

    def __init__(self, P=0.2, I=0.0, D=0.0):
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
