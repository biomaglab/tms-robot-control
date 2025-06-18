import threading
import time
import serial
import serial.tools.list_ports
from collections import deque
import numpy as np


class BufferedPressureSensorReader:
    def __init__(self, port, baudrate=115200, buffer_size=100, reconnect_interval=5):
        self.port = port
        self.baudrate = baudrate
        self.buffer_size = buffer_size
        self.reconnect_interval = reconnect_interval

        self.buffer = deque(maxlen=buffer_size)
        self.lock = threading.Lock()
        self._stop_event = threading.Event()
        self.ready = False
        self.started = False  # Flag: has valid data been received
        self.serial = None

        self._last_z_offset_sent = 0

        # Start background thread
        self.thread = threading.Thread(target=self._serial_loop, daemon=True)
        self.thread.start()

    def _serial_loop(self):
        while not self._stop_event.is_set():
            if not self.ready:
                if self._try_connect():
                    print(f"[✓] Connected to {self.port}")
                else:
                    print(f"[!] Retrying in {self.reconnect_interval}s...")
                    time.sleep(self.reconnect_interval)
                    continue

            try:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    try:
                        value = -float(line) / 10 # Rescale to match the force and torque sensor.
                        if not self.started and not self._is_valid(value):
                            print("[i] Waiting for valid pressure data...")
                            continue  # Ignore invalid/NaN at startup

                        with self.lock:
                            self.buffer.append(value)
                        if not self.started:
                            self.started = True
                            print("[✓] Pressure data started.")
                    except ValueError:
                        print(f"[!] Invalid float: {line}")
            except serial.SerialException as e:
                print(f"[!] Serial error: {e}. Attempting reconnect.")
                self._disconnect()

    def _try_connect(self):
        if not self._verify_port():
            return False
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            self.ready = True
            return True
        except serial.SerialException as e:
            print(f"[!] Failed to open port '{self.port}': {e}")
            self.ready = False
            return False

    def _is_valid(self, value):
        return not (value is None or isinstance(value, float) and (
                    value != value or value == float('inf') or value == float('-inf')))

    def _disconnect(self):
        self.ready = False
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception as e:
            print(f"[!] Error closing serial: {e}")
        self.serial = None

    def _verify_port(self):
        """Check if the port exists and is accessible."""
        available_ports = [p.device for p in serial.tools.list_ports.comports()]
        if self.port not in available_ports:
            print(f"[!] Port '{self.port}' not found among: {available_ports}")
            return False

        try:
            with serial.Serial(self.port, self.baudrate, timeout=1):
                return True
        except serial.SerialException:
            return False

    def get_latest_value(self):
        with self.lock:
            return self.buffer[-1] if self.buffer else None

    def get_buffer(self):
        with self.lock:
            return list(self.buffer)

    def has_started(self):
        return self.started

    def is_force_stable(
        self,
        force_setpoint,
        z_offset,
        setpoint_tolerance=1.5,
        threshold_std=0.1,
        min_samples=15,
        window_size=25,
        smoothing=True,
        z_offset_tolerance=1
    ):
        """
        Determine if the force is stable and the z_offset is different enough to be sent.

        Args:
            force_setpoint (float): Target force value (before scaling).
            z_offset (float): The current z-offset candidate to check.
            setpoint_tolerance (float): Allowed deviation from the scaled force setpoint.
            threshold_std (float): Max allowed standard deviation to consider force stable.
            min_samples (int): Minimum number of samples required for evaluation.
            window_size (int): Number of recent samples to consider.
            smoothing (bool): If True, apply exponential smoothing to reduce noise.
            z_offset_tolerance (float): Minimum change required from last sent z_offset.

        Returns:
            bool: True if force is stable and z_offset differs enough from the last sent value.
        """
        buffer = self.get_buffer()
        if not buffer or len(buffer) < min_samples:
            return False

        recent = buffer[-window_size:] if len(buffer) > window_size else buffer

        if smoothing and len(recent) > 1:
            alpha = 0.2
            smoothed = [recent[0]]
            for val in recent[1:]:
                smoothed.append(alpha * val + (1 - alpha) * smoothed[-1])
            recent = smoothed

        std_dev = np.std(recent)
        mean_val = np.mean(recent)

        force_near_setpoint = abs(mean_val - force_setpoint) <= setpoint_tolerance
        force_stable = std_dev < threshold_std
        z_offset_changed = not np.isclose(self._last_z_offset_sent, z_offset, atol=z_offset_tolerance) # Avoid sending the same value repeatedly

        is_stable = force_stable and force_near_setpoint and z_offset_changed
        if is_stable:
            self._last_z_offset_sent = z_offset

        return is_stable

    def stop(self):
        self._stop_event.set()
        self.thread.join()
        self._disconnect()
