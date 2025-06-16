import threading
import time
import serial
import serial.tools.list_ports
from collections import deque


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
                        value = float(line)
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

    def stop(self):
        self._stop_event.set()
        self.thread.join()
        self._disconnect()
