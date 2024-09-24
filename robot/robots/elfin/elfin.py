from time import sleep

from robot.robots.robot import Robot

from robot.robots.elfin.elfin_connection import (
    ElfinConnection,
    MotionState,
)


class Elfin(Robot):
    """
    The class for communicating with Elfin robot.
    """
    def __init__(self, ip, use_new_api=False):
        self.connection = ElfinConnection(
            ip=ip,
            use_new_api=use_new_api,
        )

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
