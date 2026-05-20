import time
from enum import Enum
from socket import AF_INET, SOCK_STREAM, socket


class MotionState(Enum):
    FREE_TO_MOVE = 0
    IN_MOTION = 1
    WAITING_FOR_EXECUTION = 2
    ERROR = 3
    UNKNOWN = 4


class ReferenceFrame(Enum):
    ROBOT = 0
    TOOL = 1


class ElfinConnection:
    PORT = 10003
    REQUEST_ENDING_CHARS = ",;"
    RESPONSE_LENGTH = 1024
    ROBOT_ID = 0

    def __init__(self, ip, use_new_api):
        """
        Class for low-level communication with Elfin robot.

        This class follows "HansRobot Communication Protocol Interface".
        """
        self.ip = ip
        self.use_new_api = use_new_api
        self.servo_started = False
        self.tcp_coordinate_system = False

        self.connected = False
        self.socket = None

    def connect(self):
        """
        Connects to the robot.

        :return: True if successful, otherwise False.
        """
        if self.connected:
            print("Already connected to the robot")
            return True

        try:
            new_socket = socket(AF_INET, SOCK_STREAM)
            new_socket.connect((self.ip, self.PORT))

            self.socket = new_socket

            self.connected = True
        except:
            print("Failed to connect to the robot")

        return self.connected

    def disconnect(self):
        """
        Disconnects from the robot.

        :return: True if successful, otherwise False.
        """
        success = False

        if not self.connected:
            print("Not connected to the robot, therefore cannot disconnect")
            return success

        try:
            self.socket.close()
            self.connected = False
            success = True
        except:
            print("Failed to disconnect from the robot")

        return success

    def _send_and_receive(self, request, verbose=False):
        if verbose:
            print("Sending request: {}".format(request))

        full_request = request + self.REQUEST_ENDING_CHARS
        try:
            # Send the request to the robot.
            self.socket.sendall(full_request.encode("utf-8"))

            # Receive the response from the robot.
            response = self.socket.recv(self.RESPONSE_LENGTH).decode("utf-8").split(",")

        except (BrokenPipeError, ConnectionResetError, TimeoutError) as e:
            print("Robot connection error: {}".format(e))
            self.connected = False
            return False, None

        if verbose:
            print("Done.")

        # Process the response.
        command = response[0]
        status = response[1]
        error_code = response[2]

        if status == "OK":
            success = True

        elif status == "Fail":
            print(
                "The command {} returned the error code: {}".format(command, error_code)
            )
            success = False

        else:
            print("Unknown status")
            success = False

        # XXX: Params returned by Elfin start from element 2 if the command was
        #   successful, otherwise element 2 is reserved for the error code. It would
        #   be cleaner if the params started always from element 3 and error code was 0
        #   if there is no error.
        fetch_params = type(response) is not bool and len(response) > 3 and success

        # The elements 2...n-1 are the parameters, where n is the last element.
        params = response[2:-1] if fetch_params else None

        return success, params

    def list_to_str(self, list):
        """
        Converts a list of numbers to a string, with each number formatted to two decimal places.

        :param list: A list of numbers, e.g. [1, 2.333, 3].
        :return: A string representation of the list with two decimal places, e.g. "1.00,2.33,3.00".
        """
        return ",".join([f"{s:.2f}" for s in list])

    # Robot commands

    def stop_robot(self):
        """
        Stops the robot's movement.

        :return: True if successful, otherwise False.
        """
        request = "GrpStop," + str(self.ROBOT_ID)
        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def enable_assistive_robot(self):
        """
        Enable assistive mode.

        :return: True if successful, otherwise False.
        """
        command = "GrpOpenFreeDriver" if self.use_new_api else "StartAssistiveMode"
        request = command + "," + str(self.ROBOT_ID)
        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def disable_assistive_robot(self):
        """
        Disable assistive mode.

        :return: True if successful, otherwise False.
        """
        command = "GrpCloseFreeDriver" if self.use_new_api else "CloseAssistiveMode"
        request = command + "," + str(self.ROBOT_ID)
        success, _ = self._send_and_receive(request, verbose=True)
        return success

    def set_speed_ratio(self, speed_ratio):
        """
        Sets the speed ratio if it has changed.

        :param float speed_ratio: Desired speed ratio, range: 0.01–1.0
        :return: True if successful, otherwise False
        """
        if getattr(self, "_last_speed_ratio", None) == speed_ratio:
            return True  # No need to resend the same speed

        request = "SetOverride," + str(self.ROBOT_ID) + "," + str(speed_ratio)
        success, _ = self._send_and_receive(request)

        if success:
            self._last_speed_ratio = speed_ratio
            time.sleep(0.1)  # Allow speed change to take effect

        return success

    def get_pose(self):
        """
        Gets the pose of the robot TCP.

        :return: A pair of a success indicator and the current pose.

            The pose is a list [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.
        """
        command = "ReadActPos" if self.use_new_api else "ReadPcsActualPos"
        request = command + "," + str(self.ROBOT_ID)

        success, params = self._send_and_receive(request)
        if not success or params is None:
            coordinates = None
        else:
            coordinates = (
                [float(s) for s in params[6:12]]
                if self.use_new_api
                else [float(s) for s in params]
            )

        return success, coordinates
    
    def get_current_tool_csc(self):
        """
        Gets the current set tool coordinate system values.

        :return: The success indicator and the current TCP coordinate system.

            The coordinate is a list [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.
        """
        # Note: command message not checked for OLD API, only tested for NEW API
        command = "ReadCurTCP" if self.use_new_api else "ReadCurTCP"
        request = command + "," + str(self.ROBOT_ID)

        success, params = self._send_and_receive(request)        
        if not success or params is None:
            coordinates = None
        else:
            # Note: return message not checked for OLD API, only tested for NEW API
            coordinates = (
                [float(s) for s in params]
                if self.use_new_api
                else [float(s) for s in params]
            )

        return success, coordinates

    def move_linear(self, target, velocity=2000, acc=2500, radius=0, move_type=1,
                    use_joint=0, seek=0, io_bit=0, io_state=0, cmd_id="ID1",
                    tcp_name="TCP_5coil", ucs_name="Base"):
        """
        Moves the robot linearly, supporting both old and new Huayan APIs.

        Parameters:
            target: [x, y, z, rx, ry, rz] – space coordinates in mm and degrees.
            velocity: double – motion velocity (mm/s for linear, °/s for joint).
            acc: double – acceleration.
            radius: double – blending radius (mm).
            move_type: int – 0 = joint move, 1 = linear move.
            use_joint: int – 0 = don't use joint coords, 1 = use joint coords.
            seek: int – 1 enables DI-stop detection.
            io_bit: int – DI index (0–7), only valid if seek=1.
            io_state: int – DI state to stop motion (0/1), only valid if seek=1.
            cmd_id: str – custom waypoint ID.
            tcp_name: str – tool coordinate system name (default "TCP_5coil").
            ucs_name: str – user coordinate system name (default "Base").
        """

        if len(target) != 6:
            raise ValueError("Target must be [x, y, z, rx, ry, rz]")

        if self.use_new_api:
            # New API: WayPoint command
            x, y, z, rx, ry, rz = target
            # Joint placeholders (dJ1–dJ6)
            j1 = j2 = j3 = j4 = j5 = j6 = 0

            command = (
                f"WayPoint,{self.ROBOT_ID},"
                f"{x},{y},{z},{rx},{ry},{rz},"
                f"{j1},{j2},{j3},{j4},{j5},{j6},"
                f"{tcp_name},{ucs_name},"
                f"{velocity*self._last_speed_ratio},{acc},{radius},"
                f"{move_type},{use_joint},{seek},{io_bit},{io_state},{cmd_id}"
            )

        else:
            # Old API: simpler MoveB command
            command = "MoveB"
            command = f"{command},{self.ROBOT_ID},{self.list_to_str(target)}"

        # Send command
        success, _ = self._send_and_receive(command)
        return success

    # Servo functions
    # Doesn't work for us since servo time/cycle overwrites the speed of the robot (direct control of servos)
    # def start_servo(self, servo_time_ms=1000, lookahead_time_ms=100):
    #     cmd = (
    #         "StartServo,"
    #         + str(self.ROBOT_ID)
    #         + ","
    #         + str(servo_time_ms*(1e-3))  # API requests seconds
    #         + ","
    #         + str(lookahead_time_ms*(1e-3))  # API requests seconds
    #     )
    #     success, _ = self._send_and_receive(cmd)
    #     self.servo_started = success
    #     return success

    # def ensure_servo_started(self):
    #     if not getattr(self, "servo_started", False):
    #         return self.start_servo()
    #     return True

    # def move_servo(self, target):
    #     if self.use_new_api:          
    #         if not self.ensure_servo_started():
    #             return False
    #         if not self.tcp_coordinate_system:
    #             success, self.tcp_coordinate_system = self.get_current_tool_csc()
    #             if not success:
    #                 return False

    #         ucs = [0] * 6
    #         request = (
    #             "PushServoP,"
    #             + str(self.ROBOT_ID)
    #             + ","
    #             + self.list_to_str(target)
    #             + ","
    #             + self.list_to_str(ucs)
    #             + ","
    #             + self.list_to_str(self.tcp_coordinate_system)
    #         )
    #         print(f"Sending target: {request}")
    #     else:
    #         request = "MoveB," + str(self.ROBOT_ID) + "," + self.list_to_str(target)

    #     success, _ = self._send_and_receive(request)
    #     # for testing purposes
    #     # time.sleep(0.1)
    #     return success

    def read_force_sensor(self):
        """
        Reads the state of the force sensor.

        :return: A pair of a success indicator and the force sensor values.

            The force sensor values are a list [Fx, Fy, Fz, Mx, My, Mz], where

            Fx, Fy, Fz are the forces in N, and
            Mx, My, Mz are the torques in Nm.
        """
        if (self.use_new_api):
            request = ("ReadForceData," 
                       + str(self.ROBOT_ID))
        else:
            request = "ReadForceSensorData"
        success, params = self._send_and_receive(request)

        if success and params is not None:
            force_sensor_values = [float(s) for s in params]
        else:
            force_sensor_values = None

        return success, force_sensor_values

    def get_motion_state(self):
        """
        Gets the motion state of the robot.

        :return: A MotionState enum value, indicating the motion state of the robot.
        """
        command = "ReadRobotState" if self.use_new_api else "ReadMoveState"
        request = command + "," + str(self.ROBOT_ID)
        success, params = self._send_and_receive(request)

        if not success or params is None:
            print("Could not read robot motion state")
            return MotionState.ERROR

        if self.use_new_api:
            moving_state = bool(int(params[0]))
            error_state = bool(int(params[2]))

            if error_state:
                return MotionState.ERROR

            if moving_state:
                return MotionState.IN_MOTION

            return MotionState.FREE_TO_MOVE
        else:
            code = int(params[0])
            if code == 0:
                return MotionState.FREE_TO_MOVE
            elif code == 1009:
                return MotionState.IN_MOTION
            elif code == 1013:
                return MotionState.WAITING_FOR_EXECUTION
            elif code == 1025:
                return MotionState.ERROR
            else:
                print("Unknown motion state: {}".format(code))
                return MotionState.UNKNOWN

    def move_circular(self, start_position, waypoint, target):
        """
        Moves the robot to the specified space coordinates using circular motion.

        :param: start_position: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm and rx, ry, rz are the rotation angles in degrees.
        :param: waypoint: [x, y, z, rx, ry, rz], where x, y, z are the coordinates in mm and rx, ry, rz are the rotation angles in degrees.
        :param: target: [x, y, z, rx, ry, rz], where

            x, y, z are the coordinates in mm, and
            rx, ry, rz are the rotation angles in degrees.

        :return: True if successful, otherwise False.
        """

        if self.use_new_api:
            # XXX: The velocity is set to 300 and the acceleration to 2500. It turns out that the units specified in the API manual
            #   (mm/s and mm/s^2, respectively) are most likely incorrect. It is uncertain what is the actual unit, but these values
            #   seem to work well enough.
            request = (
                "MoveC,"
                + str(self.ROBOT_ID)
                + ","
                + self.list_to_str(start_position)
                + ","
                + self.list_to_str(waypoint)
                + ","
                + self.list_to_str(target)
                + ",0,1,0,300,2500,1,TCP,Base,0"
            )
        else:
            # Always use movement type 0.
            movement_type_str = "0"

            # Note: The start position is unused in the old version of the Elfin API.
            request = (
                "MoveC,"
                + str(self.ROBOT_ID)
                + ","
                + self.list_to_str(waypoint[:3])
                + ","
                + self.list_to_str(target)
                + ","
                + movement_type_str
            )

        success, _ = self._send_and_receive(request, verbose=True)
        return success   
    
    # Pose tracking / position-follow commands, Elfin v6 API

    def set_pose_tracking_max_motion_limit(
        self,
        speed_ratio,
        verbose=False,
    ):
        """
        Sets the velocity for pose tracking / position-follow mode.
        User speed_ratio to calculate linear velocity in mm/s and orientation velocity in degrees/s.
        Maximum set to 1000mm/s and 360 degrees/s. 

        :return: True if successful, otherwise False.
        """
        #TODO: find a good place to set the maximum speed variables and not use hard-coded values
        request = (
            "SetPoseTrackingMaxMotionLimit,"
            + str(self.ROBOT_ID)
            + ","
            + str(speed_ratio*1000)
            + ","
            + str(speed_ratio*360)
        )

        success, _ = self._send_and_receive(request, verbose=verbose)
        return success

    # Given an error, not sure why, but in the current implementation is not needed
    def set_pose_tracking_stop_timeout(self, timeout_s, verbose=False):
        """
        Sets the pose-tracking stop timeout.

            :param timeout_s: Timeout in seconds.
            :return: True if successful, otherwise False.
        """
        request = (
            "SetPoseTrackingStopTimeOut,"
            + str(self.ROBOT_ID)
            + ","
            + str(timeout_s)
        )
        print(request)
        success, _ = self._send_and_receive(request, verbose=verbose)
        return success

    def set_pose_tracking_pid_params(self, params, verbose=False):
        """
        Setting real-time control incermental movement PID parameters of the robot.
        Defaults in the manual are [5,0.1,0,5,0.1,0].

        :param params: List/tuple of PID parameters in the exact order required
                       by the robot manual.
        :return: True if successful, otherwise False.
        """
        if not params:
            raise ValueError("params must contain at least one PID parameter")

        request = (
            "SetPoseTrackingPIDParams,"
            + str(self.ROBOT_ID)
            + ","
            + ",".join(str(p) for p in params)
        )

        success, _ = self._send_and_receive(request, verbose=verbose)
        return success

    def set_pose_tracking_target_pos(self, target, verbose=False):
        """
        Sets the distance from the targer TCP pose for pose-tracking / position-follow mode.
        For our use case all should be set to 0.
        Target pose format follows the common robot Cartesian pose convention:
            [x,  y,  z,  rx,  ry,  rz]

        where x/y/z are in mm and rx/ry/rz are in degrees

        :param target: [x,  y,  z,  rx,  ry,  rz]
        :return: True if successful, otherwise False.
        """
        if len(target) != 6:
            raise ValueError("Target must be [x,  y,  z,  rx,  ry,  rz]")

        request = (
            "SetPoseTrackingTargetPos,"
            + str(self.ROBOT_ID)
            + ","
            + self.list_to_str(target)
        )

        success, _ = self._send_and_receive(request, verbose=verbose)
        return success
    
    def set_update_tracking_pose(self, target, verbose=False):
        """
        Sets the target TCP pose for pose-tracking / position-follow mode relative to the TCP (displacement_to_target).

        Updated target pose format follows the common robot Cartesian pose convention:
            [x,  y,  z,  rx,  ry,  rz]

        where x/y/z are in mm and rx/ry/rz are in degrees.

        :param target: [x,  y,  z,  rx,  ry,  rz]
        :return: True if successful, otherwise False.
        """
        if len(target) != 6:
            raise ValueError("Target must be [x,  y,  z,  rx,  ry,  rz]")

        request = (
            "SetUpdateTrackingPose,"
            + str(self.ROBOT_ID)
            + ","
            + self.list_to_str(target)
        )

        success, _ = self._send_and_receive(request, verbose=verbose)
        return success

    def set_pose_tracking_state(self, enabled, verbose=False):
        """
        Enables or disables pose-tracking / position-follow mode.

        Assumed command format:
            SetPoseTrackingState,nRbtID,nState,

        :param enabled: True/1 to enable, False/0 to disable.
        :return: True if successful, otherwise False.
        """
        state = 1 if enabled else 0

        request = (
            "SetPoseTrackingState,"
            + str(self.ROBOT_ID)
            + ","
            + str(state)
        )

        success, _ = self._send_and_receive(request, verbose=verbose)
        return success