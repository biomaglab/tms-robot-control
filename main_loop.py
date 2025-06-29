#!/usr/bin/env python3
import os
import sys
import time
from threading import Lock

import numpy as np
import socketio
from dotenv import load_dotenv

import robot.constants as const
from robot.control.color import Color
from robot.control.robot_control import RobotControl, RobotObjective


class RemoteControl:
    def __init__(self, remote_host):
        self.__buffer = []
        self.__remote_host = remote_host
        self.__connected = False
        self.__sio = socketio.Client()

        self.__sio.on("connect", self.__on_connect)
        self.__sio.on("disconnect", self.__on_disconnect)
        self.__sio.on("to_robot", self.__on_message_receive)
        self.__sio.on("restart_robot_main_loop", self.__on_restart_main_loop)
        self.__lock = Lock()

    def __on_connect(self):
        print("Connected to {}".format(self.__remote_host))
        self.__connected = True

    def __on_disconnect(self):
        print("Disconnected")
        self.__connected = False

    def __on_message_receive(self, msg):
        self.__lock.acquire()
        self.__buffer.append(msg)
        self.__lock.release()

    def __on_restart_main_loop(self):
        """Restarts the current program.
        Automatically restarts the main_loop.py when Invesalius is started.
        The restart only works if the code is running in the python console.
        For pycharm users, the "Edit configuration" can be used to select the option "Run with Python Console".
        The conventional pycharm "run" can also be used, but every time InVesalius is started the main_loop will stop and
        requires to be manually started again.
        """
        # TODO: Disable for now - there needs to be another way around the problems that InVesalius
        #   causes here when it starts.
        pass

    def get_buffer(self):
        self.__lock.acquire()
        res = self.__buffer.copy()
        self.__buffer = []
        self.__lock.release()
        return res

    def connect(self):
        self.__sio.connect(self.__remote_host)

        while not self.__connected:
            print("Connecting...")
            time.sleep(1.0)

    def send_message(self, topic, data={}):
        self.__sio.emit("from_robot", {"topic": topic, "data": data})


# Run the script like this: python main_loop.py [host] [port].
#
# If not given, use the defaults, as shown below.
def get_command_line_arguments():
    default_host = "127.0.0.1"
    default_port = 5000

    if len(sys.argv) == 3:
        host = sys.argv[1]
        port = int(sys.argv[2])
    elif len(sys.argv) == 2:
        host = default_host
        port = int(sys.argv[1])
    else:
        host = default_host
        port = default_port

    return host, port


def get_config():
    # Load environment variables from .env file.
    load_dotenv()

    # List environment variables.
    env_vars = [
        "SITE",
        "ROBOT",
        "VERBOSE",
        "MOVEMENT_ALGORITHM",
        "USE_FORCE_SENSOR",
        "USE_PRESSURE_SENSOR",
        "COM_PORT_PRESSURE_SENSOR",
        "DWELL_TIME",
        "SAFE_HEIGHT",
        "DEFAULT_SPEED_RATIO",
        "TUNING_SPEED_RATIO",
        "STOP_ROBOT_IF_HEAD_NOT_VISIBLE",
        "TUNING_INTERVAL",
        "WAIT_FOR_KEYPRESS_BEFORE_MOVEMENT",
        "TRANSLATION_THRESHOLD",
        "ROTATION_THRESHOLD",
    ]
    for var in env_vars:
        if os.getenv(var) is None:
            print(f"Environment variable {var} not provided, exiting")
            return None

    # Create configuration dictionary.
    site = os.getenv("SITE")
    robot = os.getenv("ROBOT")
    verbose = os.getenv("VERBOSE").lower() == "true"
    movement_algorithm = os.getenv("MOVEMENT_ALGORITHM")
    dwell_time = float(os.getenv("DWELL_TIME"))
    use_force_sensor = os.getenv("USE_FORCE_SENSOR").lower() == "true"
    use_pressure_sensor = os.getenv("USE_PRESSURE_SENSOR").lower() == "true"
    com_port_pressure_sensor = os.getenv("COM_PORT_PRESSURE_SENSOR")
    safe_height = float(os.getenv("SAFE_HEIGHT"))
    default_speed_ratio = float(os.getenv("DEFAULT_SPEED_RATIO"))
    tuning_speed_ratio = float(os.getenv("TUNING_SPEED_RATIO"))
    stop_robot_if_head_not_visible = (
        os.getenv("STOP_ROBOT_IF_HEAD_NOT_VISIBLE").lower() == "true"
    )
    wait_for_keypress_before_movement = (
        os.getenv("WAIT_FOR_KEYPRESS_BEFORE_MOVEMENT").lower() == "true"
    )
    translation_threshold = float(os.getenv("TRANSLATION_THRESHOLD"))
    rotation_threshold = float(os.getenv("ROTATION_THRESHOLD"))

    # If tuning interval is not provided, set it to None, otherwise convert to float.
    tuning_interval = os.getenv("TUNING_INTERVAL")
    if tuning_interval == "":
        tuning_interval = None
    else:
        tuning_interval = float(tuning_interval)

    config = {
        "site": site,
        "robot": robot,
        "verbose": verbose,
        "movement_algorithm": movement_algorithm,
        "dwell_time": dwell_time,
        "use_force_sensor": use_force_sensor,
        "use_pressure_sensor": use_pressure_sensor,
        "com_port_pressure_sensor": com_port_pressure_sensor,
        "safe_height": safe_height,
        "default_speed_ratio": default_speed_ratio,
        "tuning_speed_ratio": tuning_speed_ratio,
        "stop_robot_if_head_not_visible": stop_robot_if_head_not_visible,
        "tuning_interval": tuning_interval,
        "wait_for_keypress_before_movement": wait_for_keypress_before_movement,
        "translation_threshold": translation_threshold,
        "rotation_threshold": rotation_threshold,
    }

    # Print configuration.
    print("Configuration")
    print("")
    for key, value in config.items():
        key_formatted = key.replace("_", " ").capitalize()
        print("{}: {}{}{}".format(key_formatted, Color.BOLD, value, Color.END))

    print("")
    if wait_for_keypress_before_movement:
        print(
            "{}Note: The robot only performs a movement when a key is pressed{}".format(
                Color.BOLD, Color.END
            )
        )
        print("")

    # Validate configuration.
    if site not in const.SITE_CONFIG:
        print("Invalid site configuration, exiting.")
        return None

    if default_speed_ratio < 0.01 or default_speed_ratio > 1:
        print("Default speed ratio must be between 0.01 and 1, exiting.")
        return None

    if tuning_speed_ratio < 0.01 or tuning_speed_ratio > 1:
        print("Tuning speed ratio must be between 0.01 and 1, exiting.")
        return None

    return config


def main(connection=None):
    config = get_config()
    if config is None:
        exit(1)

    # Configure logging.
    np.set_printoptions(formatter={"float": "{:0.1f}".format})

    # Initialize robot controller

    site = config["site"]
    robot = config["robot"]

    site_config = const.SITE_CONFIG[site]

    # If robot is set to elfin_new_api, use elfin config instead.
    robot_config = const.ROBOT_CONFIG[robot if robot != "elfin_new_api" else "elfin"]

    try:
        from .robot_api import RobotApi

        robot_control = RobotControl(
            remote_control=None,
            config=config,
            site_config=site_config,
            robot_config=robot_config,
            connection=connection,
        )
        RobotApi(connection, robot_control)
        connection_type = "ros"

    except:
        host, port = get_command_line_arguments()
        # Connect to server
        url = "http://{}:{}".format(host, port)
        remote_control = RemoteControl(url)
        remote_control.connect()
        robot_control = RobotControl(
            remote_control=remote_control,
            config=config,
            site_config=site_config,
            robot_config=robot_config,
            connection=None,
        )
        connection_type = "server"

    previous_success = False
    while True:
        if connection_type == "server":
            buf = remote_control.get_buffer()
            if len(buf) == 0:
                pass

            elif any(item in [d["topic"] for d in buf] for item in const.PUB_MESSAGES):
                topic = [d["topic"] for d in buf]

                for i in range(len(buf)):
                    if topic[i] in const.PUB_MESSAGES:
                        get_function = {
                            const.FUNCTION_CONNECT_TO_ROBOT: robot_control.on_robot_connection,
                            const.FUNCTION_SET_TARGET: robot_control.on_set_target,
                            const.FUNCTION_UNSET_TARGET: robot_control.on_unset_target,
                            const.FUNCTION_SET_TRACKER_FIDUCIALS: robot_control.on_set_tracker_fiducials,
                            const.FUNCTION_UPDATE_TRACKER_POSES: robot_control.on_update_tracker_poses,
                            const.FUNCTION_COLLECT_COORDINATES_TO_ROBOT_MATRIX: robot_control.on_create_point,
                            const.FUNCTION_RESET_ROBOT_MATRIX: robot_control.on_reset_robot_matrix,
                            const.FUNCTION_ROBOT_MATRIX_ESTIMATION: robot_control.on_robot_matrix_estimation,
                            const.FUNCTION_SET_ROBOT_TRANSFORMATION_MATRIX: robot_control.on_set_robot_transformation_matrix,
                            const.FUNCTION_COIL_AT_TARGET: robot_control.on_coil_at_target,
                            const.FUNCTION_UPDATE_DISPLACEMENT_TO_TARGET: robot_control.on_update_displacement_to_target,
                            const.FUNCTION_SET_OBJECTIVE: robot_control.on_set_objective,
                            const.FUNCTION_SET_FREE_DRIVE: robot_control.on_set_freedrive,
                            const.FUNCTION_CHECK_CONNECTION: robot_control.on_check_connection_robot,
                        }
                        get_function[const.PUB_MESSAGES.index(topic[i])](buf[i]["data"])

        if robot_control.robot:
            success = robot_control.update()

            if previous_success != success:
                # Reset objective if robot update was not successful.
                robot_control.objective = RobotObjective.NONE
                robot_control.send_objective_to_neuronavigation()

                if connection_type == "ros":
                    connection.update_robot_status(success)

                elif connection_type == "server":
                    # Send robot status to tms_robot_control via remote control.
                    topic = "Robot to Neuronavigation: Update robot status"
                    data = {"robot_status": success}
                    remote_control.send_message(topic, data)

            previous_success = success

        time.sleep(robot_config["sleep"])


if __name__ == "__main__":
    main()
