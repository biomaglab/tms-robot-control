#!/usr/bin/env python3
import os
import sys
import time
from threading import Lock

import socketio
from dotenv import load_dotenv

import robot.constants as const
from robot.control.robot_control import RobotControl

class RemoteControl:
    def __init__(self, remote_host):
        self.__buffer = []
        self.__remote_host = remote_host
        self.__connected = False
        self.__sio = socketio.Client()

        self.__sio.on('connect', self.__on_connect)
        self.__sio.on('disconnect', self.__on_disconnect)
        self.__sio.on('to_robot', self.__on_message_receive)
        self.__sio.on('restart_robot_main_loop', self.__on_restart_main_loop)
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
        self.__sio.emit('from_robot', {'topic' : topic, 'data' : data})

def reset_robot(data):
    robot_control.__init__(remote_control)
    print("Resetting robot control")

# Run the script like this: python main_loop.py [host] [port].
#
# If not given, use the defaults, as shown below.
def get_command_line_arguments():
    default_host = '127.0.0.1'
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

    # Validate environment variables.
    env_vars = [
        'SITE',
        'ROBOT',
        'VERBOSE',
        'MOVEMENT_ALGORITHM',
        'USE_FORCE_SENSOR',
        'DWELL_TIME',
        'SAFE_HEIGHT',
        'ROBOT_SPEED',
        'STOP_ROBOT_IF_HEAD_NOT_VISIBLE',
        'TUNING_INTERVAL',
    ]
    for var in env_vars:
        if os.getenv(var) is None:
            print(f"Environment variable {var} not provided, exiting")
            return None

    # Create configuration dictionary.
    site = os.getenv('SITE')
    robot = os.getenv('ROBOT')
    verbose = os.getenv('VERBOSE').lower() == 'true'
    movement_algorithm = os.getenv('MOVEMENT_ALGORITHM')
    dwell_time = float(os.getenv('DWELL_TIME'))
    use_force_sensor = os.getenv('USE_FORCE_SENSOR').lower() == 'true'
    safe_height = float(os.getenv('SAFE_HEIGHT'))
    robot_speed = float(os.getenv('ROBOT_SPEED'))
    stop_robot_if_head_not_visible = os.getenv('STOP_ROBOT_IF_HEAD_NOT_VISIBLE').lower() == 'true'

    # If tuning interval is not provided, set it to None, otherwise convert to float.
    tuning_interval = os.getenv('TUNING_INTERVAL')
    if tuning_interval == "":
        tuning_interval = None
    else:
        tuning_interval = float(tuning_interval)

    config = {
        'site': site,
        'robot': robot,
        'verbose': verbose,
        'movement_algorithm': movement_algorithm,
        'dwell_time': dwell_time,
        'use_force_sensor': use_force_sensor,
        'safe_height': safe_height,
        'robot_speed': robot_speed,
        'stop_robot_if_head_not_visible': stop_robot_if_head_not_visible,
        'tuning_interval': tuning_interval,
    }

    # Print configuration.
    print("Configuration")
    print("")
    for key, value in config.items():
        key_formatted = key.replace('_', ' ').capitalize()
        print("{}: {}".format(key_formatted, value))

    print("")

    return config


if __name__ == '__main__':
    host, port = get_command_line_arguments()

    config = get_config()
    if config is None:
        exit(1)

    # Connect to neuronavigation
    url = 'http://{}:{}'.format(host, port)

    remote_control = RemoteControl(url)
    remote_control.connect()

    # Initialize robot controller

    site = config['site']
    robot = config['robot']

    site_config = const.SITE_CONFIG[site]

    # If robot is set to elfin_new_api, use elfin config instead.
    robot_config = const.ROBOT_CONFIG[robot if robot != "elfin_new_api" else "elfin"]

    robot_control = RobotControl(
        remote_control=remote_control,
        config=config,
        site_config=site_config,
        robot_config=robot_config,
    )

    previous_robot_status = False
    while True:
        buf = remote_control.get_buffer()
        if len(buf) == 0:
            pass

        elif any(item in [d['topic'] for d in buf] for item in const.PUB_MESSAGES):
            topic = [d['topic'] for d in buf]

            for i in range(len(buf)):
                if topic[i] in const.PUB_MESSAGES:
                    get_function = {
                        const.FUNCTION_CONNECT_TO_ROBOT: robot_control.OnRobotConnection,
                        const.FUNCTION_UPDATE_ROBOT_TARGET: robot_control.OnUpdateRobotTargetMatrix,
                        const.FUNCTION_RESET_ROBOT_PROCESS: robot_control.OnResetProcessTracker,
                        const.FUNCTION_UPDATE_TRACKER_COORDINATES: robot_control.OnUpdateCoordinates,
                        const.FUNCTION_UPDATE_TRACKER_FIDUCIALS: robot_control.OnUpdateTrackerFiducialsMatrix,
                        const.FUNCTION_COLLECT_COORDINATES_TO_ROBOT_MATRIX: robot_control.OnCreatePoint,
                        const.FUNCTION_RESET_ROBOT_MATRIX: robot_control.OnResetRobotMatrix,
                        const.FUNCTION_ROBOT_MATRIX_ESTIMATION: robot_control.OnRobotMatrixEstimation,
                        const.FUNCTION_LOAD_ROBOT_MATRIX: robot_control.OnLoadRobotMatrix,
                        const.FUNCTION_RESET_ROBOT: reset_robot,
                        const.FUNCTION_COIL_AT_TARGET: robot_control.OnCoilAtTarget,
                        const.FUNCTION_DISTANCE_TO_TARGET: robot_control.OnDistanceToTarget,
                        const.FUNCTION_SEND_OBJECTIVE_FROM_NEURONAVIGATION_TO_ROBOT: robot_control.OnSetObjective,
                    }
                    get_function[const.PUB_MESSAGES.index(topic[i])](buf[i]["data"])

        if robot_control.robot:
            robot_control.update_robot_pose()
            robot_status = robot_control.get_robot_status()

            # If changed, send robot status to neuronavigation via remote control.
            if previous_robot_status != robot_status:
                topic = 'Update robot status'
                data = {'robot_status': robot_status}
                remote_control.send_message(topic, data)

            previous_robot_status = robot_status

        time.sleep(robot_config['sleep'])
