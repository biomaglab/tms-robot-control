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

def get_environment_variables():
    # Load environment variables from .env file
    load_dotenv()

    site = os.getenv('SITE')
    if site is None or site == '':
        print("SITE environment variable not provided, exiting")
        return None

    robot = os.getenv('ROBOT')
    if robot is None or robot == '':
        print("ROBOT environment variable not provided, exiting")
        return None

    movement_algorithm = os.getenv('MOVEMENT_ALGORITHM')
    if movement_algorithm is None or movement_algorithm == '':
        print("MOVEMENT_ALGORITHM environment variable not provided, exiting")
        return None

    use_force_sensor_param = os.getenv('USE_FORCE_SENSOR')
    if use_force_sensor_param is None or use_force_sensor_param == '':
        print("USE_FORCE_SENSOR environment variable not provided, exiting")
        return None

    use_force_sensor = use_force_sensor_param.lower() == 'true'

    print("Configuration")
    print("")
    print("Site: {}".format(site))
    print("Robot: {}".format(robot))
    print("Movement algorithm: {}".format(movement_algorithm))
    print("Force sensor: {}".format(use_force_sensor))
    print("")

    config = {
        'site': site,
        'robot': robot,
        'movement_algorithm': movement_algorithm,
        'use_force_sensor': use_force_sensor,
    }
    return config


if __name__ == '__main__':
    host, port = get_command_line_arguments()

    config = get_environment_variables()
    if config is None:
        exit(1)

    # Connect to neuronavigation
    url = 'http://{}:{}'.format(host, port)

    remote_control = RemoteControl(url)
    remote_control.connect()

    # Initialize robot controller
    site = config['site']
    robot = config['robot']
    movement_algorithm = config['movement_algorithm']
    use_force_sensor = config['use_force_sensor']

    site_config = const.SITE_CONFIG[site]
    robot_config = const.ROBOT_CONFIG[robot if robot != "elfin_new_api" else "elfin"]

    robot_control = RobotControl(
        robot_type=robot,
        remote_control=remote_control,
        site_config=site_config,
        robot_config=robot_config,
        use_force_sensor=use_force_sensor,
        movement_algorithm_name=movement_algorithm,
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
