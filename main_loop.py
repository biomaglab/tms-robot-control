#!/usr/bin/env python3

import time
import socketio
from threading import Lock

import constants as const
import robot.robot as Robot

class RemoteControl:
    def __init__(self, remote_host):
        self.__buffer = []
        self.__remote_host = remote_host
        self.__connected = False
        self.__sio = socketio.Client()

        self.__sio.on('connect', self.__on_connect)
        self.__sio.on('disconnect', self.__on_disconnect)
        self.__sio.on('to_robot', self.__on_message_receive)
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


if __name__ == '__main__':
    rc = RemoteControl('http://127.0.0.1:5000')
    rc.connect()
    robot = Robot.RobotControl(rc)
    previous_robot_status = False
    while True:
        buf = rc.get_buffer()
        if len(buf) == 0:
            pass
        elif any(item in [d['topic'] for d in buf] for item in const.PUB_MESSAGES):
            topic = [d['topic'] for d in buf]

            for i in range(len(buf)):
                if topic[i] in const.PUB_MESSAGES:
                    get_function = {const.FUNCTION_CONNECT_TO_ROBOT: robot.OnRobotConnection,
                                    const.FUNCTION_ROBOT_TRANSFORMATION_MATRIX: robot.OnUpdateRobotTransformationMatrix,
                                    const.FUNCTION_ROBOT_TARGET_MATRIX: robot.OnUpdateRobotTargetMatrix,
                                    const.FUNCTION_RESET_ROBOT_PROCESS: robot.OnResetProcessTracker,
                                    const.FUNCTION_UPDATE_TRACKER_COORDINATES: robot.OnUpdateCoordinates,
                                    const.FUNCTION_UPDATE_TRACKER_FIDUCIALS: robot.OnUpdateTrackerFiducialsMatrix}
                    get_function[const.PUB_MESSAGES.index(topic[i])](buf[i]["data"])

        if robot.trck_init_robot:
            current_tracker_coordinates, current_robot_coordinates, markers_flag = robot.get_coordinates_from_tracker_devices()
            robot_status = robot.robot_control(current_tracker_coordinates, current_robot_coordinates, markers_flag)

            if previous_robot_status != robot_status:
                topic = 'Update robot status'
                data = {'robot_status': robot_status}
                rc.send_message(topic, data)
                previous_robot_status = robot_status

        time.sleep(const.SLEEP_ROBOT)

