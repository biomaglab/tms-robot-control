#!/usr/bin/env python3

import time
import socketio


class RemoteControl:
    def __init__(self, buffer, remote_host):
        self.__buffer = buffer
        self.__remote_host = remote_host
        self.__connected = False
        self.__sio = socketio.Client()

        self.__sio.on('connect', self.__on_connect)
        self.__sio.on('disconnect', self.__on_disconnect)
        self.__sio.on('to_robot', self.__on_message_receive)

    def __on_connect(self):
        print("Connected to {}".format(self.__remote_host))
        self.__connected = True

    def __on_disconnect(self):
        print("Disconnected")
        self.__connected = False

    def __on_message_receive(self, msg):
        topic = msg["topic"]
        data = msg["data"]
        if data is None:
            data = {}

        self.__buffer[0] = {'topic' : topic, 'data' : data}

    def connect(self):
        self.__sio.connect(self.__remote_host)

        while not self.__connected:
            print("Connecting...")
            time.sleep(1.0)


if __name__ == '__main__':
    buffer = [{}]
    rc = RemoteControl(buffer, 'http://127.0.0.1:5000')
    rc.connect()
    while True:
        print('Sleeping for 5 sec ...')
        time.sleep(5.0)
        print('Checking the buffer. It contains %s' % str(buffer[0]))

