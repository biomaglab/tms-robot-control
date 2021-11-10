#!/usr/bin/env python3

import time
import socketio
from threading import Lock


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
    while True:
        print('Sleeping for 5 sec ...')
        time.sleep(5.0)
        buf = rc.get_buffer()
        print('Checking the buffer. It contains %i messages' % len(buf))
        print('Sleeping for 5 sec ...')
        time.sleep(5.0)
        topic = 'Delete all markers'
        print('Emitting a message, topic : %s' % topic)
        rc.send_message(topic)


