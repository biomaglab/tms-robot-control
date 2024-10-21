#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This scripts runs a Socket.IO server that forwards all the messages from
# the neuronavigation system to the robot. That is, upon receiving a
# 'from_neuronavigation' message, it emits 'to_robot' message with the same
# data.
#
# Important:
#      :param data: The data to send to the client or clients. Data can be of
#      type ``str``, ``bytes``, ``list`` or ``dict``. To send
#      multiple arguments, use a tuple where each element is of
#      one of the types indicated above.
#

import asyncio
import sys

import nest_asyncio
import socketio
import uvicorn


nest_asyncio.apply()

default_host = '127.0.0.1'

if len(sys.argv) == 3:
    host = sys.argv[1]
    port = int(sys.argv[2])
elif len(sys.argv) == 2:
    host = default_host
    port = int(sys.argv[1])
else:
    print(f'Usage: python {sys.argv[0]} [host] port')
    sys.exit(1)

sio = socketio.AsyncServer(async_mode='asgi')
app = socketio.ASGIApp(sio)

@sio.event
def from_neuronavigation(sid, msg):
    asyncio.create_task(sio.emit('to_robot', msg))
    print('Forwarding neuronavigation -> robot: %s' % str(msg))

@sio.event
def from_robot(sid, msg):
    asyncio.create_task(sio.emit('to_neuronavigation', msg))
    print('Forwarding robot -> neuronavigation: %s' % str(msg))

@sio.event
def restart_robot_main_loop(sid):
    asyncio.create_task(sio.emit('restart_robot_main_loop'))
    print('Restarting robot main_loop')


if __name__ == '__main__':
    uvicorn.run(app, host=host, port=port, loop='asyncio')
