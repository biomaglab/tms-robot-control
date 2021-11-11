#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This scripts runs a Socket.IO server that forwards all the messages from
# the neuronavigation system to the robot. That is, upon receiving a
# 'from_neuronavigation' message, it emits 'to_robot' message with the same
# data.

import asyncio
import sys

import nest_asyncio
import socketio
import uvicorn


nest_asyncio.apply()

if len(sys.argv) != 2:
    print ('Usage: python %s port' % sys.argv[0])
    sys.exit(1)

port = int(sys.argv[1])

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

if __name__ == '__main__':
    uvicorn.run(app, port=port, host='0.0.0.0', loop='asyncio')
