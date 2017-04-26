import serial
import io
from datetime import datetime
from socketIO_client import SocketIO, LoggingNamespace

def on_bbb_response(*args):
    print('on_bbb_response', args)

def joinServer(*args):
    print('connecting to HABnet server')
    socketIO.emit('join', { 'name': 'pythonClient2', 'type': 'dataSource' })

def disconnectFromServer(*args):
    print('disconnecting from HABnet server')

def addName(*args):
  print('join callback')

socketIO = SocketIO('localhost', 3000)

socketIO.on('joinedSuccessfully', addName)
socketIO.on('connect', joinServer)
socketIO.on('disconnect', disconnectFromServer)
var = 1

socketIO.emit('sensorData', {'dateCreated': '999', 'payload': {'python': 4}, 'name': 'pythonClient2'}, on_bbb_response)
socketIO.wait_for_callbacks(seconds=1)
socketIO.emit('sensorData', {'dateCreated': '999', 'payload': {'python': 5}, 'name': 'pythonClient2'}, on_bbb_response)
socketIO.wait_for_callbacks(seconds=1)
while var == 1:
    socketIO.emit('sensorData', {'dateCreated': '999', 'payload': {'python': 6}, 'name': 'pythonClient2'}, on_bbb_response)
    socketIO.wait_for_callbacks(seconds=1)
    print('Sent socket Data')





