import serial
import io
from datetime import datetime
import json
from socketIO_client import SocketIO, LoggingNamespace
SERIAL_PORT = 'COM6'

def connectToSocket(socketId):
    socketIO = SocketIO('localhost', 3000)
    socketIO.on('joinedSuccessfully', addName)
    socketIO.on('connect', joinServer)
    socketIO.on('disconnect', disconnectFromServer)

    def on_response(data):
      print('response received: ')
      print(data)

    def joinServer(*args):
        print('connecting to HABnet server')
        socketIO.emit('join', { 'name': 'pythonClient2', 'type': 'dataSource' })

    def disconnectFromServer(*args):
        print('disconnecting from HABnet server')

    def addName(*args):
      print('join callback')

    def sendSocketData(payload):
      socketIO.emit('sensorData', {'dateCreated': '999', 'payload': {'python': 4}, 'name': 'pythonClient2'}, on_response)

ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
var = 1
while True:
    hello = sio.readline()
    if hello:
        print(hello)
        start = hello.index('{')
        end = hello.index('}') + 1
        data = hello[start:end]
        d = json.loads(data)
        print d["UUID"]


#var = 1
#while var == 1:
#    socketIO.emit('sensorData', {'dateCreated': '999', 'payload': {'python': 6}, 'name': 'pythonClient2'}, on_response)
#    socketIO.wait_for_callbacks(seconds=1)
#    print('Sent socket Data')
#
#



