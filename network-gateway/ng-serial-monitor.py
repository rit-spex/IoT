import serial
import io
from datetime import datetime
import json
from collections import deque
from multiprocessing import Pool
import threading
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

def processSerial(inputString):
    print(hello)
    start = hello.find('{')
    end = hello.find('}') + 1
    if (start > -1) and (end > -1):
        data = hello[start:end]
        d = json.loads(data)
        print d["UUID"]

ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
serialQueue = deque([])
while True:
    hello = sio.readline()
    if hello != '':
       serialQueue.append(hello)

    if len(serialQueue) > 0:
        threading.Thread(target=processSerial, args=(serialQueue.popleft(),)).run()
        
