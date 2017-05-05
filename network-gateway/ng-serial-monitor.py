import serial
import io
from datetime import datetime
import time
import json
from collections import deque
from multiprocessing import Pool
import threading
from socketIO_client import SocketIO, LoggingNamespace

SERIAL_PORT = 'COM6'
sockets = {}

def sendDataViaSocket(socketData):
  socketId = socketData['UUID']
  if sockets.has_key(socketId):
      socket = sockets.get(socketId)
      sendSocketData(socket, socketData)
  else:
      sockets[socketId] = connectToSocket(socketId)

def connectToSocket(socketId):
    def on_response(data):
      print('response received: ')
      print(data)

    def joinServer(*args):
        print('connecting to HABnet server')
        socketIO.emit('join', {'name': 'pythonClient2', 'type': 'dataSource'})

    def disconnectFromServer(*args):
        print('disconnecting from HABnet server')

    def addName(*args):
      print('join callback')

    socketIO = SocketIO('localhost', 3000)
    socketIO.emit('join', {'name': socketId, 'type': 'dataSource'})
    socketIO.on('joinedSuccessfully', addName)
    socketIO.on('connect', joinServer)
    socketIO.on('disconnect', disconnectFromServer)
    return socketIO



def sendSocketData(socket, payload):
    message = {
      'dateCreated': time.time(),
      'name': payload['UUID'],
      'payload': payload
    }
    socket.emit('sensorData', json.dumps(message))
    print 'data sent'
    #print json.dumps(message)

def processSerial(inputString):
    start = hello.find('{')
    end = hello.find('}') + 1
    if (start > -1) and (end > -1):
        data = hello[start:end]
        d = json.loads(data)
        sendDataViaSocket(d)

def workThroughQueue():
  while True:
      if len(serialQueue) > 0:
          threading.Thread(target=processSerial, args=(serialQueue.popleft(),)).run()
      
ser = serial.Serial(SERIAL_PORT, 115200, timeout=0)
#sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
serialQueue = deque([])
threading.Thread(target=workThroughQueue).start()
while True:
    #hello = sio.readline()
    hello = ser.readline()
    if hello != '':
       serialQueue.append(hello)
