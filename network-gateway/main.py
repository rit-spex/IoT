import scanner
import serial
import time
import json
import sys

from socketIO_client import SocketIO, LoggingNamespace

sockets = {}

def sendDataViaSocket(socketData, uuid):
  socketId = uuid
  if socketId in sockets:
      socket = sockets.get(socketId)
      socket.emit('sensorData', socketData)
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


def format_msg(payload, uuid):
    py_structure = {
        'dateCreated': time.time(),
        'payload': payload,
        'name': uuid
    }

    return json.dumps(py_structure)

if __name__ == "__main__":
    port_name = ""
    if(len(sys.argv) > 1):
        port_name = sys.argv[1]
    else:
        port_name = "COM6"

    port = serial.Serial(port_name, timeout=1)
    scan = scanner.Scanner(port)
    pars = scanner.Parser()


    while(True):
        # chop off id and schema
        msg = scan.read_msg()
        uuid = msg[0]
        schema = msg[1]
        try:
            data_dict = pars.parse(msg[2:], schema)
            sendDataViaSocket(format_msg(data_dict, uuid), uuid)
        except KeyError:
            print("Unknown schema")