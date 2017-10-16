import scanner
import serial
import time
import json

def format_msg(payload, uuid):
    py_structure = {
        'dateCreated': time.time(),
        'payload': payload,
        'name': uuid
    }

    return json.dumps(py_structure)

if __name__ == "__main__":
    port = serial.Serial("COM6", timeout=1)
    scan = scanner.Scanner(port)
    pars = scanner.Parser()

    while(True):
        # chop off id and schema
        msg = scan.read_msg()
        uuid = msg[0]
        schema = msg[1]
        try:
            data_dict = pars.parse(msg[2:], schema)
            print(format_msg(data_dict, uuid))
        except KeyError:
            print("Unknown schema")