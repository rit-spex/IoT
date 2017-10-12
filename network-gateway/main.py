import schemas
import scanner
import serial

if __name__ == "__main__":
    port = serial.Serial("COM3", timeout=1)
    scan = scanner.Scanner(port)
    pars = scanner.Parser(schemas.SCHEMAS[1])

    while(True):
        # chop off id and schema
        msg = scan.read_msg()[2:]
        print(repr(pars.parse(msg)))
