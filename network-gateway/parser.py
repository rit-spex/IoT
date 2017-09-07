import serial

END = b'\xc0'

class Scanner(object):
    def __init__(self, port):
        self._port = port

        self._port.timeout = 0.25


    def read_msg(self):
        buffer = bytearray()

        in_char = self._port.read(1)

        # keep tacking on characters until the END char is found
        while in_char != END:
            buffer.append(ord(in_char))
            in_char = self.port.read(1)

        self.deslip(buffer)

        return buffer


    def deslip(self, buffer):
        buffer.replace(b'\xdb\xdc', b'\xc0')
        buffer.replace(b'\xdb\xdd', b'\xdb')
