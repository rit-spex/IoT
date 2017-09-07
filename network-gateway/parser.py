import serial

END = b'\xc0'

BYTE = "byte"
INT = "int"
FLOAT = "float"

class Scanner(object):
    def __init__(self, port):
        self._port = port

        self._port.timeout = 0.25


    def read_msg(self):
        buffer = bytearray()

        in_char = self._port.read(1)

        # keep tacking on characters until the END char is found
        while in_char != END:
            buffer += in_char
            in_char = self.port.read(1)

        self.deslip(buffer)

        return buffer


    def deslip(self, buffer):
        buffer.replace(b'\xdb\xdc', b'\xc0')
        buffer.replace(b'\xdb\xdd', b'\xdb')

class Parser(object):
    def __init__(self, schema):
        self._schema = schema


    def change_scheme(self, new_schema):
        self._schema = schema

    def parse(self, buffer):
        """Parse a buffer into a dict using a schema."""
        offset = 0
        ret_dict = {}

        for tup in self._schema:
            if tup[1] == FLOAT:
                ret_dict[tup[0]] = struct.unpack('>f', buffer[offset:offset+4])
                offset += 4
            elif tup[1] == INT:
                ret_dict[tup[0]] = struct.unpack('>i', buffer[offset:offset+4])
                offset += 4
            elif tup[1] == BYTE:
                ret_dict[tup[0]] = struct.unpack('>B', buffer[offset:offset+1])
                offset += 1

        return ret_dict
