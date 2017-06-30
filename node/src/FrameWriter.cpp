#include "FrameWriter.hpp"

using std::uint8_t;
using std::uint16_t;

namespace {
  constexpr uint16_t LEN = 256;
  constexpr uint8_t STX = 0x01;
  constexpr uint8_t ETX = 0x03;
  constexpr uint8_t ESC = 0xfe;
}
namespace SensorNode {

void FrameWriter::sendMsg(uint8_t *vals, uint16_t length) {
  uint16_t cursor = 0;

  _buffer[cursor] = ESC;
  cursor++;
  _buffer[cursor] = STX;
  cursor++;

  for(uint16_t i = 0; i < length; i++) {
    if(_valNeedsEscape(vals[i])) {
      _buffer[cursor] = ESC;
      cursor++;
    }

    _buffer[cursor] = vals[i];
    cursor++;
  }

  _buffer[cursor] = ESC;
  cursor++;
  _buffer[cursor] = ETX;
  cursor++;
}

bool FrameWriter::_valNeedsEscape(uint8_t val) {
  switch(val) {
    case 0x01:
    case 0x03:
    case 0xfe:
      return true;
    default:
      return false;
  }
}

}