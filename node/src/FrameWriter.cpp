#include "FrameWriter.hpp"
#include <cstdint>

using std::uint8_t;

namespace {
  constexpr uint8_t STX = 0x01;
  constexpr uint8_t ETX = 0x03;
  constexpr uint8_t ESC = 0xfe;
}
namespace SensorNode {

void FrameWriter::sendMsg() {
  int8_t buffer[256] = {0};

  buffer[0] = ESC;
  buffer[1] = STX;
}

}