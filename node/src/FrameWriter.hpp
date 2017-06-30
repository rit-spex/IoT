#pragma once

#include <RH_RF95.h>

#include <cstdint>


namespace SensorNode {

class FrameWriter {
 public:
  FrameWriter() {};
  ~FrameWriter() {};
  void sendMsg(std::uint8_t *vals, std::uint16_t length);

 private:
  bool _valNeedsEscape(uint8_t val);

  RH_RF95 _radio;
  std::uint8_t _buffer[256] = {0};
};

} // end namespace RH_RF95