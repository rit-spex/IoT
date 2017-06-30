#pragma once

#include <RH_RF95.h>

#include <cstdint>


namespace SensorNode {

class FrameWriter {
 public:
  FrameWriter();
  ~FrameWriter() {};
  void sendMsg(std::uint8_t *vals, std::uint16_t length);

 private:
  bool _valNeedsEscape(std::uint8_t val);
  void _addMessageByte(std::uint8_t val);
  void _addToBuffer(std::uint8_t val);
  void _setUpBuffer(std::uint8_t *vals, std::uint16_t length);
  void _setup();

  RH_RF95 _radio;
  std::uint8_t _buffer[256] = {0};
  std::uint16_t _cursor = 0;
  bool _isSetup = false;
};

} // end namespace RH_RF95