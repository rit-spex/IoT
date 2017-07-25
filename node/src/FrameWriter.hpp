#pragma once

#include <RH_RF95.h>

#include <cstdint>

#define MAX_RAW 125
#define MAX_BUF 251

namespace SensorNode {

class FrameWriter {
 public:
  FrameWriter();
  ~FrameWriter() {};
  void sendMsg(std::uint8_t *vals, std::uint16_t length);

 private:
  bool _isSetUp = false;
  void _slip();
  bool _setup();

  RH_RF95 _radio;
  std::uint8_t _raw_buffer[MAX_RAW] = {0};
  std::uint8_t _msg_buffer[MAX_BUF] = {0};
};

} // end namespace RH_RF95
