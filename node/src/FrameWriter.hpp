#pragma once

#include <RH_RF95.h>


namespace SensorNode {

class FrameWriter {
 public:
  FrameWriter() {};
  ~FrameWriter() {};
  void sendMsg();

 private:
  RH_RF95 _radio;
};

} // end namespace RH_RF95