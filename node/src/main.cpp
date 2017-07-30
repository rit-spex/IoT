#include <Arduino.h>

#include "FrameWriter.hpp"

#include <cstdint>

SensorNode::FrameWriter frame;

void setup() {
  Serial.begin(9600);
}
char* msg = "hello";
void loop() {
  frame.sendMsg((uint8_t*)msg, (uint16_t)sizeof(msg));
  //Serial.println(frame.maxMsg());
  //Serial.println("sending message");
  //std::uint8_t msg[] = "hello";
  //frame.sendMsg(msg, sizeof(msg));
  delay(1000);
}
