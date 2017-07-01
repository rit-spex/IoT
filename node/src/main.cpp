#include <Arduino.h>

#include "FrameWriter.hpp"

#include <cstdint>

SensorNode::FrameWriter frame;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("sending message");
  uint8_t msg[] = "hello";
  frame.sendMsg(msg, sizeof(msg));
  delay(1000);
}