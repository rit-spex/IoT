#include <Arduino.h>

#include "FrameWriter.hpp"

#include <Adafruit_BME280.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>

#include <cstdint>
#include <string.h>

SensorNode::FrameWriter frame;
#define NUM_VALS 12
#define MSG_SIZE 50

// Sensors
Adafruit_BME280 bme;
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

void setup() {
  Serial.begin(9600);

  // BME280
  if(!bme.begin())
    Serial.println("BME init failed");
  if(!accel.begin())
    Serial.println("Accel init failed");
  if(!mag.begin())
    Serial.println("Mag init failed");
  if(!gyro.begin())
    Serial.println("Gyro init failed");
  

  
}
uint8_t msg[MSG_SIZE];
void loop() {
  //frame.sendMsg((uint8_t*)msg, (uint16_t)sizeof(msg));
  //Serial.println(frame.maxMsg());
  //Serial.println("sending message");
  //std::uint8_t msg[] = "hello";
  //frame.sendMsg(msg, sizeof(msg));
  msg[0] = 0xde;
  msg[1] = 0xad;

  int offset = 2;
  float vals[NUM_VALS];

  sensors_event_t event;
  accel.getEvent(&event);
  gyro.getEvent(&event);
  mag.getEvent(&event);

  vals[0] = event.acceleration.x;
  vals[1] = event.acceleration.y;
  vals[2] = event.acceleration.z;
  vals[3] = event.gyro.x;
  vals[4] = event.gyro.y;
  vals[5] = event.gyro.z;
  vals[6] = event.magnetic.x;
  vals[7] = event.magnetic.y;
  vals[8] = event.magnetic.z;
  vals[9] = bme.readTemperature();
  vals[10] = bme.readPressure();
  vals[11] = bme.readHumidity();

  for(int i = 0; i < NUM_VALS; i++) {
    memcpy(msg + offset, vals + i, sizeof(float));
    offset += sizeof(float);
  }
  frame.sendMsg(msg, MSG_SIZE);
  delay(1000);
}
