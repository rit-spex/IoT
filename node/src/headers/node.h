#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include <Wire.h>
#include <SPI.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>

#include <RH_RF95.h>
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0

#define SEALEVELPRESSURE_HPA (1013.25)
#define GPSSerial Serial1
#define GPSECHO false

void setupSerial(void);
bool setupAccelerometer(Adafruit_LSM303_Accel_Unified& accel, bool debug = false);
bool setupMagnetometer(Adafruit_LSM303_Mag_Unified& mag, bool debug = false);
bool setupGyroscope(Adafruit_L3GD20_Unified& gyro, bool debug = false);
bool setupColorSensor(Adafruit_TCS34725& tcs, bool debug = false);
bool setupBarometer(Adafruit_BME280& bme, bool debug = false);
bool setupGPS(Adafruit_GPS& GPS, bool debug = false);
bool setupLoRa(RH_RF95& LoRa);
void sendDataViaLoRa(RH_RF95& LoRa);
char* getGPSData(Adafruit_GPS& GPS);
void prettyPrintAccelerometerData(Adafruit_LSM303_Accel_Unified& accel, sensors_event_t& event);
void prettyPrintMagnetometerData(Adafruit_LSM303_Mag_Unified& mag, sensors_event_t& event);
void prettyPrintGyroscopeData(Adafruit_L3GD20_Unified& gyro, sensors_event_t& event);
void prettyPrintColorSensorData(Adafruit_TCS34725& tcs);
void prettyPrintBarometerData(Adafruit_BME280& bme);
void prettyPrintGPSData(Adafruit_GPS& GPS);
void displaySensorDetails(void);
