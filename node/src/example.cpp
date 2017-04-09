#include <Arduino.h>

#include <Wire.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_9DOF.h>

#include "Adafruit_TCS34725.h"

#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

Adafruit_BME280 bme; // I2C


void displaySensorDetails(void);

void setup(void)
{
    Serial.begin(115200);
    Serial.println(F("Adafruit 9DOF Tester")); Serial.println("");

  /* Initialise the sensors */
    if(!accel.begin())
    {
        delay(5000);
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1){Serial.println('1');};
    }
    if(!mag.begin())
    {
        Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1){Serial.println('2');};
    }
    if(!gyro.begin())
    {
        Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
        while(1){Serial.println('3');};
    }
    if (tcs.begin()) {
        Serial.println("Found sensor");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1){Serial.println('4');};
    }

    Serial.println(F("BME280 test"));

    if (!bme.begin()) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1){Serial.println('5');};
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();
}

void loop(void)
{
    /* Get a new sensor event */
    sensors_event_t event;

    /* Display the results (acceleration is measured in m/s^2) */
    accel.getEvent(&event);
    Serial.print(F("ACCEL "));
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    mag.getEvent(&event);
    Serial.print(F("MAG   "));
    Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

    /* Display the results (gyrocope values in rad/s) */
    gyro.getEvent(&event);
    Serial.print(F("GYRO  "));
    Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");

    Serial.println(F(""));

    uint16_t r, g, b, c, colorTemp, lux;

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);

    Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");

    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();

    delay(1000);
}

void displaySensorDetails(void)
{
  sensor_t sensor;

  accel.getSensor(&sensor);
  Serial.println(F("----------- ACCELEROMETER ----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  gyro.getSensor(&sensor);
  Serial.println(F("------------- GYROSCOPE -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" rad/s"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  mag.getSensor(&sensor);
  Serial.println(F("----------- MAGNETOMETER -----------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(F(" uT"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  delay(100);
}
