#include <./headers/node.h>
#include <ArduinoJson.h>
#define SerialDebug true

JsonObject& getFullObject(DynamicJsonBuffer);
JsonObject& getGpsJson(DynamicJsonBuffer);
JsonObject& getBMEJson(DynamicJsonBuffer jBuffer);

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

Adafruit_BME280 bme; // I2C

Adafruit_GPS GPS = Adafruit_GPS(&GPSSerial);

RH_RF95 LoRa(RFM95_CS, RFM95_INT);

void setup(void)
{
  if (SerialDebug) setupSerial();
  setupAccelerometer(accel, SerialDebug);
  setupMagnetometer(mag, SerialDebug);
  setupGyroscope(gyro, SerialDebug);
  setupColorSensor(tcs, SerialDebug);
  setupBarometer(bme, SerialDebug);
  setupGPS(GPS, SerialDebug);
  setupLoRa(LoRa);
}

void loop(void)
{
  /*
  sensors_event_t event;
  accel.getEvent(&event);
  mag.getEvent(&event);
  gyro.getEvent(&event);

  uint16_t r, g, b, c, colorTemp, lux;
  tcs.getRawData(&r, &g, &b, &c);
  lux = tcs.calculateLux(r, g, b);

  bme.readTemperature();
  bme.readPressure();
  bme.readAltitude(SEALEVELPRESSURE_HPA);
  bme.readHumidity();

  char gpsString = GPS.read();
  if (GPSECHO && SerialDebug)
  {
    if (gpsString) Serial.print(c);
  }
  if (GPS.newNMEAreceived())
  {
    if (SerialDebug)
    {
      Serial.println(GPS.lastNMEA());
    }
    if (!GPS.parse(GPS.lastNMEA()))
    {
      return;
    }
  }

  if (SerialDebug)
  {

    // prettyPrintAccelerometerData(accel, event);
    // prettyPrintMagnetometerData(mag, event);
    // prettyPrintGyroscopeData(gyro, event);
    // prettyPrintColorSensorData(tcs);
    // prettyPrintBarometerData(bme);
    Serial.println("=====================");
    prettyPrintGPSData(GPS);
    Serial.println();
  }


  sendDataViaLoRa(LoRa);
  */
  DynamicJsonBuffer jsonBuffer;

  getFullObject(jsonBuffer).prettyPrintTo(Serial);
  delay(1000);
}

JsonObject& getFullObject(DynamicJsonBuffer jBuffer){
  JsonObject& root = jBuffer.createObject();

  root.set("GPS", getGpsJson(jBuffer));
  root.set("BME280", getBMEJson(jBuffer));
  return root;
}

JsonObject& getGpsJson(DynamicJsonBuffer jBuffer){
    JsonObject& gpsObject = jBuffer.createObject();

    gpsObject["lat"] = GPS.lat;
    gpsObject["latitude"] = GPS.latitude;
    gpsObject["lon"] = GPS.lon;
    gpsObject["longitude"] = GPS.longitude;
    gpsObject["speed"] = GPS.speed;
    gpsObject["angle"] = GPS.angle;
    gpsObject["alt"] = GPS.altitude;
    gpsObject["sats"] = GPS.satellites;

    return gpsObject;
}

JsonObject& getBMEJson(DynamicJsonBuffer jBuffer){
    JsonObject& bmeObject = jBuffer.createObject();

    bmeObject["tempc"] = bme.readTemperature();
    bmeObject["pres"] = bme.readPressure();
    bmeObject["alt"] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bmeObject["hum"] = bme.readHumidity();

    return bmeObject;
}

void setupSerial(void)
{
  Serial.begin(115200);
  Serial.println(F("RIT SPEX IoT Sensor Node")); Serial.println("");
}

bool setupAccelerometer(Adafruit_LSM303_Accel_Unified& accel, bool debug)
{
  if(!accel.begin())
  {
      delay(5000);
      if (debug)
      {
        Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
        while(1){Serial.println('1');};
      }
      return false;
  }
  return true;
}

bool setupMagnetometer(Adafruit_LSM303_Mag_Unified& mag, bool debug)
{
  if(!mag.begin())
  {
    if (debug)
    {
      Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
      while(1){Serial.println('2');};
    }
      return false;
  }
  return true;
}

bool setupGyroscope(Adafruit_L3GD20_Unified& gyro, bool debug)
{
  if(!gyro.begin())
  {
    if (debug)
    {
      Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
      while(1){Serial.println('3');};
    }
    return false;
  }
  return true;
}
bool setupColorSensor(Adafruit_TCS34725& tcs, bool debug)
{
  if (!tcs.begin())
  {
    if (debug)
    {
      Serial.println("No TCS34725 found ... check your connections");
      while (1){Serial.println('4');};
    }
    return false;
  }
  return true;
}

bool setupBarometer(Adafruit_BME280& bme, bool debug)
{
  if (!bme.begin())
  {
    if (debug)
    {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      while (1){Serial.println('5');};
    }
    return false;
  }
  return true;
}

bool setupGPS(Adafruit_GPS& GPS, bool debug)
{
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

bool setupLoRa(RH_RF95& LoRa)
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!LoRa.init())
  {
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!LoRa.setFrequency(RF95_FREQ))
  {
    while (1);
  }
  LoRa.setTxPower(23, false);
}

void sendDataViaLoRa(RH_RF95& LoRa)
{
  char radioPacket = *getGPSData(GPS);

  delay(10);
  LoRa.send((uint8_t *)radioPacket, sizeof(radioPacket));

  delay(10);
  LoRa.waitPacketSent();
}
