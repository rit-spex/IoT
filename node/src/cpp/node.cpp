#include <./headers/node.h>

#define SerialDebug true
#define UPDATE_DELAY 10
#define NODE_UUID "ImagineSensor1"

String getDataString();
String getUUIDString();
String getIMUString();
String getBMEString();
String getGPSString();
String getColorString();
String outgoingMsg(String msg);
String keyString(String key);
String valueString(String val);
String kvString(String key, String val);

uint16_t getUUID();

using namespace std;

Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

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
  String bmeBuffer = outgoingMsg(getBMEString());
  String imuBuffer = outgoingMsg(getIMUString());
  String gpsBuffer = outgoingMsg(getGPSString());
  String colBuffer = outgoingMsg(getColorString());

if (SerialDebug) {
  Serial.println(bmeBuffer);
  Serial.println(imuBuffer);
  Serial.println(gpsBuffer);
  Serial.println(colBuffer);
}
  LoRa.send((uint8_t*) bmeBuffer.c_str(), bmeBuffer.length());
  LoRa.send((uint8_t*) imuBuffer.c_str(), imuBuffer.length());
  // LoRa.send((uint8_t*) gpsBuffer.c_str(), gpsBuffer.length());
  LoRa.send((uint8_t*) colBuffer.c_str(), colBuffer.length());
  delay(UPDATE_DELAY);
}

String outgoingMsg(String msg)
{
    String buffer = "\{" + getUUIDString();
    buffer += msg;
    buffer += "}";

    return buffer;
}

String kvString(String key, String val){
    return keyString(key) + valueString(val);
}

String keyString(String key){
    return String("\"" + key + "\":");
}

String valueString(String val){
    return String(String(val) + ",");
}

String getColorString()
{
    String buffer;
    uint16_t r, g, b, c, colorTemp, lux;
    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);

    buffer += "\"hasColor\": true, ";

    buffer += kvString("temp", String(colorTemp));
    buffer += kvString("lux", String(lux));
    buffer += kvString("r", String(r));
    buffer += kvString("g", String(g));
    buffer += kvString("b", String(b));
    buffer += kvString("c", String(c));

    return buffer;
}

String getGPSString(){
    String buffer;

    buffer += "\"hasGPS\": true, ";

    buffer += kvString("lat", String(GPS.lat));
    buffer += kvString("lon", String(GPS.lon));
    buffer += kvString("speed", String(GPS.speed));
    buffer += kvString("angle", String(GPS.angle));
    buffer += kvString("alt", String(GPS.altitude));
    buffer += kvString("sats", String(GPS.satellites));
    buffer += kvString("latitude", String(GPS.latitude));
    buffer += kvString("longitude", String(GPS.longitude));

    buffer += "},";

    return buffer;
}


String getUUIDString() {
  String buffer;
  buffer += "\"UUID\": \"";
  buffer += String(NODE_UUID);
  buffer += "\", " ;
  return buffer;
}

String getBMEString(){
    String buffer;
    buffer += "\"hasBarometer\": true, ";

    buffer += "\"tempc\":"; buffer += String(bme.readTemperature()); buffer += ",";
    buffer += "\"pres\":"; buffer += String(bme.readPressure()); buffer += ",";
    buffer += "\"alt\":"; buffer += String(bme.readAltitude(SEALEVELPRESSURE_HPA)); buffer += ",";
    buffer += "\"hum\":"; buffer += String(bme.readHumidity()); buffer += ",";

    return buffer;
}

String getIMUString(){
  String buffer;
  buffer += "\"hasIMU\": true, ";

  sensors_event_t event;
  accel.getEvent(&event);
  gyro.getEvent(&event);
  mag.getEvent(&event);

  buffer += "\"accelX\":"; buffer += String(event.acceleration.x); buffer += ",";
  buffer += "\"accelY\":"; buffer += String(event.acceleration.y); buffer += ",";
  buffer += "\"accelZ\":"; buffer += String(event.acceleration.z); buffer += ",";

  buffer += "\"magX\""; buffer += String(event.magnetic.x); buffer += ",";
  buffer += "\"magY\""; buffer += String(event.magnetic.y); buffer += ",";
  buffer += "\"magZ\""; buffer += String(event.magnetic.z); buffer += ",";

  buffer += "\"gyroX\""; buffer += String(event.gyro.z); buffer += ",";
  buffer += "\"gyroY\""; buffer += String(event.gyro.z); buffer += ",";
  buffer += "\"gyroZ\""; buffer += String(event.gyro.z); buffer += ",";

  return buffer;
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
  GPS.begin(115200);
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
  LoRa.setModemConfig(RH_RF95::ModemConfigChoice::Bw500Cr45Sf128);
  LoRa.setTxPower(23, false);
}
