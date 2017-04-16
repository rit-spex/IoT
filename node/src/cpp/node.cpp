#include <./headers/node.h>
#define SerialDebug true

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

    prettyPrintAccelerometerData(accel, event);
    prettyPrintMagnetometerData(mag, event);
    prettyPrintGyroscopeData(gyro, event);
    prettyPrintColorSensorData(tcs);
    prettyPrintBarometerData(bme);
    prettyPrintGPSData(GPS);
    Serial.println();
  }

  sendDataViaLoRa(LoRa);
  delay(1000);
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

char* getGPSData(Adafruit_GPS& GPS)
{
  char str[80];
   strcpy (str,"these ");
   strcat (str,"strings ");
   strcat (str,"are ");
   strcat (str,"concatenated.");

   return str;

/*
   strcpy (str,"Time: ");
   strcpy (str,"these ");(GPS.hour, DEC); Serial.print(':');
   strcpy (str,"these ");(GPS.minute, DEC); Serial.print(':');
   strcpy (str,"these ");(GPS.seconds, DEC); Serial.print('.');
   Serial.println(GPS.milliseconds);
   strcpy (str,"these ");("Date: ");
   strcpy (str,"these ");(GPS.day, DEC); Serial.print('/');
   strcpy (str,"these ");(GPS.month, DEC); Serial.print("/20");
   strcpy (str,"these ");(GPS.year, DEC);
   strcpy (str,"these ");("Fix: "); Serial.print((int)GPS.fix);
   strcpy (str,"these ");(" quality: "); Serial.println((int)GPS.fixquality);
   */
}

void prettyPrintAccelerometerData(Adafruit_LSM303_Accel_Unified& accel, sensors_event_t& event)
{
  accel.getEvent(&event);
  Serial.print(F("ACCEL "));
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
}

void prettyPrintMagnetometerData(Adafruit_LSM303_Mag_Unified& mag, sensors_event_t& event)
{
  mag.getEvent(&event);
  Serial.print(F("MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
}
void prettyPrintGyroscopeData(Adafruit_L3GD20_Unified& gyro, sensors_event_t& event)
{
  gyro.getEvent(&event);
  Serial.print(F("GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");
}

void prettyPrintColorSensorData(Adafruit_TCS34725& tcs)
{
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
}

void prettyPrintBarometerData(Adafruit_BME280& bme)
{
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
}

void prettyPrintGPSData(Adafruit_GPS& GPS)
{
  Serial.print("\nTime: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.println(GPS.milliseconds);
  Serial.print("Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
  Serial.print("Fix: "); Serial.print((int)GPS.fix);
  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
  }
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
