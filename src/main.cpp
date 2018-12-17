
// #define DEBUG 1


#include <MKRWAN.h>
#include <RTCZero.h>
#include <CayenneLPP.h>
#include <Wire.h>

// Sensor Libraries
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"
#include <ArduinoECCX08.h>

#include <ArduinoLowPower.h>

// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool ecc508_found= false;
bool voltage_found= true;

unsigned static int sleeptime = 60;

CayenneLPP lpp(51);
String rcvBuffer;

// LoRa Definitions
LoRaModem modem;
String appEui = "70B3D57ED001308A";
String appKey = "584E76FEA1282A36D93D526FF86E8FF3";

//
// General helper functions
//
RTCZero rtc;
bool rtc_init_done = false;
bool rtc_alarm_raised = false;

void rtcAlarm() {
  rtc_alarm_raised = true;
}

void initMyRtc() {
  rtc.begin();
  rtc.setEpoch(0);
  rtc.attachInterrupt(rtcAlarm);
  rtc_init_done = true;
}

void sleepfor(int seconds) {
  digitalWrite(LED_BUILTIN, LOW);
#if DEBUG
    delay(seconds*1000);
#else
    LowPower.sleep(seconds * 1000);
#endif
  digitalWrite(LED_BUILTIN, HIGH);
}


//
// Scan for sensors
//
void setupI2C() {
  byte error, address;
  int nDevices;

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)

#if DEBUG
  Serial.println("Scanning i2c bus");
#endif
  Wire.begin();
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
#if DEBUG
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
#endif

      if (address == 0x39) {
        tsl2561 = Adafruit_TSL2561_Unified(address);
        tsl2561_found = tsl2561.begin();
#if DEBUG
        Serial.print("TSL2561 found? ");
        Serial.println(tsl2561_found);
#endif
        if (tsl2561_found) {
          // init the sensor
          tsl2561.enableAutoRange(true);
          tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);
        }
      }
      if (address == 0x40) {
        // SI7021
        si7021 = Adafruit_Si7021();
        si7021_found = si7021.begin();
#if DEBUG
        Serial.print("Si7021 found? ");
        Serial.println(si7021_found);
#endif
      }

      if (address == 0x60) {
        // ECC508
        ecc508_found = ECCX08.begin();
#if DEBUG
        Serial.print("ECC508 found? ");
        Serial.println(ecc508_found);
#endif
      }

      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
#if DEBUG
        Serial.print("BME280 found? ");
        Serial.println(bme280_found);
#endif
      }
    }
  }
}

void setupLora() {
  modem.begin(EU868);
#if DEBUG
  Serial.print("Your module version is: ");
  Serial.println(modem.version());
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
#endif
  int connected = 0;
  while (!connected) {
    connected = modem.joinOTAA(appEui,appKey);
    if (!connected) {
      sleepfor(sleeptime);
      modem.begin(EU868);
    }
  }
  modem.minPollInterval(60);
}


void setup() {

#if DEBUG
  Serial.begin(115200);
  while (!Serial);
  Serial.println("System starting");
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);

  setupI2C();
  setupLora();
  rcvBuffer.reserve(64);

  analogReadResolution(10);
  analogReference(AR_INTERNAL1V0); //AR_DEFAULT: the default analog reference of 3.3V // AR_INTERNAL1V0: a built-in 1.0V reference
  //analogReference(AR_EXTERNAL);
  //
}

void read_tsl2561() {
  sensors_event_t event;
  tsl2561.getEvent(&event);
  lpp.addLuminosity(4,event.light);
}

void read_si7021() {
  lpp.addTemperature(1, si7021.readTemperature());
  lpp.addRelativeHumidity(2, si7021.readHumidity());
}

void read_bme280() {
  lpp.addTemperature(1,bme280.readTemperature());
  lpp.addRelativeHumidity(2,bme280.readHumidity());
  lpp.addBarometricPressure(3,bme280.readPressure() / 100.0F);
}

float my_voltage() {

  // read the input on analog pin 0:

  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  return sensorValue * (3.25 / 1023.0);
}

void read_voltage() {
  float v = my_voltage();

  if (v == 0 || v > 2.7) {
    sleeptime = 60;
  } else {
    sleeptime = 120;
  }

  if (v > 0.0) {
    lpp.addAnalogInput(5,my_voltage());
  }
}

void readSensors() {
  lpp.reset();
  if (si7021_found) {
    read_si7021();
  }
  if (bme280_found) {
    read_bme280();
  }
  if (tsl2561_found) {
    read_tsl2561();
  }
  if (voltage_found) {
    read_voltage();
  }
}

void sendBuffer() {
  if (lpp.getSize() > 0) {
    int err;
    modem.beginPacket();
    modem.write(lpp.getBuffer(), lpp.getSize());
    err = modem.endPacket(false);
#if DEBUG
    if (err > 0)
      Serial.println("Packet sent");
    else
      Serial.println("Error sending packet");
#else
    if (err == 0) {
      // re-join
      setupLora();
    }
#endif
  }
}

String receiveData(String rcv) {
  int count = 0;
  while (modem.available()) {
    rcv += (char)modem.read();
    count++;
  }
#if DEBUG
  if (count > 0) {
    Serial.print(count);
    Serial.println(" Bytes received");
  }
#endif
  return rcv;
}

void loop() {
  // put your main code here, to run repeatedly:
  readSensors();
  sendBuffer();

  rcvBuffer = "";
  receiveData(rcvBuffer);

#if DEBUG
  if (rcvBuffer.length()) {
    Serial.print("Received: " + rcvBuffer + " - ");
    for (unsigned int i = 0; i < rcvBuffer.length(); i++) {
      Serial.print(rcvBuffer[i] >> 4, HEX);
      Serial.print(rcvBuffer[i] & 0xF, HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
#endif

  sleepfor(sleeptime);
}
