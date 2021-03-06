
// #define DEBUG 1


#include <MKRWAN.h>
#include <RTCZero.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <ArduinoLog.h>
#include <WDTZero.h>

// Sensor Libraries
#include "Adafruit_Si7021.h"
#include "Adafruit_BME280.h"
#include "Adafruit_TSL2561_U.h"
#include "Adafruit_ADS1015.h"
#include <ArduinoECCX08.h>

#include <ArduinoLowPower.h>

// Global Objects
Adafruit_Si7021 si7021;
Adafruit_BME280 bme280;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT);
Adafruit_ADS1115 ads1115;

bool si7021_found = false;
bool bme280_found = false;
bool tsl2561_found= false;
bool ecc508_found= false;
bool voltage_found= true;
bool ads1115_found = false;

bool led_dynamic = true;

unsigned static int sleeptime = 60;
unsigned static int error_counter = 0;

CayenneLPP lpp(51);
#define MAX_BUF_LEN 64
char rcvBuffer[MAX_BUF_LEN];

// LoRa Definitions
LoRaModem modem;
String appEui = "70B3D57ED001308A";
String appKey = "A429A527FE6074F13A1511040BCEBB05";

//
// General helper functions
//
RTCZero rtc;
WDTZero dog;

bool rtc_init_done = false;
bool rtc_alarm_raised = false;

void rtcAlarm() {
  rtc_alarm_raised = true;
}

void setup_RTC() {
  rtc.begin();
  rtc.setEpoch(0);
  rtc.attachInterrupt(rtcAlarm);
  rtc_init_done = true;
}

void setup_serial() {
  Serial.begin(115200);
#if DEBUG
  delay(5*1000);
  while (!Serial);
#endif
}

void sleepfor(int seconds) {
  uint32_t now = rtc.getEpoch();

  Log.verbose(F("entering sleepfor(%d)"),seconds);
#if defined(DEBUG)
  delay(seconds * 1000);
#else
  // rtc.setAlarmEpoch(now + seconds);
  // rtc.enableAlarm(rtc.MATCH_MMSS);
  Serial.end();
  USBDevice.detach();
  if (led_dynamic)
    digitalWrite(LED_BUILTIN, LOW);
  // rtc.standbyMode();    // Sleep until next alarm match
  LowPower.sleep(seconds * 1000);
  if (led_dynamic)
    digitalWrite(LED_BUILTIN, HIGH);
  USBDevice.attach();
  setup_serial();
  delay(1000);
#endif
  Log.verbose(F("leaving sleepfor(%d)"),seconds);
  rtc.disableAlarm();
}


//
// Scan for sensors
//
void setup_I2C() {
  byte error, address;

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

  Log.verbose("Scanning i2c bus");
  Wire.begin();
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Log.verbose(F("I2C device found at address 0x%x !"),address);

      if ((address == 0x39) || (address == 0x29) || (address == 0x49)) {

        tsl2561 = Adafruit_TSL2561_Unified(address);
        tsl2561_found = tsl2561.begin();
        Log.verbose(F("TSL2561 found? %T"),tsl2561_found);
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
        Log.verbose(F("Si7021 found? %T"),si7021_found);
      }
      if ((address >= 0x48) && (address <= 0x4b)) {
        ads1115 = Adafruit_ADS1115(address);
        ads1115.begin();
        ads1115_found = true;
        Log.notice(F("ADS1115 found at 0x%x"),address);
      }
      if (address == 0x60) {
        // ECC508
        ecc508_found = ECCX08.begin();
        Log.verbose(F("ECC508 found? %T"),ecc508_found);
      }

      if (address == 0x76 || address == 0x77) {
        // BME280
        bme280_found = bme280.begin(address);
        Log.verbose(F("BME280 found? %T"),bme280_found);
      }
    }
  }
}

void setup_Lora() {
  modem.begin(EU868);
  Log.notice(F("Your module version is: %s"),modem.version().c_str());
  Log.notice(F("Your device EUI is: %s"),modem.deviceEUI().c_str());
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

// Logging helper routines
void printTimestamp(Print* _logOutput) {
  char c[12];
  sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void printNewline(Print* _logOutput) {
  _logOutput->print('\n');
}

void setup_logging() {
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
  Log.setPrefix(printTimestamp);
  Log.setSuffix(printNewline);
  Log.verbose("Logging has started");
}

void setup() {

  setup_serial();

  pinMode(LED_BUILTIN, OUTPUT);
  if (led_dynamic)
    digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);

  setup_logging();

  setup_RTC();
  setup_I2C();
  setup_Lora();

  analogReadResolution(12);
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

void read_ads1115() {
  for (int i=0;i<4;i++) {
    uint16_t r = ads1115.readADC_SingleEnded(i);
    lpp.addLuminosity(20+i, r);
  }
}

float my_voltage() {

  // read the input on analog pin 0:

  int sensorValue = analogRead(ADC_BATTERY);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  return sensorValue * (3.25 / 4096.0);
}

void read_voltage() {
  float v = my_voltage();

  Log.verbose(F("Voltage is %D"), v);

  if (v > 0.1) {
    sleeptime = (int)(10000.0 / pow(v,4.0));
  } else {
    sleeptime = 90;
  }

  if (sleeptime < 90)
    sleeptime = 90;

  if (v > 0.0) {
    lpp.addAnalogInput(5,my_voltage());
  }
}

void readSensors() {
  lpp.reset();
  setup_I2C();
  if (si7021_found) {
    read_si7021();
  }
  if (bme280_found) {
    read_bme280();
  }
  if (tsl2561_found) {
    read_tsl2561();
  }
  if (ads1115_found) {
    read_ads1115();
  }
  if (voltage_found) {
    read_voltage();
  }
}

void send_32bit(uint32_t x) {
  lpp.addDigitalInput(10,x & 0xff);
  x >>= 8;
  lpp.addDigitalInput(11,x & 0xff);
  x >>= 8;
  lpp.addDigitalInput(12,x & 0xff);
  x >>= 8;
  lpp.addDigitalInput(13,x & 0xff);
}

void sendBuffer() {
  int err;
  modem.beginPacket();
  if (lpp.getSize() > 0)
    modem.write(lpp.getBuffer(), lpp.getSize());

  err = modem.endPacket(false);
  if (err > 0) {
    Log.verbose(F("Packet sent"));
    error_counter = 0;
  }
  else {
    error_counter++;
    Log.error(F("Error sending packet: %d - Error counter: %d"),err,error_counter);
    if (error_counter > 100) {
      // re-join
      modem.restart();
      setup_Lora();
    }
  }
}

unsigned char receiveData(char *rcv) {
  int count = 0;
  while (modem.available() && (count < MAX_BUF_LEN)) {
    rcv[count++] =  (char)modem.read();
  }
  if (count > 0) {
    Log.verbose(F("Bytes recived: %d"),count);
  }
  return count;
}

// -------------- Command Processing -----------------
void process_system_led_command(unsigned char len, char *buffer) {
  if (len == 0)
    return;

  switch (buffer[0]) {
    case 0:
      led_dynamic = false;
      pinMode(LED_BUILTIN,LOW);
      break;
    case 1:
      led_dynamic = false;
      pinMode(LED_BUILTIN, HIGH);
      break;
    case 0xff:
      led_dynamic = true;
      break;
    default:
      Log.error(F("Unknown LED command %d"), buffer[0]);
      break;
  }
}


void process_system_command(unsigned char len,  char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length system command"));
    return;
  }
  switch (buffer[0]) {
    case 0x01:
      dog.setup(WDT_HARDCYCLE16S);
      delay(1000*20);
      break;
    case 0x03:
      process_system_led_command(len-1,buffer+1);
      break;
  }
}

void process_sensor_command(unsigned char len,  char *buffer) {
  if (len == 0) {
    Log.error(F("Zero length sensor command"));
    return;
  }
  switch (buffer[0]) {
    case 0xff:
      setup_I2C();
      break;
  }
}

void process_received_lora(unsigned char len,  char *buffer) {
  if (len == 0)
    return;

  Log.verbose(F("Processing %d bytes of received data"),len);
  switch (buffer[0]) {
    case 0:
      process_system_command(len-1,buffer+1);
      break;
    case 1:
      process_sensor_command(len-1,buffer+1);
      break;
    default:
      Log.error(F("Unknown command %d"),buffer[0]);
      break;
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  readSensors();
  sendBuffer();

  unsigned char rcvLen = receiveData(rcvBuffer);

  if (rcvLen > 0) {
    process_received_lora(rcvLen,rcvBuffer);
  }

  sleepfor(sleeptime);
}
