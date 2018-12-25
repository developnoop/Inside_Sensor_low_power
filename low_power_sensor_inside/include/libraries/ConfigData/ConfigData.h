#ifndef ConfigData_h
#define ConfigData_h

// Here i comment out were the sensor will send its data from, this affects the sended RF values and in this version also the used sensors!
//#define Sensor_Bath // Config Code for Sensor Bath?
//#define Sensor_Balcony // Config Code for Sensor Balcony?
//#define Sensor_MasterBed // Config Code for Sensor MasterBedroom?
#define Sensor_Pond // Config Code for Sensor Pond?

// Which type of sensor do we want to use? This will be set now through the indirect through the location
#define DS18B20_use     0 // 1 = used, 0 = unused; if sensor DS18B20 is used (only temperature values!!)
#define DHT22_use       0 // 1 = used, 0 = unused; if sensor DHT22 is used (temperature and humidity)

/*These values define the RF code value sent if the sensor values are
equals to 0, for example, if the sensor value of temperature is 24°C, the
program is going to send 33240, this resulting value can be interpreted
either at gateway level or better at domotic software level (example openhab)*/
// MR: As the values can reach 4 numbers (100.0 for 100% humidity shift to 4 Numbers!
#define MIN_ERRORCODE  "999900"
#ifdef Sensor_Bath
#define DHT22_use       1
// Bath Sensor Values
#define HUM   "110000" // DHT 22 measures from 0.0 to 100.0
#define TEMP  "130400" // the DHT 22 Sensor give temp from -40.0 to 80.0
#define VOLT  "150000"
// changed to sensor specific ERRORCODE like 9999XY where x stands for the corresponding board 0-9 and Y stands for the sensor on the board from 0-9
// in this way the board itself could send info if only the temperature sensor is not working or if a pressure sensor is not working and so on.
#define ERRORCODE  "999901"  // First Board (0Y) and First Sensor (X1)
#endif

#ifdef Sensor_Balcony
#define DHT22_use       1
// Balcony Sensor Values
#define HUM   "210000" // DHT 22 measures from 0.0 to 100.0
#define TEMP  "230400" // the DHT 22 Sensor give temp from -40.0 to 80.0
#define VOLT  "250000"
// changed to sensor specific ERRORCODE like 9999XY where x stands for the corresponding board 0-9 and Y stands for the sensor on the board from 0-9
// in this way the board itself could send info if only the temperature sensor is not working or if a pressure sensor is not working and so on.
// Balcony Error Code
#define ERRORCODE  "999911"  // Second Board (1Y) and First Sensor (X1)
#endif

#ifdef Sensor_MasterBed
#define DHT22_use       1
// Master Bedroom Sensor Values
#define HUM   "310000" // DHT 22 measures from 0.0 to 100.0
#define TEMP  "330400" // the DHT 22 Sensor give temp from -40.0 to 80.0
#define VOLT  "350000"
// changed to sensor specific ERRORCODE like 9999XY where x stands for the corresponding board 0-9 and Y stands for the sensor on the board from 0-9
// in this way the board itself could send info if only the temperature sensor is not working or if a pressure sensor is not working and so on.
#define ERRORCODE  "999921"  // Third Board (2Y) and First Sensor (X1)
#endif

#ifdef Sensor_Pond
#define DS18B20_use     1
// Pond Sensor Values
#define TEMP2 "410550" // the DS18B20 Sensor give temp from -55.0 to 125.0
#define TEMP  "430550" // the DS18B20 Sensor give temp from -55.0 to 125.0
#define VOLT  "450000"
// changed to sensor specific ERRORCODE like 9999XY where x stands for the corresponding board 0-9 and Y stands for the sensor on the board from 0-9
// in this way the board itself could send info if only the temperature sensor is not working or if a pressure sensor is not working and so on.
#define ERRORCODE  "999931"  // Fourth Board (3Y) and First Sensor (X1)
#define ERRORCODE2  "999932"  // Fourth Board (3Y) and Second Sensor (X2)
#endif

#if (DS18B20_use == 0) && (DHT22_use == 0)   // no DS18B20 and no DHT22
 #error At least one Sensor needs to be used! Check define of sensor location!
#endif

#if DHT22_use == 1   // DHT22 sensor used in project
#include <dhtnew.h>
#endif

#if DS18B20_use == 1   // DS18B20 used in project
#include <OneWire.h>
#include <DallasTemperature.h>
#endif

#if DHT22_use == 1
#define DHTTYPE DHT22 // which of the DHT sensors do we use= 11 or 22?
#endif

// Define the used pins for the sensors, these are Arduino pin numbers which are != ATMEGA328P pin numbers!
#if DHT22_use == 1
const int SensorPin = 3;
const int SensorPowerPin = 4;
#elif DS18B20_use == 1
const int SensorPin = 3;
const int SensorPowerPin = 4;
#endif

// define and declare different variables for the usage of the DS18B20
#if DS18B20_use == 1
#define TEMPERATURE_PRECISION 12
int numberOfDevices; // Number of temperature devices found (onewire aka ds18b20)
DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address
// declare the addresses of the expected sensors
DeviceAddress DEVICE_0 = {0x28, 0x07, 0x1C, 0x43, 0x98, 0x0B, 0x00, 0x80};
DeviceAddress DEVICE_1 = {0x28, 0xFF, 0x04, 0x0A, 0xC1, 0x17, 0x01, 0x68};
//DeviceAddress DEVICE_2 = {0x28, 0x07, 0x00, 0x07, 0x55, 0xBB, 0x01, 0x2C}; // this one is only for testing, not for productive!!!!
#endif

// I limit the writing to the cells to 30k writes, ATMEL says 100k is okay, but better be safe then sorry!
// Lifetime for the EEPROM with a 30k write cycle would be roughly: 1023 bytes divided by 12 bytes per block = 85 blocks
// 10 per hour x 24h x 356 days = ~86k writes a year, so roughly 3 (86k / 30k) blocks a year which gives us 28 years of lifetime (85 / 3).
#define MAXEEPROMWRITE 30000

//Pin on which the sensors are connected; Arduino Pin number, not ATMEGA328 Pin number!!
const int LedPin = 9;

//Pin on which the RF is connected
const int EmitPin = 6;
const int EmitPowerPin = 7;

const int TimeToSleep = 600; // set time to sleep (approx) in seconds, between 10 and 13 minutes, depending on temperature of the chip
const int TimeToSleepError = 60; // short error time to sleep, around 1 minute

#endif
