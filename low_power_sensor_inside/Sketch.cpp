/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

/*
Low power node  - ATMega328p program to send humidity, temperature and battery voltage

This program enables to send sensor data with low power:
- send sensor data to a 433Mhz gateway
- with 3 AA batteries measured at 5V the current consumption in sleep mode is around 5uA
Adaption to ATMEGA Studio 7.0 and changes for reliability: Marc Ruppert

Contributors:
- 1technophile
Based on the libraries:
- RCSwitch
- LowPower
Based on the work of:
Nick Gammon : http://www.gammon.com.au/power
Rocketscream: https://github.com/rocketscream/Low-Power
Tinkerit: https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
Documentation:
Project home: https://github.com/1technophile/low_power_sensor
Blog: http://1technophile.blogspot.com/2016/12/low-cost-low-power-room-sensor.html
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

//#define DHT_ORG // Original Arduino library with small adaption from my side
#define DHT_NEW // library from Rob.Tillaart
//#define DHT_MYS // library from Mysensor with adaptions

#include <dhtnew.h>
#include "LowPower.h"
#include <RCSwitch.h>
#include <string.h>
#include <avr/eeprom.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void sleepSeconds(int seconds);
void TempAndHum();
void sendData(long dataTosend, long dataType);
long vccVoltage();
void trc(String msg);
//End of Auto generated function prototypes by Atmel Studio
void readEEData();


struct Data { // Sizeof should be 12 Bytes
	uint16_t writecounter;		// used for counting eeprom writes, to limit the writing to the same cell (max 100k!)
	float ee_temperature;		// used to store the temperature in eeprom
	float ee_humidity;			// used to store the humidity in eeprom
	uint16_t tempdrop_counter;	// used to store the registered temperatures drops of more then 10 deg!
};

// I limit the writing to the cells to 30k writes, ATMEL says 100k is okay, but better be safe then sorry!
// Lifetime for the EEPROM with a 30k write cycle would be roughly: 1023 bytes divided by 12 bytes per block = 85 blocks
// 10 per hour x 24h x 356 days = ~86k writes a year, so roughly 3 (86k / 30k) blocks a year which gives us 28 years of lifetime (85 / 3).
#define MAXEEPROMWRITE 30000 
#define DHTTYPE DHT22

// Here i comment out were the sensor will send its data from, this affects the sended RF values
#define Sensor_Bath // Config Code for Sensor Bath?
//#define Sensor_Balcony // Config Code for Sensor Balcony?
//#define Sensor_MasterBed // Config Code for Sensor MasterBedroom?

RCSwitch mySwitch = RCSwitch();

//Pin on which the sensors are connected
const int LedPin = 9;
const int DhtPin = 3;
const int DhtPowerPin = 4;
//Pin on which the RF is connected
const int EmitPin = 6;
const int EmitPowerPin = 7;

const int TimeToSleep = 600; // set time to sleep (approx) in seconds, between 10 and 13 minutes, depending on temperature of the chip
const int TimeToSleepError = 60; // short error time to sleep, around 1 minute

// SleepTimer: Time to deep sleep, adapted to error situation:
// No error during measurement: Sleep for TimeToSleep
// Error during measurement: Sleep for TimeToSleepError!
int SleepTimer;

DHTNEW dht(DhtPin);

// define humidity variable to hold the final value
float humidity = NAN; // Set the default value to non valid values.
float temperature = NAN; // Set the default value to non valid values.
uint8_t temp_short_sleep = 0; // used to indicate that a difference of 10 degrees was measured and a short sleep is advised, once!
bool fresh_eeprom = true; // indicates a fresh flashed chip with empty eeprom, address should start at 1 if true
uint8_t ee_address = 1; // default start address, has to be adapted during runtime.
Data ee_data;
uint8_t ee_data_size;


//Do we want to see trace for debugging purposes
#define TRACE 0  // 0= trace off 1 = trace on

/*These values define the RF code value sent if the sensor values are
equals to 0, for example, if the sensor value of temperature is 24Â°C, the
program is going to send 33240, this resulting value can be interpreted
either at gateway level or better at domotic software level (example openhab)*/
// MR: As the values can reach 4 numbers (100.0 for 100% humidity shift to 4 Numbers!
#ifdef Sensor_Bath
// Bath Sensor Values
#define HUM   "110000" // DHT 22 measures from 0.0 to 100.0
#define TEMP  "130400" // the DHT 22 Sensor give temp from -40.0 to 80.0
#define VOLT  "150000"
// changed to sensor specific ERRORCODE like 9999XY where x stands for the corresponding board 0-9 and Y stands for the sensor on the board from 0-9
// in this way the board itself could send info if only the temperature sensor is not working or if a pressure sensor is not working and so on.
#define ERRORCODE  "999901"  // First Board (0Y) and First Sensor (X1)
#endif
#ifdef Sensor_Balcony
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
// Master Bedroom Sensor Values
#define HUM   "310000" // DHT 22 measures from 0.0 to 100.0
#define TEMP  "330400" // the DHT 22 Sensor give temp from -40.0 to 80.0
#define VOLT  "350000"
// changed to sensor specific ERRORCODE like 9999XY where x stands for the corresponding board 0-9 and Y stands for the sensor on the board from 0-9
// in this way the board itself could send info if only the temperature sensor is not working or if a pressure sensor is not working and so on.
#define ERRORCODE  "999921"  // Third Board (2Y) and First Sensor (X1)
#endif



void setup()
{
	
	// WRITE ALL PINS AS INPUT LOW UNTIL NEEDED - DEFAULT STARTING STATE - ALLOWS LOW POWER SLEEP - PIN STATES CHANGED LOCALLY AS REQUIRED - MUST WRITE BACK TO INPUT LOW BEFORE SLEEP!
	for (byte i = 0; i <= A5; i++){
		pinMode (i, INPUT);    // changed as per below
		digitalWrite (i, LOW);  //     ditto
	}
	if (TRACE) {
		Serial.begin(9600);
	}
	// initialize the input for presence detection
	pinMode(DhtPowerPin,INPUT);
	pinMode(EmitPowerPin,INPUT);
	// start led signal
	pinMode(LedPin,OUTPUT);
	digitalWrite(LedPin, HIGH);
	delay(200);
	digitalWrite(LedPin, LOW);
	pinMode(LedPin,INPUT);
	
	SleepTimer = TimeToSleep; // Setup for long sleep, always hope the best!
	// Launch traces for debugging purposes
	trc("Start of the program");
	ee_data_size = sizeof (ee_data);

}

void loop()
{
	// begin emitting
	
	pinMode(EmitPowerPin,OUTPUT);
	digitalWrite(EmitPowerPin, HIGH);
	mySwitch.enableTransmit(EmitPin);  // Using Pin #6
	mySwitch.setRepeatTransmit(15); //increase transmit repeat to avoid lost of rf sending

	// send battery voltage
	trc("Voltage: ");
	trc(String(vccVoltage()));
	sendData(vccVoltage(), atol(VOLT));

	// read eeprom values
	readEEData();

	// send temp and hum
	pinMode(DhtPowerPin,OUTPUT);
	digitalWrite(DhtPowerPin, HIGH);
	
	//dht.begin();
	// no begin in this libary used anymore
	
	delay(100); // added to give the DHT more time to startup, did not fix spontaneous temperature drops!
	TempAndHum();
	digitalWrite(DhtPowerPin, LOW);
	pinMode(DhtPin,INPUT);//disable the internal pull up resistor enable by dht.begin
	digitalWrite(DhtPin, LOW); // MR for getting rid of the last 13mA
	
	pinMode(DhtPowerPin,INPUT);
	
	// send battery voltage, move for test reasons, power sourge during sending?
//	trc("Voltage: ");
//	trc(String(vccVoltage()));
//	sendData(vccVoltage(), atol(VOLT));
	
	//deactivate the transmitter
	mySwitch.disableTransmit();
	pinMode(EmitPowerPin,INPUT);
	digitalWrite(EmitPowerPin,LOW); // mr
	pinMode(EmitPin,INPUT); //mr
	digitalWrite(EmitPin,LOW); // mr
	

	// sleep for x seconds
	trc("Sleep");
	sleepSeconds(SleepTimer);
	//sleepSeconds(TimeToSleep);

}

void sleepSeconds(int seconds)
{
	for (int i = 0; i < (seconds/8); i++) { // changed SLEEP_1S to SLEEP_8S, so seconds have to be divided by 8 to match
		LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
	}
}


void checkForFreshEEprom(){ // reads the first Byte Address of the EEPROM and checks if the value is 255, the inital value, or 0
	uint8_t ee_value;
	ee_value = eeprom_read_byte((uint8_t*) 0);
	if ((0 == ee_value) || (255 == ee_value)) { // zero or 255 detected, eeprom seems to be empty
		ee_address = 1; // if eeprom is empty start after struct counter
		fresh_eeprom = true;
		eeprom_write_byte((uint8_t*) 0, 1);
		} else { // value between 1 and 254 detected, not empty
		ee_address = (ee_data_size * ee_value)-(ee_data_size-1); //
		if ((ee_address + ee_data_size) > E2END){ // calculated address bigger then eeprom, should not happen, but in case start from adress 1
			ee_address = 1;
		}
		fresh_eeprom = false;
	}
}

void readEEData(){
	Data localtmp;
	checkForFreshEEprom(); // always test for empty eeprom and recalculate read and write address!
	//ee_data.writecounter = eeprom_read_word((uint16_t*) ee_address);
	//ee_data.ee_temperature = eeprom_read_float((float*) (ee_address+2));
	//ee_data.ee_humidity = eeprom_read_float((float*) (ee_address+6))
	eeprom_read_block((void*)&localtmp, (const void*)ee_address, sizeof(localtmp));
	if (fresh_eeprom) {
		localtmp.writecounter=0;
		localtmp.tempdrop_counter=0;
		localtmp.ee_temperature=NAN;
		localtmp.ee_humidity=NAN;
	}
	ee_data = localtmp;
}

void writeEEData(boolean add_temp_drop){
	unsigned int ee_value;
	Data localtmp;
	
	if (add_temp_drop) { // temperature drop seen, so increase the value!
		ee_data.tempdrop_counter++;
	}
	ee_data.writecounter++;
	
	eeprom_update_block((const void*)&ee_data, (void*)ee_address, ee_data_size);
	
	if(ee_data.writecounter >= MAXEEPROMWRITE) { // the eeprom cell where the data is written to was written MAXEEPROMWRITE, use the next address block
		ee_value = eeprom_read_byte((uint8_t*) 0);
		ee_value++;
		eeprom_update_byte((uint8_t*)0, ee_value);
		ee_address = (ee_data_size * ee_value)-(ee_data_size-1); // calculate the new address
		if ((ee_address + ee_data_size) > E2END){ // calculated address bigger then eeprom, should not happen, but in case start from EEPROM address 1
			ee_address = 1;
		} else { // now the new block has to be nulled, because probably only FFs are present
			localtmp.writecounter=0;
			localtmp.tempdrop_counter=0;
			localtmp.ee_temperature=NAN;
			localtmp.ee_humidity=NAN;
			eeprom_write_block((const void*)&localtmp, (void*)ee_address, ee_data_size);			
		}
	}
	
}
void measureTempAndHum(){
	// The original TempAndHum function was split two allow a better error handling
	// This function now only measures and the handling of the values is done outside of this function.
	delay(500);
	int loop = 0;
	int chk;
	while (loop < 5) {
		//retrieving value of temperature and humidity from DHT
		//humidity = dht.readHumidity();
		//temperature = dht.readTemperature();
		chk = dht.read();
		switch (chk)
		{
			case DHTLIB_OK:
			humidity = dht.humidity;
			temperature = dht.temperature;
			break;
			case DHTLIB_ERROR_CHECKSUM:
			loop++; // Error in checksum, read again
			break;
			case DHTLIB_ERROR_TIMEOUT:
			loop++; // Timeout Error, read again
			break;
			default:
			loop++; // Some unknown error, read again
			break;
		}
		if (isnan(humidity) || isnan(temperature)) { // not a number read, so do another turn!
			loop++;
			} else {
			break; // at least one correct value read, so exist while
		}
		delay(2100);
	}
}

void TempAndHum(){
	volatile int dropcheck_temp; // volatile only needed for debug reasons! without, the compiler optimized it away and the debugger can't read the fxxx value
	volatile int dropcheck_hum;
	//retrieving value of temperature and humidity from DHT
	measureTempAndHum();
	if (isnan(humidity) || isnan(temperature)) {
		trc("Failed to read from DHT sensor!");
		if (temp_short_sleep > 0) { // only send error message after two erroneous measurements aka NAN or TempDrop seen! 
			sendData(atol(ERRORCODE), atol(HUM));//send error code, as the same error code is used for both, only send it once
			temp_short_sleep = 0;
		}
		else {
			temp_short_sleep++;
		}
		//sendData(atol(ERRORCODE), atol(TEMP));//send error code
		SleepTimer = TimeToSleepError; // Set sleep time for short sleep
	} else {
		if(isnan(ee_data.ee_humidity) || isnan(ee_data.ee_temperature)){
			sendData(int(humidity*10), atol(HUM));
			sendData(int(temperature*10), atol(TEMP));
			SleepTimer = TimeToSleep; // everything ok, so send to long sleep!
			temp_short_sleep = 0;
			// prepare data for writing
			ee_data.ee_humidity = humidity;
			ee_data.ee_temperature = temperature;
			// write to eeprom
			writeEEData(false);
		} else { // old values available from last measure check against huge difference in temp!
			// Could also use fabsf(float1-float2) instead of abs (int(float1 - float2))
			dropcheck_temp = ((int)ee_data.ee_temperature) - ((int)temperature);
			dropcheck_hum = ((int)ee_data.ee_humidity) - ((int)humidity);
			if((dropcheck_temp > 10) and (dropcheck_hum > 10)) { // absolute difference between two measurement greater then 10 degrees? Better check again! 
			//if((abs((int)(ee_data.ee_temperature-temperature)) > 10) and (abs((int)(ee_data.ee_humidity-humidity))> 10)) { 
				if (temp_short_sleep < 1) { // not yet a short sleep timer set (two times we wait for correct values!)
					temp_short_sleep++;
					writeEEData(true); // Write to EEPROM that we saw a temperature drop
					SleepTimer = TimeToSleepError; // Set sleep time for double short sleep (roughly 2 minutes!)
				}
				else { // short sleep was ordered already, but didn't change the measurement, so we think a temperature drop has really happened.
					temp_short_sleep = 0;
					ee_data.ee_humidity = humidity;
					ee_data.ee_temperature = temperature;
					// write to eeprom
					writeEEData(true);
					//now send data
					sendData(int(humidity*10), atol(HUM));
					sendData(int(temperature*10), atol(TEMP));
					SleepTimer = TimeToSleep; // everything ok, so send to long sleep!
				}

			} else { // saw a difference smaller 10 deg, save humidity and temperature for later reference and send data!
				temp_short_sleep = 0;
				ee_data.ee_humidity = humidity;
				ee_data.ee_temperature = temperature;
				writeEEData(false);
				sendData(int(humidity*10), atol(HUM));
				sendData(int(temperature*10), atol(TEMP));
				SleepTimer = TimeToSleep; // everything ok, so send to long sleep!
			}
		}
	}
}


void sendData(long dataTosend, long dataType){
	long sum = atol(ERRORCODE);

	trc("DataToSend");
	trc(String(dataTosend));
	trc("DataType");
	trc(String(dataType));
	
	if (dataTosend == sum) {
		// nothing to do sending error code
		} else {
		sum = dataTosend + dataType; // sending value added to topic offset
	}
	
	trc("Sum");
	trc(String(sum));
	
	//sending value by RF
	mySwitch.send(sum,24);
	
}

// https://code.google.com/archive/p/tinkerit/wikis/SecretVoltmeter.wiki
// https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long vccVoltage() {
	long result = 0;
	// Read 1.1V reference against AVcc
	#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
	ADMUX = _BV(MUX5) | _BV(MUX0);
	#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
	ADMUX = _BV(MUX3) | _BV(MUX2);
	#else
	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	#endif
	//ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
	delay(10); // Wait for Vref to settle
	ADCSRA |= _BV(ADSC); // Convert
	while (bit_is_set(ADCSRA,ADSC));
	result = ADCL;
	result |= ADCH<<8;
	result = 1126400L / result; // Back-calculate AVcc in mV
	//result = 1125300L / result; // Back-calculate AVcc in mV // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
	return result;
}


//trace function
void trc(String msg){
	if (TRACE) {
		Serial.println(msg);
	}
}

