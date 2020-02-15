#include <Arduino.h>

/*
Low power node  - ATMega328p program to send humidity, temperature and battery voltage

This program enables to send sensor data with low power:
- send sensor data to a 433Mhz gateway
- with 4 AA batteries measured at 5V the current consumption in sleep mode is around 5uA

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
Blog: https://1technophile.blogspot.com/2016/12/low-cost-low-power-room-sensor.html
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

/*
 - adaption to ATMEGA Studio 7.0 and changes for reliability: Marc Ruppert
 - added the possibility to send different RF Values based on "location" of sensor
 - added the possibility to use ds18b20 instead of dht22
*/

// Important: All needed config has to be done in the ConfigData.h! Look there first!
#include <ConfigData.h> // here all Config Switches are located

#include "LowPower.h"
#include <RCSwitch.h>
#include <string.h>
#include <avr/eeprom.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void sleepSeconds(int seconds);
void TempAndHum_DHT22();
void prepare_onewire_data();
void sendData(long dataTosend, long dataType);
void trc(String msg);
//End of Auto generated function prototypes by Atmel Studio
void readEEData();
long vccVoltage();

struct Data { // Sizeof should be 12 Bytes
	uint16_t writecounter;		// used for counting eeprom writes, to limit the writing to the same cell (max 100k!)
	float ee_temperature;		// used to store the temperature in eeprom
	float ee_humidity;			// used to store the humidity in eeprom
	uint16_t tempdrop_counter;	// used to store the registered temperatures drops of more then 10 deg!
};
// create the RF Switch, needed for sending values
RCSwitch mySwitch = RCSwitch();

// SleepTimer: Time to deep sleep, adapted to error situation:
// No error during measurement: Sleep for TimeToSleep
// Error during measurement: Sleep for TimeToSleepError!
int SleepTimer;

#if DHT22_use == 1
DHTNEW dht(SensorPin); // Setup a DHT sensor with data expected on pin SensorPin
#endif

#if DS18B20_use == 1
OneWire oneWire(SensorPin); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.
#endif

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


void setAllPinInputLow()
{
	// WRITE ALL PINS AS INPUT LOW UNTIL NEEDED
	// DEFAULT STARTING STATE 
	// ALLOWS LOW POWER SLEEP 
	// PIN STATES CHANGED LOCALLY AS REQUIRED 
	// Important: MUST WRITE BACK TO INPUT LOW BEFORE SLEEP! 
	for (byte i = 0; i <= A5; i++){
		pinMode (i, INPUT);    // changed as per below
		digitalWrite (i, LOW);  //     ditto
	}
}

void ledOneBlink()
{
	// start led signal
	pinMode(LedPin,OUTPUT);
	digitalWrite(LedPin, HIGH);
	delay(200);
	digitalWrite(LedPin, LOW);
	pinMode(LedPin,INPUT);
}

void pinPowerOn(const int PowerPin)
{
	pinMode(PowerPin,OUTPUT); // set the powerpin of the sensor to output
	digitalWrite(PowerPin, HIGH); // give the powerpin 3.3 V
}

void pinPowerOff(const int PowerPin, const int SensorPin)
{
	pinMode(PowerPin,INPUT);
	digitalWrite(PowerPin, LOW);
	pinMode(SensorPin,INPUT);      //disable the internal pull up resistor enable by dht.begin
	digitalWrite(SensorPin, LOW);  // MR for getting rid of the last 13mA
}

void setup()
{
	setAllPinInputLow();

	if (TRACE) {
		Serial.begin(9600);
	}
	
	#if DHT22_use == 1
	// initialize the input for presence detection
	pinMode(SensorPowerPin,INPUT);
	#endif
	
	#if DS18B20_use == 1
	pinMode(SensorPowerPin,INPUT);
	#endif
	
	// Set the Power of the transmitter to input;Result:  turn the transmitter off!
	pinMode(EmitPowerPin,INPUT);
	
	// Signal one blink with the led
	ledOneBlink();
	
	SleepTimer = TimeToSleep; // Setup for long sleep, always hope the best!
	// Launch traces for debugging purposes
	trc("Start of the program");
	
	// calculate sizeof one EEPROM Date Unit!
	ee_data_size = sizeof (ee_data);

}

#if DHT22_use == 1
void loop_dht22() // DHT22 only part of loop
{
	pinPowerOn(SensorPowerPin);
	delay(100); // added to give the DHT more time to startup, did not fix spontaneous temperature drops!
	TempAndHum_DHT22();
	pinPowerOff(SensorPowerPin, SensorPin);
}
#endif

#if DS18B20_use == 1

float initDS18B20()
{
	sensors.begin(); //start up temp sensor
	sensors.requestTemperatures(); // Get the temperature
	return sensors.getTempCByIndex(0);
}

void loop_onewire()  // DS18B20 only part of loop
{
	pinPowerOn(SensorPowerPin);
	delay(750); // 750 ms needed for temperature calculations!

	temperature = initDS18B20();	
	
	// Start up the library
	sensors.begin();
	// Grab a count of devices on the wire
	numberOfDevices = sensors.getDeviceCount();
	// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
	sensors.setResolution(TEMPERATURE_PRECISION);

	sensors.requestTemperatures(); // Send the command to get temperatures
	
	temperature = sensors.getTempC(DEVICE_0);
	humidity = sensors.getTempC(DEVICE_1);
	
#if 0
	// only for testing: Delete later starting from here!
	temperature = sensors.getTempC(DEVICE_2);
	humidity = sensors.getTempC(DEVICE_2);
	// end here
#endif
	
	prepare_onewire_data(); // Use a simple function to copy the data
	
	pinPowerOff(SensorPowerPin, SensorPin);
}
#endif

void loop()
{
	// begin emitting
	pinPowerOn(EmitPowerPin);
	//pinMode(EmitPowerPin,OUTPUT);
	//digitalWrite(EmitPowerPin, HIGH);
	mySwitch.enableTransmit(EmitPin);  // Using Pin #6
	mySwitch.setRepeatTransmit(15); //increase transmit repeat to avoid lost of rf sending

	// send battery voltage
	trc("Voltage: ");
	trc(String(vccVoltage()));
	sendData(vccVoltage(), atol(VOLT));

	// read eeprom values
	readEEData();

	#if DHT22_use == 1
		loop_dht22();
	#endif
	
	#if DS18B20_use == 1
		loop_onewire();
	#endif
	
	//deactivate the transmitter
	mySwitch.disableTransmit();
	pinPowerOff(EmitPowerPin, EmitPin);
//	pinMode(EmitPowerPin,INPUT);
//	digitalWrite(EmitPowerPin,LOW); // mr
//	pinMode(EmitPin,INPUT); //mr
//	digitalWrite(EmitPin,LOW); // mr
	

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

#if DHT22_use == 1
void measureTempAndHum_DHT22(){  // only for DHT22 usage!
	// The original TempAndHum function was split two allow a better error handling
	// This function now only measures and the handling of the values is done outside of this function.
	delay(500);
	int loop = 0;
	int chk;
	while (loop < 5) {
		chk = dht.read();
		switch (chk)
		{
			case DHTLIB_OK:
			humidity = dht.humidity;
			temperature = dht.temperature;
			break;
			case DHTLIB_ERROR_CHECKSUM:
			// Error in checksum, read again
			break;
			case DHTLIB_ERROR_TIMEOUT:
			// Timeout Error, read again
			break;
			default:
			// Some unknown error, read again
			break;
		}
		if (isnan(humidity) || isnan(temperature) || (chk != DHTLIB_OK)) { // not a number read, so do another turn!
			loop++;
			} else {
				if ((humidity > MAXHUMIDITY) || (temperature > MAXTEMPERATURE )) { // invalid value recieved, not catched by internal validation!
					loop++;
				}
				else {
					break; // at least one correct value read, so exist while
				}
		}
		delay(2100);
	}
}

void TempAndHum_DHT22(){ // only for DHT22
	volatile int dropcheck_temp; // volatile only needed for debug reasons! without, the compiler optimized it away and the debugger can't read the fxxx value
	volatile int dropcheck_hum;
	//retrieving value of temperature and humidity from DHT
	measureTempAndHum_DHT22();
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
#endif


#if DS18B20_use == 1
void prepare_onewire_data()
{
	if ((isnan(humidity) || isnan(temperature) || (humidity < -126.0) || (temperature < -126.0))) { // -127 is also a error value of the DS18B20!
		trc("Failed to read from one of the onewire sensor!");
		if (isnan(temperature) || (temperature < -126.0)) {
			sendData(atol(ERRORCODE), atol(TEMP));//send error code for Device_0
		} else {
			sendData(atol(ERRORCODE2), atol(TEMP2));//send error code for Device_1
		}
		SleepTimer = TimeToSleepError; // Set sleep time for short sleep
	} else {
	if(isnan(ee_data.ee_humidity) || isnan(ee_data.ee_temperature)){
		sendData(int(humidity*10), atol(TEMP2));
		sendData(int(temperature*10), atol(TEMP));
		SleepTimer = TimeToSleep; // everything ok, so send to long sleep!

		// prepare data for writing
		ee_data.ee_humidity = humidity;
		ee_data.ee_temperature = temperature;
		// write to eeprom
		writeEEData(false);
		} else { // old values available from last measure, here not checked against last measurement (let's hope the DS18B20 does not has these outtakes as the DHT22
			// save temperature2 and temperature for later reference and send data!
			ee_data.ee_humidity = humidity; // temperature2
			ee_data.ee_temperature = temperature;
			writeEEData(false);
			sendData(int(humidity*10), atol(TEMP2));
			sendData(int(temperature*10), atol(TEMP));
			SleepTimer = TimeToSleep; // everything ok, so send to long sleep!
		}
	}
}
#endif


void sendData(long dataTosend, long dataType){
	// long sum = atol(ERRORCODE); // original code
	long sum = atol(MIN_ERRORCODE); // error code is at least this number big :-)
	
	trc("DataToSend");
	trc(String(dataTosend));
	trc("DataType");
	trc(String(dataType));


//	if (dataTosend == sum) { // original code
	if (dataTosend >= sum) { 
		// nothing to do sending error code
		sum = dataTosend;
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

