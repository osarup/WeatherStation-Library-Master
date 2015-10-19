/*
WeatherStation.cpp - Combined ibrary for our portable weather station with GPS and logging functionality, 
made to be used with our senior year engineering project. Used with the following hardware:

-> Intel Galileo Gen 2
-> GY-GPS6MV2 with Ublox Neo-6M
-> Adafruit BMP180 Breakout Board (Pressure and Temperature)
-> Adafruit SI1145 Breakout Board (UV/Visible/IR detector)
-> Sparkfun HTU21D Breakout Board (Humidity and Temperature)
-> Nokia 5110-like PCD8544 based LCD (48x24)
-> SD card for datalogging and Linux installation (for permanent sketch functionality).

Libraries used:
-> Adafruit PCD8544 Nokia LCD library*
-> Adafruit GFX Library
-> Adafruit SI1145 library*
-> Adafruit BMP085 library*
-> Sparkfun HTU21D library*
-> Mikal Hart's TinyGPS++ library*

*slight modifications had to be made to these libraries for compatibility,
documented in lib_modifications.txt

Please note that this is library assumes you're running the latest version of Intel's Arduino IDE and is not
designed or tested for older versions of the software.
We used version 1.6.0.

Please note that the Galileo Gen 2 uses the following sizes of datatypes:
bool - 1 bit boolean
byte - 8 bits (1 Byte) integer
short - 16b or 2B integer
int - 32b or 4B integer
float - 32b or 4B floating point
double - 64b or 8B floating point

I/O interfaces used:
Serial - USB
Serial1 - Pins 0,1
I2C - Pins A4,A5 and dedicated SDA, SCL (but they're internally shorted with A4,A5, as in the Arduino Uno).
SPI - we've used hardware SPI pins 3,4,5,11,13
SDIO - dedicated SD card I/O interface
A1 - push button input
A0 - backlight control output
Digital pin 2 - external interrupt activated by push button

Created by Ojas Sarup; 9th April 2015
*/

#include "WeatherStation.h"

//constructor class; blank for now
WeatherStation::WeatherStation(){}

bool WeatherStation::begin()
{	
	//set appropriate pin mode for LCD's backlight control pin, and make sure it's off.
	pinMode(_backlightCtrlPin, OUTPUT);
	digitalWrite(_backlightCtrlPin, LOW);
	_backlightFlag = false;
	
	//initialise I2C bus and sensors
	Wire.begin();
	
	Serial.println("Checking for SI1145 sensor...");
	if (!_lightSensor.begin()) 
	{
		Serial.println("Could not find a valid SI1145 sensor, check wiring!");
		return false;
	}
  
	Serial.println("Checking for BMP180 sensor...");
	if (!_pressureSensor.begin()) 
	{
		Serial.println("Could not find a valid BMP180 sensor, check wiring!");
		return false;
	}
  
	Serial.println("Checking for HTU21D sensor...");
	if (_humiditySensor.readHumidity() == 998) 
	{
		Serial.println("Could not find a valid HTU21D sensor, check wiring!");
		return false;
	}
	
	//Start Serial1 for the GPS. Takes some time to start the interface so delay a sec
	Serial1.begin(9600);
	delay(1000);
	
	Serial.println("Checking for GPS...");
	//check for GPS
	unsigned long ms = millis();
	while ((Serial1.available() > 0) && ((millis()-ms) < 5000))
	{
		_gps.encode(Serial1.read());
	}
	if (_gps.charsProcessed() < 10)
	{
		Serial.println("No GPS detected: check wiring.");
		return false;
	}
	
	//ack completed init process to USB console
	Serial.println("OK!");
	
	//return true if all went well
	return true;
}

float WeatherStation::calcRMS()
{
	return sqrt((pow(_visibleIntensity*2,2)+pow(_IRintensity,2))/2);
}

void WeatherStation::controlBacklight()
{
	_rms = calcRMS();
	
	if (_rms < 409.5 && !_backlightFlag)
	{
		digitalWrite(_backlightCtrlPin,HIGH);
		_backlightFlag = true;
	}
	else if (_rms > 413.0 && _backlightFlag)
	{
		digitalWrite(_backlightCtrlPin,LOW);
		_backlightFlag = false;
	}
}

void WeatherStation::calcTemperature()
{
	temperature = (_pressureTemp + _humidityTemp)/2.0;
}

//Serial output from all sensors.
void WeatherStation::displayInfoSerial()
{
	Serial.print("\nTime: ");
	if (validT && validL)
	{
		if (Hr < 10) Serial.print("0");
		Serial.print(Hr);
		Serial.print(":");
		if (Min < 10) Serial.print("0");
		Serial.print(Min);
	}
	else
	{
		Serial.print("INVALID");
	}
	
	Serial.print("\nDate: ");
	if (validD && validL)
	{
		Serial.print(Day);
		Serial.print("-");
		Serial.print(Month);
		Serial.print("-");
		Serial.print(Year);
	}
	else
	{
		Serial.print("INVALID");
	}
	
	if (validL)
	{
		Serial.print("\nLat: ");
		Serial.println(latitude, 6);
		Serial.print("Lon: ");
		Serial.print(longitude, 6);
	}
	else
	{
		Serial.print("\nLocation: "); 
		Serial.print("INVALID");
	}
	  
	Serial.print("\nAltitude: ");
	if (validA)
	{
		Serial.print(gpsAltitude_m);
		Serial.print(" m or ");
		Serial.print(gpsAltitude_ft);
		Serial.print("feet");
	}
	else
	{
		Serial.print("INVALID");
	}
	
	Serial.print("\nEstimated Altitude(m): ");
	Serial.println(estimatedAltitude);
	
	Serial.print("Temperature(*C): ");
	Serial.println(temperature);
	
	Serial.print("Absolute Pressure(mBar): ");
	Serial.println(absolutePressure);
	
	Serial.print("Relative Pressure(mBar): ");
	Serial.println(relativePressure);
	
	Serial.print("Relative Humidity(%): ");
	Serial.println(humidity);
	
	Serial.print("UV Index: ");
	Serial.println(UVindex);
}

//-------------------------------------------PRESSURE SENSOR FUNCTIONS---------------------------------------------------------------------------------------------------
void WeatherStation::getPressureSensorData()
{
	_pressureTemp = _pressureSensor.readTemperature();
	absolutePressure = _pressureSensor.readPressure()/100.0;	//divide by 100 to convert pascals to milibar
	estimatedAltitude = _pressureSensor.readAltitude(); 	//according to standard sea level pressure
	
	if (validA)
		relativePressure = _pressureSensor.readSealevelPressure((float)gpsAltitude_m)/100.0;	//typecast to float from double
	else
		relativePressure  = absolutePressure; //readSealevelPressure takes altitude = 0m by default. Using estimatedAltitude as input would return absolutePressure again.
}
//-------------------------------------------HUMIDITY SENSOR FUNCTIONS---------------------------------------------------------------------------------------------------
void WeatherStation::getHumiditySensorData()
{
	_humidityTemp = _humiditySensor.readTemperature();
	humidity = _humiditySensor.readHumidity();
}
//-------------------------------------------UV AND LIGHT SENSOR FUNCTIONS-----------------------------------------------------------------------------------------------
//call this BEFORE the control backlight function
void WeatherStation::getUVandLightData()
{
	_visibleIntensity = _lightSensor.readVisible();
	_IRintensity = _lightSensor.readIR();
	UVindex = _lightSensor.readUV();
}
//-------------------------------------------GPS FUNCTIONS---------------------------------------------------------------------------------------------------------------
void WeatherStation::getLocation()
{
  if (_gps.location.isValid())
  {
	validL = true;
	latitude = _gps.location.lat();
	longitude = _gps.location.lng();
  }
  else
  {
	validL = false;
  }
}

void WeatherStation::getDate()
{
  if (_gps.date.isValid())
  {
	validD = true;
	Day = _gps.date.day();
	Month = _gps.date.month();
	Year = _gps.date.year();
  }
  else
  {
	validD = false;
  }
}

void WeatherStation::getTime()
{
  if (_gps.time.isValid())
  {
	validT = true;
	Min = _gps.time.minute();
	Hr = _gps.time.hour();
  }
  else
  {
	validT = false;
  }
}

void WeatherStation::getAltitude()
{
  if (_gps.altitude.isValid())
  {
	validA = true;
	gpsAltitude_m = _gps.altitude.meters();
	gpsAltitude_ft = _gps.altitude.feet();
  }
  else
  {
	validA = false;
  }
}
  
void WeatherStation::getGPSdata()
{
	if((Serial1.available() > 0) && _gps.encode(Serial1.read()));
	{
		getLocation();
		getDate();
		getTime();
		getAltitude();
	}
}
//-------------------------------------------THE END---------------------------------------------------------------------------------------------------------------------