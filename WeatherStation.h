/*
WeatherStation.h - Combined ibrary for our portable weather station with GPS and logging functionality, 
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

#ifndef WEATHERSTATION_H
#define WEATHERSTATION_H

// All of these libraries (except Arduino.h) must be included in the final sketch too, thanks to how the Arduino IDE compiles things.
// As far as I know, this is NOT how it works in C++. To prevent any weirdness, I'm using "" instead of <> here (but use <> in the sketch).
// Reason seems to be that libraries are compiled separately from the sketch or something like that (sorry, not very clear on this).
// Because of this, include guards are necessary.
#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"

#ifndef ADAFRUIT_SI1145_H
#include "Adafruit_SI1145.h"
#endif

#ifndef ADAFRUIT_BMP085_H
#include "Adafruit_BMP085.h"
#endif

#ifndef HTU21D_H
#include "HTU21D.h"
#endif

#ifndef __TinyGPSpp_h
#include "TinyGPSpp.h"
#endif

#ifndef _ADAFRUIT_GFXG_H
#include "Adafruit_GFXG.h"
#endif

/*
#ifndef _ADAFRUIT_PCD8544G_H
#include "Adafruit_PCD8544G.h"
#endif
*/

class WeatherStation
{
	private:
		//private variables/objects should begin with underscores
		
		Adafruit_SI1145 _lightSensor;
		Adafruit_BMP085 _pressureSensor;
		HTU21D _humiditySensor;
		TinyGPSPlus _gps;
		//I don't see any point of making this a part of the library, find in main sketch
		//Adafruit_PCD8544 _lcdDisplay = Adafruit_PCD8544(5, 4, 3);
		
		const int _backlightCtrlPin = A0;
		bool _backlightFlag;
		
		short _visibleIntensity, _IRintensity;
		float  _rms, _pressureTemp, _humidityTemp;
		
		float calcRMS();
		
		void getLocation();
		void getTime();
		void getDate();
		void getAltitude();
		
	public:
	
		bool validL, validD, validT, validA;
		byte Day, Month, Min, Hr;
		short Year;
		float UVindex, humidity, temperature, absolutePressure, relativePressure, estimatedAltitude;
		double latitude, longitude, gpsAltitude_m, gpsAltitude_ft;
		
		WeatherStation();
		bool begin();
		void controlBacklight();
		
		void getGPSdata();
		void getPressureSensorData();
		void getHumiditySensorData();
		void calcTemperature();
		void getUVandLightData();
		
		void displayInfoSerial();
		
};

#endif