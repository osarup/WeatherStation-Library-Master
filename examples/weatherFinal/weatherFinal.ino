/*
weatherFinal.ino -  front end sketch for our project and library

Used with the following hardware:

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

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_BMP085.h>
#include <HTU21D.h>
#include <Adafruit_GFXG.h>
#include <Adafruit_PCD8544G.h>
#include <TinyGPSpp.h>
#include <WeatherStation.h>

const int toggleButtonPin = A1;
const int intrPin = 2;
int continuousLog;
bool externIntrDisabled;

//add more sensors here, don't add a comma after the last header name.
char *headerList[] = {"Date(DD-MM-YYYY),","Time(HHMM),","Latitude,","Longitude,","Altitude(m),","Estimated Altitude(m),","Temperature(*C),","Absolute Pressure(mBar),","Relative Pressure(mBar),","Relative Humidity(%),","UV Index"};

char *filePathName = "defaultLog.csv";

Adafruit_PCD8544 lcdDisplay = Adafruit_PCD8544(5, 4, 3);

WeatherStation stationObject;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600); //needed for status and debugging messages
  
  delay(5000); //to allow for the user the time to start the serial monitor in time, could always restart the sketch though
  
  //Set appropriate mode for the pin whose switch indicates continuous data logging.
  //Make sure continuous logging is OFF.
  pinMode(toggleButtonPin, INPUT);
  continuousLog = LOW;
  
  //Set interrupt 0 on pin 2 (falling edge triggered)
  attachInterrupt(intrPin,intrRoutine, FALLING);
  
  //Keep track of the above external interrupt being enabled or disabled.
  externIntrDisabled = false;
  
  //initialise SD card
  Serial.println("Initialising SD card interface...");
  if (!SD.begin()) 
  {
  	Serial.println("Failed to init SD card!");
  }
  
  //check if file of same name exists, if it does, then no need for column headers.
  if(!SD.exists(filePathName))
  {
    makeLogHeaders ();
  }
  
  //initialise LCD display
  lcdDisplay.begin();
  lcdDisplay.setContrast(45);
  lcdDisplay.clearDisplay();
  lcdDisplay.setCursor(0,20); 
  lcdDisplay.print("Initialised...");
  lcdDisplay.display();
  
  if(!stationObject.begin())
  {
    Serial.println("Oops! Something went wrong!");
    //suspend program
    while(1);
  }
  
  delay(2000);

}

void loop() 
{
  // put your main code here, to run repeatedly:
  
  // Get input from sensors
  stationObject.getGPSdata();
  stationObject.getPressureSensorData();
  stationObject.getHumiditySensorData();
  stationObject.getUVandLightData();
  
  // Process some of the collected data
  stationObject.calcTemperature();
  stationObject.controlBacklight();
  
  // Output data to appropriate destinations
  stationObject.displayInfoSerial();
  checkForContinuousLogging();
  displayInfoLCD();
}

//---------------------------------LCD STUFF-------------------------------------
void displayInfoLCD()
{
  //Prepare first screen of info
  lcdDisplay.clearDisplay(); //clears display AND BUFFER BE CAREFUL
  /*
    //Display time and date if you want to
    
    lcdDisplay.print("\nTime: ");
    if (stationObject.validT && stationObject.validL)
    {
	if (Hr < 10) lcdDisplay.print("0");
	lcdDisplay.print(stationObject.Hr);
	lcdDisplay.print(":");
	if (Min < 10) lcdDisplay.print("0");
	lcdDisplay.print(stationObject.Min);
    }
    else
    {
	lcdDisplay.print("INVALID");
    }
	
    lcdDisplay.print("\nDate: ");
    if (validD && validL)
    {
	lcdDisplay.print(Day);
	lcdDisplay.print("-");
	lcdDisplay.print(stationObject.Month);
	lcdDisplay.print("-");
	lcdDisplay.print(stationObject.Year);
    }
    else
    {
	lcdDisplay.print("INVALID");
    }
   */
	 
    if (stationObject.validL)
    {
      lcdDisplay.print("Lat: ");
      lcdDisplay.println(stationObject.latitude, 5);
      lcdDisplay.print("Lon: ");
      lcdDisplay.println(stationObject.longitude, 5);
    }
    else
    {
      lcdDisplay.print("Lat: ");
      lcdDisplay.println("INVALID");
      lcdDisplay.print("Lon: ");
      lcdDisplay.println("INVALID");
    }
	  
    if (stationObject.validA)
    {
      lcdDisplay.print("Alt: ");
      lcdDisplay.print(stationObject.gpsAltitude_m);
      lcdDisplay.println("m");
    }
    else
    {
      lcdDisplay.print("Est Alt: ");
      lcdDisplay.print(stationObject.estimatedAltitude,0);
      lcdDisplay.println("m");
    }
    
    //Present first screenful of information
    lcdDisplay.display();
    delay(5000);
    
    //Prepare second screen of information
    lcdDisplay.clearDisplay();
    lcdDisplay.print("Temp: ");
    lcdDisplay.print(stationObject.temperature);
    lcdDisplay.println("*C");

    lcdDisplay.println("Rel Pres: ");
    lcdDisplay.print(stationObject.relativePressure,1);
    lcdDisplay.println(" mBar");
	
    lcdDisplay.print("Hum: ");
    lcdDisplay.print(stationObject.humidity);
    lcdDisplay.println("%");
	
    lcdDisplay.print("UV Ind: ");
    lcdDisplay.println(stationObject.UVindex);
    
    //Present second screen to user.
    lcdDisplay.display();
    delay(5000);
}
//---------------------------------PUSH BUTTONS AND ISR-------------------------------------
//checks whether the SD card should be continuously written to.
void checkForContinuousLogging()
{
  continuousLog = digitalRead(toggleButtonPin);
  
  switch (continuousLog)
  {
    case HIGH:  if (!externIntrDisabled)
                {
                  detachInterrupt(intrPin);
                  externIntrDisabled = true;
                  Serial.println("Continuous logging enabled, external interrupts disabled!");
                }
                writeSensorLog();
                break;
    
    default:   if (externIntrDisabled)
                {
                  attachInterrupt(intrPin, intrRoutine, FALLING);
                  externIntrDisabled = false;
                  Serial.println("Continuous logging disabled, external interrupts enabled!");
                }
  }
}

//The interrupt service routine for writing to the SD card on demand
void intrRoutine()
{
	writeSensorLog();
        Serial.println("ISR Triggered!");
}
//------------------------------------------SD CARD FUNCTIONS---------------------------------
void makeLogHeaders()
{
  File myFile;
  String displayString, errorString, writeString, blankString = "";
  
  /*if(!SD.exists(filePathName))
  {
   Serial.println("defaultLog.csv does not exist, creating.");
   system("touch /media/mmcblk0p1/defaultLog");
  }*/
  
  myFile = SD.open(filePathName, FILE_WRITE);
  
  if (myFile) 
  {
    displayString = blankString + "Writing headers to " + filePathName + "...";
    Serial.print(displayString);
    
    //writeString = blankString + header1 + "," + header2 + "," + header3 + "," + header4; //extend this if new sensors added, don't forget commas!
    
    //commmas should be part of the header string, simpler
    writeString = blankString + headerList[0] + headerList[1] + headerList[2] + headerList[3] + headerList[4] + headerList[5] + headerList[6] + headerList[7] + headerList[8] + headerList[9] + headerList[10];
    myFile.println(writeString); 
     
    myFile.close();
    
    Serial.println("done.");
  } 
  else 
  {
    // if the file didn't open, print an error:
    errorString = blankString + "Could not write to " + filePathName;
    Serial.println(errorString);
  }
}

//The actual sensor data gets used here, be careful of the order.
//Don't add a comma after the last sensor, just \n or println (not both)
void writeSensorLog ()
{
  File myFile;
  String displayString, errorString, hrStr, minStr, timeString="INVALID", dateString="INVALID", blankString = "";
  
  /*if(!SD.exists(filePathName))
  {
   Serial.println("test.txt does not exist, creating.");
   system("touch /media/mmcblk0p1/test.txt");
  }*/
  
  //Prepare formatted time string
  if (stationObject.validT && stationObject.validL)
  {
    if (stationObject.Hr < 10)
    hrStr = blankString + "0" + stationObject.Hr;
    else
    hrStr = blankString + stationObject.Hr;
    
    if (stationObject.Min < 10)
    minStr = blankString + "0" + stationObject.Min;
    else
    minStr = blankString + stationObject.Min;
    
    timeString = blankString + hrStr + ":" + minStr;
  }
  
  //Prepare formatted date string
  if (stationObject.validD && stationObject.validL)
  {
    dateString = blankString + stationObject.Day + "-" + stationObject.Month + "-" + stationObject.Year;
  }
  
  myFile = SD.open(filePathName, FILE_WRITE);
  
  if (myFile) 
  {
    displayString = blankString + "Writing data to " + filePathName + "...";
    Serial.print(displayString);
    
    myFile.print(dateString); myFile.print(",");  //0
    myFile.print(timeString); myFile.print(",");  //1
    
    if (stationObject.validL)
    {
      myFile.print(stationObject.latitude); myFile.print(",");  //2
      myFile.print(stationObject.longitude); myFile.print(",");  //3
    }
    else
    {
      myFile.print("INVALID,");
      myFile.print("INVALID,");
    }
    
    if (stationObject.validA)
    {
      myFile.print(stationObject.gpsAltitude_m); myFile.print(",");  //4
    }
    else
    {
      myFile.print("INVALID,");
    }
    
    
    myFile.print(stationObject.estimatedAltitude); myFile.print(",");  //5
    myFile.print(stationObject.temperature); myFile.print(",");  //6
    myFile.print(stationObject.absolutePressure); myFile.print(",");  //7
    myFile.print(stationObject.relativePressure); myFile.print(",");  //8
    myFile.print(stationObject.humidity); myFile.print(",");  //9
    myFile.println(stationObject.UVindex); //10
    //extend this if new sensors added
     
    myFile.close();
    
    Serial.println("done.");
  } 
  else 
  {
    // if the file didn't open, print an error:
    errorString = blankString + "Could not write to " + filePathName;
    Serial.println(errorString);
  }
}
//----------------------------------------------------------------------------------------------------
