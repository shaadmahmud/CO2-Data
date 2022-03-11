
#include <avr/dtostrf.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include "SparkFun_SCD4x_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x
#include <RTCZero.h>
#include "ArduinoLowPower.h"
#include <SensirionI2CScd4x.h>


//SCD4x mySensor;
SFE_UBLOX_GNSS myGNSS;
RTCZero rtc;
SensirionI2CScd4x scd4x;
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
File myFile;

const byte seconds = 0;
const byte minutes = 0;
const byte hours = 16;

/* Change these values to set the current initial date */
const byte day = 25;
const byte month = 9;
const byte year = 15;
volatile bool alarmFlag = false; // Start awake


const int chipSelect = 13;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
char buff[] = "20180810T143000Z";
float pressure;   
float temperature;  
float altimeter; 
float humidity;
float gasResistance;
float CO2;
//float altimeter;

String Year,Month,Day,Hour,Minute,Second;

//double f = 0;
//f=atof();
char charRead;
char runMode;
byte i=0; //counter
char dataStr[100] = "";
char buffer[7];

void setup()
{
 // Open serial communications and wait for port to open:
  Serial.begin(115200);

  rtc.begin(); // initialize RTC 24H format
  resetAlarm();
  
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.print("Initializing SCD41...");
  Wire.begin();
  uint16_t error;
  char errorMessage[256];
  scd4x.begin(Wire);

   //error = scd4x.startPeriodicMeasurement();

   error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }


  error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }



//   if (mySensor.begin() == false)
//  {
//    Serial.println(F("Sensor not detected. Please check wiring. Freezing..."));
//    while (1)
//      ;
//  }

   Serial.print("Initializing uBLOX GPS...");

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }
  myGNSS.setI2COutput(COM_TYPE_UBX); 
   
  Serial.print("Initializing SD card...");
   
 // if (!SD.begin(chipSelect,MOSI1,MISO1,SCK1)) {
    if (!SD.begin(chipSelect,MISO1,MOSI1, SCK1)) {
    Serial.println("initialization failed!");
    return;
  }


  Serial.print("Initializing BME688...");
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME688 sensor, check wiring!");
    while (1);
  }
  
  Serial.println("initialization done.");
    // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms


  if (SD.exists("csv.txt")) 
  {
    Serial.println("Removing simple.txt");
    SD.remove("csv.txt");
    Serial.println("Done");
  } 

  //write csv headers to file:
   myFile = SD.open("csv.txt", FILE_WRITE);  
   if (myFile) // it opened OK
    {
    Serial.println("Writing headers to csv.txt");
    myFile.println("Time,Pressure,Temperature, Humidity, Gas Resistance, ,Altitude, CO2(SCD41)");
    myFile.close(); 
    Serial.println("Headers written");
    }
  else 
    Serial.println("Error opening csv.txt");  

  rtc.attachInterrupt(alarmMatch);
  rtc.standbyMode();
    

}

void loop()
{


if (alarmFlag == true){
    alarmFlag= false; 
    uint16_t error;
    char errorMessage[256];
    uint16_t co2;
    float tempr;
    float humi;
    error = scd4x.readMeasurement(co2, tempr, humi);
  
     if (! bme.performReading()) {
         Serial.println("Failed to perform reading :(");
         return;
        }
 
 dataStr[0] = 0;
 pressure = bme.pressure/100.0;
 temperature = bme.temperature;
 humidity = bme.humidity;
 gasResistance= bme.gas_resistance / 1000.0;
 altimeter = bme.readAltitude(SEALEVELPRESSURE_HPA); 
 //CO2= mySensor.getCO2();

//printTime();
//dtostrf(buff, 5, 1, buffer);
// strcat(dataStr, buffer);//add it onto the end
// strcat( dataStr, ", "); //append the delimeter
 
 //dtostrf(floatVal, minimum width, precision, character array);
dtostrf(pressure, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
 strcat( dataStr, buffer); //append the coverted float
 strcat( dataStr, ", "); //append the delimeter

dtostrf(temperature, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
 strcat( dataStr, buffer); //append the coverted float
 strcat( dataStr, ", "); //append the delimeter
 
dtostrf(humidity, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
 strcat( dataStr, buffer); //append the coverted float
 strcat( dataStr, ", "); //append the delimeter
 
dtostrf(gasResistance, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
 strcat( dataStr, buffer); //append the coverted float
 strcat( dataStr, ", "); //append the delimeter

dtostrf(altimeter, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
 strcat( dataStr, buffer); //append the coverted float
 strcat( dataStr, ", "); //terminate correctly 

dtostrf(co2, 5, 1, buffer);  //5 is mininum width, 1 is precision; float value is copied onto buff
 strcat( dataStr, buffer); //append the coverted float
 strcat( dataStr, 0); //terminate correctly 

 
 //Serial.println(dataStr);
  //printTime();


  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" hPa");
  
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" *C");

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  Serial.print("Gas Resistance= ");
  Serial.print(gasResistance);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(altimeter);
  Serial.println(" m");

  Serial.print(F("CO2:"));
  //Serial.print(mySensor.getCO2());
  Serial.print(co2);
  Serial.println(" ppm");

delay(100);
  
myFile = SD.open("csv.txt", FILE_WRITE);     
    // if the file opened okay, write to it:
    if (myFile) 
    {
      printTime();
      //Serial.println("Writing to csv.txt");
      myFile.print(buff); 
      myFile.print("  "); 
      myFile.println(dataStr); 
      myFile.close();
    } 
    else 
    {
      Serial.println("error opening csv.txt");
    }
    delay(200);  
    resetAlarm(); 
  
// LowPower.companionSleep();
 // LowPower.sleep(2000);

  }


rtc.standbyMode(); 
  
}

void printTime(){
  
  if (millis() - lastTime > 1000) 
  {
    lastTime = millis();

Year = myGNSS.getYear();
Month= myGNSS.getMonth();
Day= myGNSS.getDay();
Hour= myGNSS.getHour();
Minute=myGNSS.getMinute();
Second= myGNSS.getSecond();

sprintf(buff, "%02d/%02d/%02d %02d:%02d:%02d", myGNSS.getYear(),myGNSS.getMonth(), myGNSS.getDay(), myGNSS.getHour(), myGNSS.getMinute(),myGNSS.getSecond());
Serial.println(buff);
  }
}

void alarmMatch()
{
  Serial.println("Alarm Match!");
  alarmFlag = true; // Set flag

}

void resetAlarm(void) {
  Serial.println("Reset Alarm Function!");
  rtc.setTime(hours, minutes, seconds);
  rtc.setDate(day, month, year);

  rtc.setAlarmTime(16, 00, 10);
  rtc.enableAlarm(rtc.MATCH_HHMMSS);
}
 
