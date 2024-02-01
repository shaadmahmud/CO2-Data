#if !defined(ARDUINO_ARCH_RP2040)
  #error For RP2040 only
#endif

#if defined(ARDUINO_ARCH_MBED)
  
  #define PIN_SD_MOSI       PIN_SPI_MOSI
  #define PIN_SD_MISO       PIN_SPI_MISO
  #define PIN_SD_SCK        PIN_SPI_SCK
  #define PIN_SD_SS         PIN_SPI_SS

#else

  #define PIN_SD_MOSI       PIN_SPI0_MOSI
  #define PIN_SD_MISO       PIN_SPI0_MISO
  #define PIN_SD_SCK        PIN_SPI0_SCK
  #define PIN_SD_SS         PIN_SPI0_SS
  
#endif

#include <Arduino.h>
#include <U8g2lib.h>
#include <SparkFun_RV8803.h> //Get the library here:http://librarymanager/All#SparkFun_RV-8803
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <RP2040_SD.h>
#include "mbed.h"
#include "rtos.h"
#include <avr/dtostrf.h>
File root;

Adafruit_MPU6050 mpu;

RV8803 rtc;
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
String name = "David Prentice";  //in SRAM
String data = "";
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ 3, /* data=*/ 2, /* reset=*/ U8X8_PIN_NONE); 
float X;
float Y;
float Z;
float startMillis;
float currentMillis;
float rawMillis;


void setup(void) {
//u8g2.setI2CAddress(0x3C);
u8g2.begin();
if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
}

Serial.print("Initializing SD card with SS = ");  Serial.println(PIN_SD_SS);
Serial.print("SCK = ");   Serial.println(PIN_SD_SCK);
Serial.print("MOSI = ");  Serial.println(PIN_SD_MOSI);
Serial.print("MISO = ");  Serial.println(PIN_SD_MISO);

if (!SD.begin(PIN_SD_SS)) 
 {
    Serial.println("Initialization failed!");
    return;
  }
  
Serial.println("Initialization done.");

mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
mpu.setGyroRange(MPU6050_RANGE_250_DEG);
mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
Serial.println("");
delay(100);


u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(0,15,"iBUG Stick V1.1");
    u8g2.drawStr(0,32,"UNH");
    //u8g2.drawBox(3,7,25,15);
    delay(500);
  } 
  
  while ( u8g2.nextPage() );

  delay(500);

 // rtc.begin();
  
}

void loop(void) 
{
  String dataString = "";
 // sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);




// data = "AccelX = " + String(a.acceleration.x) + " " + " AccelY = " + String(a.acceleration.y) + " " + " AccelZ = " + String(a.acceleration.z) + " "+ " GyroX = " + String(g.gyro.x) + " " + " GyroY = " + String(g.gyro.y) + " " + "GyroZ = " + String(g.gyro.z);
// delay(1000); 

//data = String(a.acceleration.x) + " " + String(a.acceleration.y) + " " + String(a.acceleration.z) + " "+ String(g.gyro.x) + " " +  String(g.gyro.y) + " " + String(g.gyro.z);
//delay(1000); 

#define fileName  "DATALOG.txt"
//File dataFile = SD.open("DATALOG.txt", FILE_WRITE);
File dataFile = SD.open("DATALOG.txt", O_CREAT | O_WRITE | O_APPEND);

// if (dataFile) 
//   {
//     dataFile.println(data);
//     dataFile.flush();
//     dataFile.close();
//     Serial.println(data);
   
//    }
if (dataFile) {                                                                 // if the file is available, write to it:

  for(uint8_t i = 0; i < 50; i++){                                                 // For i = 0, execute the below code. Then increment i by 1. Once past 250, exit for() loop
    //digitalWrite(recordingLed, HIGH);

    sensors_event_t a, g, temp;                                                     // Commands to trigger event sensing and acceleration logging.
    mpu.getEvent(&a, &g, &temp);

    X = a.acceleration.x/9.81;                                                      // Divide normal acceleration return values to calculate G forces
    Y = a.acceleration.y/9.81;
    Z = a.acceleration.z/9.81;
   
  u8g2.firstPage();
  do {
  u8g2.drawStr(0, 15, "X: ");  // write something to the internal memory
  u8g2.setCursor(18, 15);  //separate setCursor(), print() accepts all Print.h arguments
  u8g2.print(X);  // 
  u8g2.drawStr(60, 15, ", Y: ");
  u8g2.setCursor(86, 15); 
  u8g2.print(Y);
  u8g2.drawStr(0, 32, "Z: ");
  u8g2.setCursor(18, 32); 
  u8g2.print(Z);
  } while (u8g2.nextPage());


    rawMillis = millis()-startMillis;                                               // Take our current millis value and subtract our beginning startMillis from it to get how long we've actually been recording.

    dataFile.print(rawMillis/1000, 3);                                              // Store data in a Comma-Seperated-Value format. The '3' after rawMillis/1000 says print out to 3 decimal places.
    dataFile.print(", ");                                                           // I just use a txt file since GNUplot can plot from that value, but a CSV library could probably be used here.
    dataFile.print(X);                                                              // Excel doesn't allow plots beyond 255 datapoints for god knows why. So GNUplot is basically a necessity to plot these accurately to timestamp.
    dataFile.print(", ");
    dataFile.print(Y);
    dataFile.print(", ");
    dataFile.print(Z);
    dataFile.println();
    Serial.print("AccelX:");
    Serial.print(X);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(Y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.println(Z);
   
  delay(10);
    }
    dataFile.flush();
    dataFile.close();
 }
  // if the file isn't open, pop up an error:
else 
      {
        Serial.print("Error opening "); Serial.println(fileName);
      }  
  // Serial.print("AccelX:");
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print("AccelY:");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // Serial.print("AccelZ:");
  // Serial.print(a.acceleration.z);
  // Serial.print(", ");
  // Serial.print("GyroX:");
  // Serial.print(g.gyro.x);
  // Serial.print(",");
  // Serial.print("GyroY:");
  // Serial.print(g.gyro.y);
  // Serial.print(",");
  // Serial.print("GyroZ:");
  // Serial.print(g.gyro.z);
  // Serial.println("");

      
//delay(1000);



   // u8g2.clearDisplay();
}