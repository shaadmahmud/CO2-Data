#include <Arduino.h>
#include <U8g2lib.h>
#include <SparkFun_RV8803.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
String name = "David Prentice";  //in SRAM


/// i2c enum //
U8G2_SSD1306_128X32_UNIVISION_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ 3, /* data=*/ 2, /* reset=*/ U8X8_PIN_NONE); 
RV8803 rtc;



void setup(void) {

  Wire.begin();

  if (!rtc.begin()) //Initilizes the RH_ASK object
    Serial.println("INIT FAILED");

  u8g2.begin();  

  
  
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(0,15,"iBUG Stick V1.1");
    u8g2.drawStr(0,32,"UNH");
    //u8g2.drawBox(3,7,25,15);
  } 
  
  while ( u8g2.nextPage() );

  delay(2000);
  
}

void loop(void) 
{
    int x= analogRead(A0);
//    u8g2.firstPage();
//    do {
//        u8g2.drawStr(0, 15, "A: ");  // write something to the internal memory
//        //u8g2.drawStr(10, 20, name.c_str());  // write something to the internal memory
//        //u8g2.drawStr(5, 30, fixed.c_str());  // write String from SRAM
//        //u8g2.drawStr(0, 40, F("ORIGINAL IN FLASH").c_str());  // does not work
//        u8g2.setCursor(18, 15);  //separate setCursor(), print() accepts all Print.h arguments
//        //u8g2.print(F("New Version"));  //
//        u8g2.print(x);  //
//    } while (u8g2.nextPage());

    printTime();
    delay(1000);


   // u8g2.clearDisplay();
}


void printTime(){
  
    if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    //String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
    String currentTime = rtc.stringTime(); //Get the time
    //Serial.print(currentDate);
    //Serial.print(" ");
   // Serial.println(currentTime);
    u8g2.firstPage();
    do {
        u8g2.drawStr(0, 15, "Time: ");  // write something to the internal memory
        //u8g2.drawStr(10, 20, name.c_str());  // write something to the internal memory
        //u8g2.drawStr(5, 30, fixed.c_str());  // write String from SRAM
        //u8g2.drawStr(0, 40, F("ORIGINAL IN FLASH").c_str());  // does not work
        u8g2.setCursor(0, 30);  //separate setCursor(), print() accepts all Print.h arguments
        //u8g2.print(F("New Version"));  //
        u8g2.print(currentTime);  //
    } while (u8g2.nextPage());
    
  }
  else
  {
    Serial.print("RTC read failed");
  }
 }
