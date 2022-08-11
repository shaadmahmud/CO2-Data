#include <Arduino.h>
#include <U8g2lib.h>
#include <SparkFun_RV8803.h> //Get the library here:http://librarymanager/All#SparkFun_RV-8803


RV8803 rtc;
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
String name = "David Prentice";  //in SRAM
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ 3, /* data=*/ 2, /* reset=*/ U8X8_PIN_NONE); 

void setup(void) {
//u8g2.setI2CAddress(0x3C);
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

  rtc.begin();
  
}

void loop(void) 
{
    int x= analogRead(A0);
   // String currentDate = rtc.stringDateUSA();
    //String currentTime = rtc.stringTime();
    u8g2.firstPage();
    do {
   u8g2.drawStr(0, 15, "A: ");  // write something to the internal memory
          if (rtc.updateTime() == true) //Updates the time variables from RTC
      {
       String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    //String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
        String currentTime = rtc.stringTime(); //Get the time
     }
     u8g2.setCursor(18, 15);  //separate setCursor(), print() accepts all Print.h arguments
        //u8g2.print(F("New Version"));  //
     u8g2.print(currentDate);  //
    } while (u8g2.nextPage());
    delay(1000);


   // u8g2.clearDisplay();
}
