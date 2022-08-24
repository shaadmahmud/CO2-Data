#include <Arduino.h>
#include "LoRaWan-Arduino.h" //http://librarymanager/All#SX126x
#include <SPI.h>
#include <SparkFun_RV8803.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <RP2040_SD.h>
//#include "pico/stdlib.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include <LoRa.h>
#include "hardware/xosc.h"
//#include "hardware/rosc.h"


#include <stdio.h>

#include "mbed.h"
#include "rtos.h"
#include <Adafruit_NeoPixel.h>

#define PIN_SD_MOSI       PIN_SPI_MOSI
#define PIN_SD_MISO       PIN_SPI_MISO
#define PIN_SD_SCK        PIN_SPI_SCK
#define PIN_SD_SS         PIN_SPI_SS



#define DONEPIN 8
using namespace std::chrono_literals;
using namespace std::chrono;
#define SEALEVELPRESSURE_HPA (1013.25)


#define PIN        7 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 1 // Popular NeoPixel ring size


const String DELIMITER = "|";
const int ADDRESS = 116;

RV8803 rtc;
Adafruit_BME680 bme;


Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ 3, /* data=*/ 2, /* reset=*/ U8X8_PIN_NONE); 
bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_1                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5             /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;         /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;         /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                      /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_OFF, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
void lorawan_unconf_finished(void);
void lorawan_conf_finished(bool result);
String currentDate;
String currentTime;

String dataString = "";

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                          lorawan_rx_handler, lorawan_has_joined_handler,
                                          lorawan_confirm_class_handler, lorawan_join_failed_handler,
                                          lorawan_unconf_finished, lorawan_conf_finished
                                         };
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x43, 0xE6};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE9};
uint8_t nodeAppKey[16] = {0x08, 0x71, 0x89, 0xF3, 0x26, 0xD4, 0xC0, 0xC7, 0x8E, 0xFC, 0x05, 0x05, 0x36, 0x53, 0x82, 0xBF};



//uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0x45, 0x30};
//uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF};
//uint8_t nodeAppKey[16] = {0x59, 0xC4, 0x9B, 0xA6, 0xA6, 0x13, 0xE7, 0x6D, 0x03, 0x2C, 0xC2, 0x68, 0x4F, 0x2D, 0xCB, 0x9F};




// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
//#define LORAWAN_APP_INTERVAL 600000  
#define LORAWAN_APP_INTERVAL 500     /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */ 
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

mbed::Ticker appTimer;
void tx_lora_periodic_handler(void);

static uint32_t count = 0;
static uint32_t count_fail = 0;

bool send_now = false;

void setup()
{
  pinMode(24, OUTPUT);
  digitalWrite(24, LOW);
  pinMode(DONEPIN, OUTPUT);
  digitalWrite(DONEPIN, LOW);


  // Initialize Serial for debug output
  time_t timeout = millis();
  
  Wire.begin();
  //Serial.begin(115200);
  if (!SD.begin(PIN_SD_SS)) 
  {
    return;
  }
  
  //Serial.println("Initialization done.");


//Set Clock Speed
 // set_sys_clock_khz(10000, false); 
  set_sys_clock_khz(78000,false);
  
 
 turnOffADC();
 //turnOffUsb();
 //xosc_disable();
  
  u8g2.begin();

  pixels.begin();
  
  if (!bme.begin()) {
    //Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_ncenB12_tr);
    u8g2.drawStr(0,15,"iBUG LoRaWan");
    u8g2.drawStr(0,32,"UNH RSL");
    //u8g2.drawBox(3,7,25,15);
  } while ( u8g2.nextPage() );


  // Initialize LoRa chip.
  lora_rak11300_init();


  if (rtc.begin() == false)
  {
    //Serial.println("Something went wrong, check wiring");
    while(1);
  }
  //Serial.println("RTC online!");


  //Serial.println("=====================================");
  //Serial.println("Welcome to RAK11300 LoRaWan!!!");
  if (doOTAA)
  {
    //Serial.println("Type: OTAA");
  }
  else
  {
    //Serial.println("Type: ABP");
  }

//  switch (g_CurrentRegion)
//  {
//    case LORAMAC_REGION_AS923:
//      Serial.println("Region: AS923");
//      break;
//    case LORAMAC_REGION_AU915:
//      Serial.println("Region: AU915");
//      break;
//    case LORAMAC_REGION_CN470:
//      Serial.println("Region: CN470");
//      break;
//    case LORAMAC_REGION_CN779:
//      Serial.println("Region: CN779");
//      break;
//    case LORAMAC_REGION_EU433:
//      Serial.println("Region: EU433");
//      break;
//    case LORAMAC_REGION_IN865:
//      Serial.println("Region: IN865");
//      break;
//    case LORAMAC_REGION_EU868:
//      Serial.println("Region: EU868");
//      break;
//    case LORAMAC_REGION_KR920:
//      Serial.println("Region: KR920");
//      break;
//    case LORAMAC_REGION_US915:
//      Serial.println("Region: US915");
//      break;
//    case LORAMAC_REGION_RU864:
//      Serial.println("Region: RU864");
//      break;
//    case LORAMAC_REGION_AS923_2:
//      Serial.println("Region: AS923-2");
//      break;
//    case LORAMAC_REGION_AS923_3:
//      Serial.println("Region: AS923-3");
//      break;
//    case LORAMAC_REGION_AS923_4:
//      Serial.println("Region: AS923-4");
//      break;
//  }
//  Serial.println("=====================================");

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  uint32_t err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0)
  {
   // Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
  //turnOffUsb();

}

void loop()
{
  // Every LORAWAN_APP_INTERVAL milliseconds send_now will be set
  // true by the application timer and collects and sends the data
  if (send_now)
  {
    //Serial.println("Sending frame now...");
     //clocks_init();
    send_now = false;
    send_lora_frame();
    delay(2000);
    digitalWrite(DONEPIN, HIGH);
    delay(1);
    digitalWrite(DONEPIN, LOW);
    delay(1);
  }
  //LoRa.sleep();
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void)
{
  if (doOTAA == true)
  {
    //Serial.println("OTAA Mode, Network Joined!");
    
  }
  else
  {
   // Serial.println("ABP Mode");
  }

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    // Start the application timer. Time has to be in microseconds
    appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  //Serial.println("OTAA join failed!");
  //Serial.println("Check your EUI's and Keys's!");
 // Serial.println("Check if a Gateway is in range!");
}
/**@brief Function for handling LoRaWan received data from Gateway
   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  //Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
    //            app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  //Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void lorawan_unconf_finished(void)
{
  //Serial.println("TX finished");
}

void lorawan_conf_finished(bool result)
{
  //Serial.printf("Confirmed TX %s\n", result ? "success" : "failed");
}

void send_lora_frame(void)
{

  
  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);

#define fileName  "datalog.txt"



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.



// blink
blinky(300);

//RTC time and date
updateTD();
//Serial.print(currentDate);
//Serial.print(" ");
//Serial.println(currentTime);

// Update Oled
updateOled();

// Read BME688

  if (! bme.performReading()) {
    //Serial.println("Failed to perform reading :(");
    return;
  }
  uint16_t temp = bme.temperature;
  uint16_t hm = bme.humidity;
  uint16_t pr = bme.pressure / 100.0;
  uint16_t gs= bme.gas_resistance / 1000.0; 


  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = 0x02;
  m_lora_app_data.buffer[i++] = (uint8_t)(temp >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)temp;
  m_lora_app_data.buffer[i++] = (uint8_t)(hm >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)hm;
  m_lora_app_data.buffer[i++] = (uint8_t)((pr & 0xFF000000) >> 24);   
  m_lora_app_data.buffsize = i;
  //Serial.printf("Data %d\n", m_lora_app_data);

File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) 
  {
    dataFile.println(pr);
    dataFile.close();
    
    // print to the serial port too:
    //Serial.print("SD Data: ");
    //Serial.println(pr);
  }
  // if the file isn't open, pop up an error:
  else 
  {
   // Serial.print("Error opening "); Serial.println(fileName);
  }
  
  
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    //Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    //Serial.printf("lmh_send fail count %d\n", count_fail);
  }

  
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void)
{
  appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  // This is a timer interrupt, do not do lengthy things here. Signal the loop() instead
  send_now = true;
}



void blinky(int tm){
  digitalWrite(24, HIGH);
  delay(tm);
  digitalWrite(24, LOW);
  delay(tm);
  digitalWrite(24, HIGH);
  delay(tm);
  digitalWrite(24, LOW);  
  }

void updateTD(){
    if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    //String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
    currentTime = rtc.stringTime(); //Get the time
  }
  else
  {
    //Serial.print("RTC read failed");
  }
}

void updateOled(){
   u8g2.firstPage();
   do { 
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_t0_11_tf);
    u8g2.drawStr(0, 15, "T: ");  // write something to the internal memory
    u8g2.setCursor(18, 15);  //separate setCursor(), print() accepts all Print.h arguments
    u8g2.print(currentTime);  //
    u8g2.drawStr(0, 30, "D: ");
    u8g2.setCursor(18, 30);
    u8g2.print(currentDate); 
    
    } while (u8g2.nextPage());
 }


void set_sys_clock_pll(uint32_t vco_freq, uint post_div1, uint post_div2) {
  if (!running_on_fpga()) {
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    pll_init(pll_sys, 1, vco_freq, post_div1, post_div2);
    uint32_t freq = vco_freq / (post_div1 * post_div2);

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    0,  // No aux mux
                    12 * MHZ,
                    12 * MHZ);

    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    freq, freq);

    clock_configure(clk_peri,
                    0,  // Only AUX mux on ADC
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);
  }
}
bool check_sys_clock_khz(uint32_t freq_khz, uint *vco_out, uint *postdiv1_out, uint *postdiv_out) {
  uint crystal_freq_khz = clock_get_hz(clk_ref) / 1000;
  for (uint fbdiv = 320; fbdiv >= 16; fbdiv--) {
    uint vco = fbdiv * crystal_freq_khz;
    if (vco < 400000 || vco > 1600000) continue;
    for (uint postdiv1 = 7; postdiv1 >= 1; postdiv1--) {
      for (uint postdiv2 = postdiv1; postdiv2 >= 1; postdiv2--) {
        uint out = vco / (postdiv1 * postdiv2);
        if (out == freq_khz && !(vco % (postdiv1 * postdiv2))) {
          *vco_out = vco * 1000;
          *postdiv1_out = postdiv1;
          *postdiv_out = postdiv2;
          return true;
        }
      }
    }
  }
  return false;
}
static inline bool set_sys_clock_khz(uint32_t freq_khz, bool required) {
  uint vco, postdiv1, postdiv2;
  if (check_sys_clock_khz(freq_khz, &vco, &postdiv1, &postdiv2)) {
    set_sys_clock_pll(vco, postdiv1, postdiv2);
    return true;
  } else if (required) {
    panic("System clock of %u kHz cannot be exactly achieved", freq_khz);
  }
  return false;
}


void power_save(uint16_t psinterval){
LoRa.sleep();
delay(2);
set_sys_clock_khz(10000, false); // Set System clock to 10 MHz, sys_clock_khz(10000, true); did not work for me
delay(2);
//vreg_set_voltage(VREG_VOLTAGE_0_95); // 0.85V did not work, 0.95V seems pretty stable
delay(psinterval);
//vreg_set_voltage(VREG_VOLTAGE_DEFAULT); // corresponds to 1.10V, not sure if that is really required for 48 MHz
delay(2);
//set_sys_clock_48mhz(); // Set System clock back to 48 MHz to make LoRa work
delay(2);
LoRa.idle();
}


void rgb_led(uint16_t DELAYVAL){
   
    pixels.clear(); // Set all pixel colors to 'off'

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    for (int j = 0; j < 255; j += 2) {
   // pixels.setPixelColor(i, pixels.Color(j,i*256 + j, j));
    pixels.setPixelColor(random(0, 7), random(0, 255), random(0, 255), random(0, 255));
    pixels.setBrightness(50);
    pixels.show();   // Send the updated pixel colors to the hardware.
    
    delay(DELAYVAL); // Pause before next pass through loop
    }
  }
  
  }
void turnOffUsb() {
// CLK USB = 0MHz
    clock_stop(clk_usb);
    pll_deinit(pll_usb);
}

void turnOffADC() {
    // CLK ADC = 0MHz
    clock_stop(clk_adc);
}



//
//void updateReadings()
//{
//  if (! bme.performReading()) {
//    Serial.println("Failed to perform reading :(");
//    return;
//  }
//  float temperature = bme.temperature;
//  float humidity = bme.humidity;
//  float pressure = bme.pressure / 100.0;
//  float gas= bme.gas_resistance / 1000.0;
//
//  String payload = buildPayload(temperature, humidity, pressure, gas);
//  // Serial.println("Payload: " + payload); // display the payload for debugging
//
//  displayResults(temperature, humidity, pressure, gas); // display the results for debugging
//}


//void displayResults(float t, float h, float p, int r)
//{
//  Serial.print("Temperature: ");
//  Serial.println(t);
//  Serial.print("Humidity: ");
//  Serial.println(h);
//  Serial.print("Pressure: ");
//  Serial.println(p);
//  Serial.print("Gas Resistance ");
//  Serial.print(r);
//  Serial.println("———-");
//}
//
//String buildPayload(float t, float h, float p, int r)
//{
//  String readings = "";
//  readings += t;
//  readings += DELIMITER;
//  readings += h;
//  readings += DELIMITER;
//  readings += p;
//  readings += DELIMITER;
//  readings += r;
//  readings += DELIMITER;
//
//
//  String payload = "";
//  payload += "AT+SEND=";
//  payload += ADDRESS;
//  payload += ",";
//  payload += readings.length();
//  payload += ",";
//  payload += readings;
//  payload += "\r\n";
//
//  return payload;
//}
