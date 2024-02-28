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
#include "LoRaWan-Arduino.h" //http://librarymanager/All#SX126x
#include <SPI.h>

#include <stdio.h>
#include "Adafruit_BME680.h"
#include <Adafruit_Sensor.h>
#include <RP2040_SD.h>
#include <SparkFun_RV8803.h>

#include "mbed.h"
#include "rtos.h"
#include <avr/dtostrf.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 8

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

using namespace std::chrono_literals;
using namespace std::chrono;

Adafruit_BME680 bme; // I2C
RV8803 rtc; // I2C

#define LEDPIN   24
#define DONEPIN  23

///distance measurement defines
#define TRIGGER_PIN 6
#define ECHO_PIN 9

#define USONIC_DIV 58.0
#define MEASURE_SAMPLE_DELAY 5
#define MEASURE_SAMPLES 20
#define MEASURE_DELAY 25
//end //

//Code for 1203 // 
uint8_t sensor1[8] = {0x28, 0x77, 0x4C, 0x44, 0xD4, 0xE1, 0x3C, 0x0E};
uint8_t sensor2[8] = {0x28, 0x43, 0xD9, 0x44, 0xD4, 0xE1, 0x3C, 0x5A};
uint8_t sensor3[8] = {0x28, 0xC8, 0x06, 0x44, 0xD4, 0xE1, 0x3C, 0x79};
uint8_t sensor4[8] = {0x28, 0xD7, 0xC3, 0x44, 0xD4, 0xE1, 0x3C, 0xF6};
uint8_t sensor5[8] = {0x28, 0x5C, 0xFC, 0x44, 0xD4, 0xE1, 0x3C, 0x0F};
uint8_t sensor6[8] = {0x28, 0xB4, 0x4E, 0x44, 0xD4, 0xE1, 0x3C, 0x4B};
uint8_t sensor7[8] = {0x28, 0x93, 0xE7, 0x44, 0xD4, 0xE1, 0x3C, 0xB8};
uint8_t sensor8[8] = {0x28, 0x6F, 0x74, 0x44, 0xD4, 0xE1, 0x3C, 0x4E};
uint8_t sensor9[8] = {0x28, 0x43, 0x68, 0x44, 0xD4, 0xE1, 0x3C, 0xD8};
uint8_t sensor10[8] = {0x28, 0x4F, 0x05, 0x44, 0xD4, 0xE1, 0x3C, 0x58};
uint8_t sensor11[8] = {0x28, 0xE7, 0x96, 0x44, 0xD4, 0xE1, 0x3C, 0x5C};
uint8_t sensor12[8] = {0x28, 0xDE, 0xA0, 0x44, 0xD4, 0xE1, 0x3C, 0x3F};
uint8_t sensor13[8] = {0x28, 0xCD, 0x30, 0x44, 0xD4, 0xE1, 0x3C, 0x8A};
uint8_t sensor14[8] = {0x28, 0xF5, 0x8C, 0x44, 0xD4, 0xE1, 0x3C, 0xA8};
uint8_t sensor15[8] = {0x28, 0x58, 0x5D, 0x44, 0xD4, 0xE1, 0x3C, 0x2D};
uint8_t sensor16[8] = {0x28, 0x05, 0x17, 0x44, 0xD4, 0xE1, 0x3C, 0x1D};
uint8_t sensor17[8] = {0x28, 0xFC, 0xE9, 0x44, 0xD4, 0xE1, 0x3C, 0xFD};
uint8_t sensor18[8] = {0x28, 0xB7, 0x85, 0x44, 0xD4, 0xE1, 0x3C, 0x40};
uint8_t sensor19[8] = {0x28, 0x37, 0xB8, 0x44, 0xD4, 0xE1, 0x3C, 0xC2};
uint8_t sensor20[8] = {0x28, 0x99, 0xDD, 0x44, 0xD4, 0xE1, 0x3C, 0x4E};


File root;
bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60										  /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_3									  /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5							/*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3										  /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;					/* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;				  /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;							        /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);
void lorawan_unconf_finished(void);
void lorawan_conf_finished(bool result);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                          lorawan_rx_handler, lorawan_has_joined_handler,
                                          lorawan_confirm_class_handler, lorawan_join_failed_handler,
                                          lorawan_unconf_finished, lorawan_conf_finished
                                         };
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x56, 0xCD};
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFA};
uint8_t nodeAppKey[16] = {0x88, 0x9F, 0x52, 0x46, 0xD2, 0xB9, 0x53, 0x46, 0x26, 0xE6, 0x55, 0x52, 0x53, 0x12, 0xD8, 0xE6};

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 256                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 500                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.
#define SEALEVELPRESSURE_HPA (1013.25)
char dataStr[100] = "";
char buffer[7];
char buff[] = "20180810T143000Z";

mbed::Ticker appTimer;
void tx_lora_periodic_handler(void);

static uint32_t count = 0;
static uint32_t count_fail = 0;
String data = "";
bool send_now = false;
bool send_complete = false;
float pressure;   
float temperature;  
float altimeter; 
float humidity;
float gasResistance;


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(TRIGGER_PIN, OUTPUT); // Initializing Trigger Output and Echo Input
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIGGER_PIN, LOW); // Reset the trigger pin and wait a half a second
  sensors.begin();  // this is for DS18B20 temp sensor
  Wire.begin();
  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial)
  {
    if ((millis() - timeout) < 5000)
    {
      delay(100);
    }
    else
    {
      break;
    }
  }
  Serial.print("Initializing SD card with SS = ");  Serial.println(PIN_SD_SS);
  Serial.print("SCK = ");   Serial.println(PIN_SD_SCK);
  Serial.print("MOSI = ");  Serial.println(PIN_SD_MOSI);
  Serial.print("MISO = ");  Serial.println(PIN_SD_MISO);

  if (!SD.begin(PIN_SD_SS)) 
  {
    Serial.println("Initialization failed in SD Card!");
    return;
  }
   

  if (rtc.begin() == false)
  {
    Serial.println("RTC RB8803 Device not found. Please check wiring. Freezing.");
    while(1);
  }
  Serial.println("RTC online!");



  // Initialize LoRa chip.
  lora_rak11300_init();

  Serial.println("=====================================");
  Serial.println("Welcome to RAK11300 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
    case LORAMAC_REGION_CN779:
      Serial.println("Region: CN779");
      break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
    case LORAMAC_REGION_RU864:
      Serial.println("Region: RU864");
      break;
    case LORAMAC_REGION_AS923_2:
      Serial.println("Region: AS923-2");
      break;
    case LORAMAC_REGION_AS923_3:
      Serial.println("Region: AS923-3");
      break;
    case LORAMAC_REGION_AS923_4:
      Serial.println("Region: AS923-4");
      break;
  }
  Serial.println("=====================================");

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
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();
  Serial.println("Initialization done.");
}

void loop()
{
  // Every LORAWAN_APP_INTERVAL milliseconds send_now will be set
  digitalWrite(LEDPIN, HIGH);
  String dataString = "";
  dataStr[0] = 0;
  sensors.requestTemperatures();
  

//data = "Temp = " + String(temperature) + " " + " humidity = " + String(humidity) + " " + " Pr = " + String(pressure) + " "+ " gasResistance = " + String(gasResistance) + " " + " Alt = " + String(altimeter) + " " + "Distance = " + String(distance);

data = "Temperature at 1inch: " + String(sensors.getTempC(sensor1)) + ", Temperature at 2inch: " + String(sensors.getTempC(sensor2)) + ", Temperature at 3inch: " + String(sensors.getTempC(sensor3)) + ", Temperature at 4inch: " + String(sensors.getTempC(sensor4)) +
", Temperature at 5inch: " + String(sensors.getTempC(sensor5)) + ", Temperature at 6inch: " + String(sensors.getTempC(sensor6)) + ", Temperature at 7inch: " + String(sensors.getTempC(sensor7)) + ", Temperature at 8inch: " + String(sensors.getTempC(sensor8)) +    
", Temperature at 9inch: " + String(sensors.getTempC(sensor9)) +  ", Temperature at 10inch: " + String(sensors.getTempC(sensor10)) + ", Temperature at 11inch: " + String(sensors.getTempC(sensor11)) + ", Temperature at 12inch: " + String(sensors.getTempC(sensor12)) +  
", Temperature at 13inch: " + String(sensors.getTempC(sensor13)) +  ", Temperature at 14inch: " + String(sensors.getTempC(sensor14)) + ", Temperature at 15inch: " + String(sensors.getTempC(sensor15)) + ", Temperature at 16inch: " + String(sensors.getTempC(sensor16)) + 
", Temperature at 17inch: " + String(sensors.getTempC(sensor17)) +  ", Temperature at 18inch: " + String(sensors.getTempC(sensor18)) + ", Temperature at 19inch: " + String(sensors.getTempC(sensor19)) + ", Temperature at 20inch: " + String(sensors.getTempC(sensor20));  

if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    sprintf(buff, "%02d/%02d/%02d %02d:%02d:%02d", rtc.getYear(),rtc.getMonth(), rtc.getWeekday(), rtc.getHours(), rtc.getMinutes(),rtc.getSeconds());
  }
  else
  {
    Serial.print("RTC read failed");
  }



delay(1000);

#define fileName  "DATALOG.txt"
File dataFile = SD.open("DATALOG.txt", FILE_WRITE);

  // if the file is available, write to it:
if (dataFile) 
  {
    Serial.print("Inside SD ");
    dataFile.print(buff);
    dataFile.print(" : ");
    dataFile.println(data);
    dataFile.close();
    // print to the serial port too:
    Serial.print(buff);
    Serial.print(" : ");
    Serial.println(data);
    //Serial.println(dataStr);

    delay(1000);
   
  }
  // if the file isn't open, pop up an error:
else 
  {
    Serial.print("Error opening "); Serial.println(fileName);
  }
  

delay(1000);

if (send_now)
  {
    Serial.println("Sending frame now...");
    send_now = false;
    send_lora_frame();
       
  }
else {


}
delay(3000);

if(send_complete){
  Serial.println("Send Complete!");
  send_complete =false; 
  digitalWrite(LEDPIN, LOW);
  digitalWrite(DONEPIN, HIGH);
  }

else {
  Serial.println("Error: Unable to send!");
  digitalWrite(LEDPIN, HIGH);
  digitalWrite(DONEPIN, LOW);
}  
  
}

/**@brief LoRa function for handling HasJoined event.
*/
void lorawan_has_joined_handler(void)
{
  if (doOTAA == true)
  {
    Serial.println("OTAA Mode, Network Joined!");
  }
  else
  {
    Serial.println("ABP Mode");
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
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}
/**@brief Function for handling LoRaWan received data from Gateway

   @param[in] app_data  Pointer to rx data
*/
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
                app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void lorawan_unconf_finished(void)
{
  Serial.println("TX finished");
}

void lorawan_conf_finished(bool result)
{
  Serial.printf("Confirmed TX %s\n", result ? "success" : "failed");
}

void send_lora_frame(void)
{
//   uint16_t Tm = 0;
//   uint16_t Pr = 0;
//   uint16_t Alt = 0;
//   uint16_t Hm = 0;
//   uint16_t gR = 0;
//   long Distance = 0; 
//   if (! bme.performReading()) {
//     Serial.println("Failed to perform reading of BME 680x in LoRa Send :(");
//     return;
//   }
//  Pr = bme.pressure/100.0;
//  Tm = bme.temperature;
//  Hm = bme.humidity;
//  gR= bme.gas_resistance / 1000.0;
//  Alt = bme.readAltitude(SEALEVELPRESSURE_HPA); 
//  Distance = measure();

sensors.requestTemperatures();

uint16_t temp1 = sensors.getTempC(sensor1)*100; 
uint16_t temp2 = sensors.getTempC(sensor2)*100; 
uint16_t temp3 = sensors.getTempC(sensor3)*100; 
uint16_t temp4 = sensors.getTempC(sensor4)*100; 
uint16_t temp5 = sensors.getTempC(sensor5)*100; 
uint16_t temp6 = sensors.getTempC(sensor6)*100; 
uint16_t temp7 = sensors.getTempC(sensor7)*100; 
uint16_t temp8 = sensors.getTempC(sensor8)*100; 
uint16_t temp9 = sensors.getTempC(sensor9)*100; 
uint16_t temp10 = sensors.getTempC(sensor10)*100; 
uint16_t temp11 = sensors.getTempC(sensor11)*100; 
uint16_t temp12 = sensors.getTempC(sensor12)*100; 
uint16_t temp13 = sensors.getTempC(sensor13)*100; 
uint16_t temp14 = sensors.getTempC(sensor14)*100; 
uint16_t temp15 = sensors.getTempC(sensor15)*100; 
uint16_t temp16 = sensors.getTempC(sensor16)*100; 
uint16_t temp17 = sensors.getTempC(sensor17)*100; 
uint16_t temp18 = sensors.getTempC(sensor18)*100; 
uint16_t temp19 = sensors.getTempC(sensor19)*100; 
uint16_t temp20 = sensors.getTempC(sensor20)*100; 


  if (lmh_join_status_get() != LMH_SET)
  {
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = 0x01;
  m_lora_app_data.buffer[i++] = (uint8_t)((temp1& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp1& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp2& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp2& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp3& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp3& 0x00FF);  
  m_lora_app_data.buffer[i++] = (uint8_t)((temp4& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp4& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp5& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp5& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp6& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp6& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp7& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp7& 0x00FF);  
  m_lora_app_data.buffer[i++] = (uint8_t)((temp8& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp8& 0x00FF); 
  m_lora_app_data.buffer[i++] = (uint8_t)((temp9& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp9& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp10& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp10& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp11& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp11& 0x00FF);  
  m_lora_app_data.buffer[i++] = (uint8_t)((temp12& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp12& 0x00FF); 
  m_lora_app_data.buffer[i++] = (uint8_t)((temp13& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp13& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp14& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp14& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp15& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp15& 0x00FF);  
  m_lora_app_data.buffer[i++] = (uint8_t)((temp16& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp16& 0x00FF); 
  m_lora_app_data.buffer[i++] = (uint8_t)((temp17& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp17& 0x00FF);
  m_lora_app_data.buffer[i++] = (uint8_t)((temp18& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp18& 0x00FF);  
  m_lora_app_data.buffer[i++] = (uint8_t)((temp19& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp19& 0x00FF); 
  m_lora_app_data.buffer[i++] = (uint8_t)((temp20& 0xFF00) >> 8);
  m_lora_app_data.buffer[i++] = (uint8_t)(temp20& 0x00FF); 
  m_lora_app_data.buffsize = i;
  Serial.printf("Data %d\n", m_lora_app_data);
  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS)
  {
    count++;
    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
  send_complete =true;
}

/**@brief Function for handling user timerout event.
*/
void tx_lora_periodic_handler(void)
{
  appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  // This is a timer interrupt, do not do lengthy things here. Signal the loop() instead
  send_now = true;
}

// distnace measure function 

long measure()
{
long measureSum = 0;
  for (int i = 0; i < MEASURE_SAMPLES; i++)
    {
      delay(MEASURE_SAMPLE_DELAY);   //avg measurement
      measureSum += singleMeasurement();
    }
  return measureSum / MEASURE_SAMPLES;
}

long singleMeasurement()
{
  long duration = 0; // Measure: Put up Trigger...
  digitalWrite(TRIGGER_PIN, HIGH); // Wait for 11 µs ...
  delayMicroseconds(11); // Put the trigger down ...
  digitalWrite(TRIGGER_PIN, LOW); // Wait for the echo ...
  duration = pulseIn(ECHO_PIN, HIGH);
  return (long) (((float) duration / USONIC_DIV) * 10.0);
}


void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  Serial.print(tempC);
  Serial.write(0xC2); Serial.write(0xB0);
  Serial.print("C  |  ");
  Serial.print(DallasTemperature::toFahrenheit(tempC));
  Serial.write(0xC2); Serial.write(0xB0);
  Serial.println("F");
}
